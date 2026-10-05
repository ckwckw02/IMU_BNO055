#!/usr/bin/env python3
"""Collect BNO055 IMU data from an ESP32 over serial and plot it in real time.

The ESP32 firmware (IMU_BNO055.ino) streams one CSV line per sample at 100 Hz:

    t_ms,euler_x,...,cal_mag        (24 fields, see FIELDS below)

This script
  * reads the serial port in a background thread,
  * appends every sample to data/imu_data_YYYYMMDD_HHMMSS.csv immediately,
  * renders all channels as live rolling-window subplots (~10 s window),
    redrawn with numpy + TkAgg blitting for high-FPS updates (default 30).

Usage:
    python imu_collect_plot.py --port COM5 [--baud 921600] [--fps 30]

Dependencies:
    pip install pyserial matplotlib numpy
"""

from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # only for type annotations, not imported at runtime
    from matplotlib.lines import Line2D

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'numpy'. Install it with:  pip install numpy")

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'pyserial'. Install it with:  pip install pyserial")

import matplotlib

# Prefer a GUI backend so the live plot window actually appears.
try:
    import tkinter  # noqa: F401 - only used to detect Tk availability
    matplotlib.use("TkAgg")
except ImportError:
    pass

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'matplotlib'. Install it with:  pip install matplotlib")

if plt.get_backend().lower() in {"agg", "pdf", "svg", "ps"}:
    sys.exit(
        f"Matplotlib has no GUI backend available (using '{plt.get_backend()}'), "
        "so no plot window can appear.\n"
        "Fix: make sure tkinter is installed with your Python (python.org "
        "installer -> check 'tcl/tk and IDLE'; conda: 'conda install tk'), then re-run,\n"
        "or force a GUI backend before running:\n"
        "    set MPLBACKEND=TkAgg        (Windows cmd)\n"
        "    $env:MPLBACKEND='TkAgg'     (PowerShell)"
    )

# --- Wire protocol (must match IMU_BNO055.ino) -------------------------------
NUM_FIELDS = 24
FIELDS = [
    "t_ms",
    "euler_x", "euler_y", "euler_z",            # deg
    "gyro_x", "gyro_y", "gyro_z",               # deg/s
    "accel_x", "accel_y", "accel_z",            # m/s^2
    "linacc_x", "linacc_y", "linacc_z",         # m/s^2
    "mag_x", "mag_y", "mag_z",                  # uT
    "grav_x", "grav_y", "grav_z",               # m/s^2
    "temp_c",                                   # degC
    "cal_sys", "cal_gyro", "cal_accel", "cal_mag",  # 0..3
]
IDX = {name: i for i, name in enumerate(FIELDS)}

GROUPS = [
    ("Euler angles (deg)", ["euler_x", "euler_y", "euler_z"]),
    ("Gyroscope (deg/s)", ["gyro_x", "gyro_y", "gyro_z"]),
    ("Accelerometer (m/s^2)", ["accel_x", "accel_y", "accel_z"]),
    ("Linear acceleration (m/s^2)", ["linacc_x", "linacc_y", "linacc_z"]),
    ("Magnetometer (uT)", ["mag_x", "mag_y", "mag_z"]),
    ("Gravity (m/s^2)", ["grav_x", "grav_y", "grav_z"]),
    ("Temperature (degC)", ["temp_c"]),
]

WINDOW_S = 10.0        # rolling plot window (seconds)
PLOT_FPS = 30          # default plot refresh rate (override with --fps, up to ~60)
MAX_PLOT_POINTS = 400  # max points per line on screen; display-only decimation,
                       # the CSV still stores every sample at full 100 Hz
AUTOSCALE_EVERY = 6    # re-check Y limits every N frames (numpy min/max is cheap)
FORCE_FULL_EVERY = 30  # unconditional full redraw every N frames (refreshes title)
FLUSH_EVERY = 10       # CSV flush cadence in rows (~0.1 s at 100 Hz)


def make_csv_path() -> Path:
    """New timestamped CSV file per run, stored in ./data next to this script."""
    out_dir = Path(__file__).resolve().parent / "data"
    out_dir.mkdir(exist_ok=True)
    return out_dir / f"imu_data_{datetime.now():%Y%m%d_%H%M%S}.csv"


def find_port() -> str | None:
    """Auto-detect a likely ESP32 serial port."""
    ports = list_ports.comports()
    if not ports:
        return None
    for p in ports:  # prefer common USB-serial chips used with the ESP32
        desc = (p.description or "").lower()
        if any(k in desc for k in ("cp210", "ch340", "esp32")):
            return p.device
    return ports[0].device


class ImuCollector:
    """Reads the ESP32 stream, stores every sample to CSV, feeds the live plot."""

    def __init__(self, port: str, baud: int, fps: int = PLOT_FPS) -> None:
        # Short read timeout so the reader thread notices stop() quickly.
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)
        self.csv_path = make_csv_path()
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(FIELDS)
        self._csv_file.flush()

        # Bounded buffer for the plot (~10 s at 100 Hz); the CSV keeps everything.
        self.samples: deque = deque(maxlen=int(WINDOW_S * 100) + 50)
        self.lock = threading.Lock()
        self.total_samples = 0
        self.start_time = time.monotonic()
        self._stop = threading.Event()
        self.fps = fps
        self.reader_thread: threading.Thread | None = None
        self.timer = None
        self._frame_count = 0
        # Blitting state: cached static canvas (axes/grid/labels) that we
        # restore each frame and repaint only the data lines on top of it.
        self.background = None
        self._resized = False
        self._rows_since_flush = 0

    # -- serial reader (background thread) ------------------------------------
    def reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                line = self.ser.readline()
            except OSError:  # port closed or device unplugged
                break
            if not line:
                continue
            parts = line.decode("ascii", errors="ignore").strip().split(",")
            if len(parts) != NUM_FIELDS:
                continue  # header / partial line
            try:
                values = tuple(float(p) for p in parts)
            except ValueError:
                continue  # non-numeric line (e.g. the ESP32 CSV header)
            with self.lock:
                self.samples.append(values)
                self.total_samples += 1
            # Store immediately - one row per sample; flush every FLUSH_EVERY
            # rows so a crash loses at most ~0.1 s while I/O stays cheap.
            self._csv_writer.writerow(parts)
            self._rows_since_flush += 1
            if self._rows_since_flush >= FLUSH_EVERY:
                self._rows_since_flush = 0
                self._csv_file.flush()

    # -- real-time plot ---------------------------------------------------------
    def build_figure(self) -> None:
        self.fig, axes = plt.subplots(
            len(GROUPS), 1, figsize=(12, 14), sharex=True)
        self.axes = list(axes)
        # Per-axis (column index, Line2D) pairs for fast blitted redraws.
        self.axis_lines: list[tuple["plt.Axes", list[tuple[int, "Line2D"]]]] = []
        for ax, (title, channels) in zip(self.axes, GROUPS):
            ax.set_title(title, fontsize=9, loc="left")
            ax.grid(True, alpha=0.3)
            # Fixed axes: the data scrolls inside a static window, so the
            # grid, ticks and labels can be cached once and blitted per frame.
            ax.autoscale(enable=False)
            ax.set_xlim(0.0, WINDOW_S)
            lines = []
            for ch in channels:
                (line,) = ax.plot([], [], label=ch, linewidth=1.0)
                lines.append((IDX[ch], line))
            self.axis_lines.append((ax, lines))
            if len(channels) > 1:
                ax.legend(fontsize=7, ncol=len(channels), loc="upper right")
        self.axes[-1].set_xlabel("Time (s)")
        plt.tight_layout(rect=(0, 0, 1, 0.985))

        # Status line; refreshed on full redraws (see _full_redraw).
        self.suptitle = self.fig.suptitle("", fontsize=10)
        # A resize invalidates the cached background -> full redraw next frame.
        self.fig.canvas.mpl_connect(
            "resize_event", lambda e: setattr(self, "_resized", True))

        # Prime the cached background (empty plot) before starting updates.
        self._full_redraw()

        # Update the plot from a canvas timer that runs inside the GUI event
        # loop - keeps the window responsive and works with any backend.
        self.timer = self.fig.canvas.new_timer(interval=int(1000.0 / self.fps))
        self.timer.add_callback(self._update_frame)
        self.timer.start()

    def _update_frame(self) -> None:
        with self.lock:
            snap = list(self.samples)
        if not snap:
            return
        # Vectorized rolling-window view (display-only decimation; the CSV
        # keeps every sample at full 100 Hz). Data is re-based so it always
        # spans [0, WINDOW_S] inside the fixed axes.
        arr = np.asarray(snap, dtype=np.float64)
        step = max(1, len(arr) // MAX_PLOT_POINTS)
        view = arr[::step]
        t0_ms = max(0.0, float(view[-1, IDX["t_ms"]]) - WINDOW_S * 1000.0)
        x_s = (view[:, IDX["t_ms"]] - t0_ms) / 1000.0

        self._frame_count += 1
        full_redraw = (
            self.background is None
            or self._resized
            or self._frame_count % FORCE_FULL_EVERY == 0
        )
        if not full_redraw and self._frame_count % AUTOSCALE_EVERY == 0:
            # Cheap numpy min/max; only touch the axes (and force a redraw)
            # when the data range actually moved beyond a small tolerance.
            if self._y_limits_need_update(view):
                full_redraw = True
        if full_redraw:
            self._fit_y_limits(view)
            self._full_redraw()

        # Blit: restore the cached static background, repaint only the lines.
        try:
            self.fig.canvas.restore_region(self.background)
            for ax, lines in self.axis_lines:
                for idx, line in lines:
                    line.set_data(x_s, view[:, idx])
                    ax.draw_artist(line)
            self.fig.canvas.blit(self.fig.bbox)
        except Exception:  # backend hiccup -> fall back to a plain full draw
            self.background = None
            self.fig.canvas.draw_idle()

    def _fit_y_limits(self, view: np.ndarray) -> None:
        """Set each subplot's Y range from the current window (+5% padding)."""
        for ax, (title, channels) in zip(self.axes, GROUPS):
            cols = view[:, [IDX[c] for c in channels]]
            lo, hi = float(cols.min()), float(cols.max())
            if hi - lo < 1e-9:
                lo, hi = lo - 0.5, hi + 0.5
            pad = (hi - lo) * 0.05
            ax.set_ylim(lo - pad, hi + pad)

    def _y_limits_need_update(self, view: np.ndarray) -> bool:
        """True if any channel left the current Y range by more than ~10%."""
        for ax, (title, channels) in zip(self.axes, GROUPS):
            cols = view[:, [IDX[c] for c in channels]]
            lo, hi = float(cols.min()), float(cols.max())
            cur_lo, cur_hi = ax.get_ylim()
            span = max(abs(cur_hi - cur_lo), 1e-9)
            if lo < cur_lo - 0.1 * span or hi > cur_hi + 0.1 * span:
                return True
        return False

    def _full_redraw(self) -> None:
        """Full canvas draw and re-capture of the static background."""
        self._resized = False
        elapsed = time.monotonic() - self.start_time
        rate = self.total_samples / elapsed if elapsed > 0 else 0.0
        self.suptitle.set_text(
            f"IMU BNO055 - {self.total_samples} samples | {rate:.1f} Hz | "
            f"CSV: {self.csv_path.name}")
        # Hide the dynamic lines while capturing so they are not baked in.
        for _, lines in self.axis_lines:
            for _, line in lines:
                line.set_visible(False)
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        for _, lines in self.axis_lines:
            for _, line in lines:
                line.set_visible(True)

    def stop(self) -> None:
        self._stop.set()
        if self.timer is not None:
            self.timer.stop()
        try:
            self.ser.close()
        except OSError:
            pass
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=2.0)
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except OSError:
            pass
        plt.close("all")
        elapsed = time.monotonic() - self.start_time
        rate = self.total_samples / elapsed if elapsed > 0 else 0.0
        print(f"\nSaved {self.total_samples} samples in {elapsed:.1f} s "
              f"({rate:.1f} Hz) to:\n  {self.csv_path}")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Collect BNO055 IMU data from an ESP32 and plot it live.")
    ap.add_argument("--port", default=None,
                    help="Serial port (e.g. COM5). Default: auto-detect.")
    ap.add_argument("--baud", type=int, default=115200,
                    help="Baud rate; must match the firmware (default 115200)")
    ap.add_argument("--fps", type=int, default=PLOT_FPS,
                    help=f"Plot refresh rate in FPS (default {PLOT_FPS}, "
                         "try up to ~60 on a fast machine)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No serial port found. Connect the ESP32 and pass --port COMx.")
    if not args.port:
        print(f"Auto-detected serial port {port} (override with --port).")

    try:
        collector = ImuCollector(port, args.baud, args.fps)
    except OSError as exc:  # includes serial.SerialException
        sys.exit(f"Could not open {port}: {exc}")

    print(f"Streaming from {port} @ {args.baud} baud -> {collector.csv_path}")
    print("Stop with Ctrl+C (or close the plot window).")

    collector.reader_thread = threading.Thread(
        target=collector.reader_loop, name="serial-reader", daemon=True)
    collector.reader_thread.start()

    try:
        collector.build_figure()
        plt.show()  # blocks until the window is closed or Ctrl+C is pressed
    except KeyboardInterrupt:
        pass
    finally:
        collector.stop()


if __name__ == "__main__":
    main()
