#!/usr/bin/env python3
"""Collect BNO055 IMU data from an ESP32 over serial and plot it in real time.

The ESP32 firmware (IMU_BNO055.ino) streams one CSV line per sample at 100 Hz:

    t_ms,euler_x,...,cal_mag        (24 fields, see FIELDS below)

This script
  * reads the serial port in a background thread,
  * appends every sample to data/imu_data_YYYYMMDD_HHMMSS.csv immediately,
  * renders all channels as live rolling-window subplots (~10 s window).

Usage:
    python imu_collect_plot.py --port COM5 [--baud 921600]

Dependencies:
    pip install pyserial matplotlib
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
PLOT_FPS = 10          # plot refresh rate - kept modest to limit CPU usage
MAX_PLOT_POINTS = 300  # max points per line on screen; display-only decimation,
                       # the CSV still stores every sample at full 100 Hz
AUTOSCALE_EVERY = 4    # re-fit Y axes every N frames (relim/autoscale is costly)


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

    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=1)
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
        self.reader_thread: threading.Thread | None = None
        self.timer = None
        self._frame_count = 0

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
            # Store immediately - one row per sample, flushed so a crash loses nothing.
            self._csv_writer.writerow(parts)
            self._csv_file.flush()

    # -- real-time plot ---------------------------------------------------------
    def build_figure(self) -> None:
        self.fig, axes = plt.subplots(
            len(GROUPS), 1, figsize=(12, 14), sharex=True)
        self.axes = list(axes)
        self.lines: list[tuple[int, "Line2D"]] = []
        for ax, (title, channels) in zip(self.axes, GROUPS):
            ax.set_title(title, fontsize=9, loc="left")
            ax.grid(True, alpha=0.3)
            for ch in channels:
                (line,) = ax.plot([], [], label=ch, linewidth=1.0)
                self.lines.append((IDX[ch], line))
            if len(channels) > 1:
                ax.legend(fontsize=7, ncol=len(channels), loc="upper right")
        self.axes[-1].set_xlabel("Time (s)")
        plt.tight_layout(rect=(0, 0, 1, 0.985))

        # Update the plot from a canvas timer that runs inside the GUI event
        # loop - keeps the window responsive and works with any backend.
        self.timer = self.fig.canvas.new_timer(interval=int(1000.0 / PLOT_FPS))
        self.timer.add_callback(self._update_frame)
        self.timer.start()

    def _update_frame(self) -> None:
        with self.lock:
            snap = list(self.samples)
            total = self.total_samples
        if snap:
            # Decimate for display only (the CSV keeps every sample): limits
            # each line to ~MAX_PLOT_POINTS points so redraws stay cheap.
            step = max(1, len(snap) // MAX_PLOT_POINTS)
            view = snap[::step]
            x_s = [s[0] / 1000.0 for s in view]
            x_max = x_s[-1]

            # 1. Update all line data.
            for idx, line in self.lines:
                line.set_data(x_s, [s[idx] for s in view])

            # 2. Auto-fit each subplot's Y axis (min/max). Throttled to every
            #    AUTOSCALE_EVERY frames: relim()/autoscale_view() is the most
            #    expensive part of a frame and 10 Hz refits look continuous.
            if self._frame_count % AUTOSCALE_EVERY == 0:
                for ax in self.axes:
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)

            # 3. Rolling X range (sharex=True -> setting the last axis syncs all).
            self.axes[-1].set_xlim(
                max(0.0, x_max - WINDOW_S), max(WINDOW_S, x_max))
        self._frame_count += 1
        elapsed = time.monotonic() - self.start_time
        rate = total / elapsed if elapsed > 0 else 0.0
        self.fig.suptitle(
            f"IMU BNO055 - {total} samples | {rate:.1f} Hz | "
            f"CSV: {self.csv_path.name}", fontsize=10)
        self.fig.canvas.draw_idle()

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
    ap.add_argument("--baud", type=int, default=921600,
                    help="Baud rate; must match the firmware (default 921600)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No serial port found. Connect the ESP32 and pass --port COMx.")
    if not args.port:
        print(f"Auto-detected serial port {port} (override with --port).")

    try:
        collector = ImuCollector(port, args.baud)
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
