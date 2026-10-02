#!/usr/bin/env python3
"""Visualize BNO055 IMU data recorded by imu_collect_plot.py.

Plots all channels from a CSV file as stacked subplots with an interactive,
zoomable time axis.

Zoom / navigation:
  * Mouse wheel over the plot ....... zoom the TIME axis in/out around cursor
  * Toolbar "Zoom" (magnifier) ...... box-zoom both axes to a region
  * Toolbar "Pan" (hand) ............ drag to move
  * Toolbar "Home" or press 'r' ..... reset view to full range

Usage:
    python imu_plot_csv.py                          # newest CSV in ./data
    python imu_plot_csv.py data\\imu_data_20261002_131142.csv
    python imu_plot_csv.py --save figure.png        # export instead of GUI

Dependencies:
    pip install matplotlib
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path



try:
    import matplotlib
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'matplotlib'. Install it with:  pip install matplotlib")

# Respect an explicit MPLBACKEND override; otherwise prefer a GUI backend.
if not os.environ.get("MPLBACKEND"):
    try:
        import tkinter  # noqa: F401 - only used to detect Tk availability
        matplotlib.use("TkAgg")
    except ImportError:
        pass

import matplotlib.pyplot as plt

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

MIN_ZOOM_S = 0.02   # closest the time axis can be zoomed in (s)


def find_latest_csv() -> Path | None:
    """Newest CSV file in ./data next to this script."""
    data_dir = Path(__file__).resolve().parent / "data"
    if not data_dir.is_dir():
        return None
    files = sorted(data_dir.glob("imu_data_*.csv"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def load_csv(path: Path) -> list[tuple[float, ...]]:
    """Read the CSV and return a list of 24-field float tuples (skips bad rows)."""
    samples: list[tuple[float, ...]] = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) != NUM_FIELDS:
                continue
            try:
                samples.append(tuple(float(v) for v in row))
            except ValueError:
                continue  # header or malformed line
    if not samples:
        sys.exit(f"No valid data rows found in {path}")
    return samples


def build_figure(samples: list[tuple[float, ...]], csv_path: Path):
    """Create the figure; returns (fig, x0, x1) where [x0, x1] is the full range."""
    t0 = samples[0][IDX["t_ms"]]
    # Time axis in seconds relative to the first sample.
    x_all = [(s[IDX["t_ms"]] - t0) / 1000.0 for s in samples]
    x0, x1 = x_all[0], x_all[-1]

    fig, axes = plt.subplots(len(GROUPS), 1, figsize=(12, 14), sharex=True)
    for ax, (title, channels) in zip(axes, GROUPS):
        ax.set_title(title, fontsize=9, loc="left")
        ax.grid(True, alpha=0.3)
        for ch in channels:
            i = IDX[ch]
            ax.plot(x_all, [s[i] for s in samples], label=ch, linewidth=0.8)
        if len(channels) > 1:
            ax.legend(fontsize=7, ncol=len(channels), loc="upper right")
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"{csv_path.name} - {len(samples)} samples | "
                 f"visible range shown below", fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.985))

    # Keep the suptitle in sync with the visible (zoomed) time window.
    def update_title(_event=None):
        lo, hi = axes[-1].get_xlim()
        fig.suptitle(
            f"{csv_path.name} - {len(samples)} samples | "
            f"visible: {lo:.2f} s .. {hi:.2f} s  (wheel = zoom time axis)",
            fontsize=10)

    fig.canvas.mpl_connect("draw_event", update_title)
    return fig, x0, x1


def make_scroll_zoom(fig, axes, x0: float, x1: float):
    """Mouse-wheel zoom of the shared TIME axis, centered on the cursor."""
    factor = 1.25

    def on_scroll(event):
        if event.inaxes is None or event.xdata is None:
            return
        # Zoom in (wheel up) shrinks the range; wheel out grows it.
        scale = 1.0 / factor if event.button == "up" else factor
        lo, hi = axes[-1].get_xlim()
        center = float(event.xdata)
        half = (hi - lo) * scale / 2.0

        # Clamp: never zoom in closer than MIN_ZOOM_S, never out past full range.
        if (hi - lo) * scale < MIN_ZOOM_S or (hi - lo) * scale > (x1 - x0) * 1.05:
            return
        new_lo = center - half
        new_hi = center + half
        if new_lo < x0:
            new_hi += x0 - new_lo
            new_lo = x0
        if new_hi > x1:
            new_lo -= new_hi - x1
            new_hi = x1
        axes[-1].set_xlim(new_lo, new_hi)  # sharex=True -> all subplots follow
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("scroll_event", on_scroll)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Plot BNO055 IMU CSV data with a zoomable time axis.")
    ap.add_argument("file", nargs="?", default=None,
                    help="CSV file to plot (default: newest in ./data)")
    ap.add_argument("--save", metavar="PNG", default=None,
                    help="Save the figure to this PNG file instead of showing a window")
    args = ap.parse_args()

    path = Path(args.file) if args.file else find_latest_csv()
    if not path or not path.is_file():
        sys.exit("No CSV file found. Pass one as an argument, e.g.\n"
                 "  python imu_plot_csv.py data\\imu_data_20261002_131142.csv")

    print(f"Loading {path} ...")
    samples = load_csv(path)
    fig, x0, x1 = build_figure(samples, path)
    make_scroll_zoom(fig, fig.axes, x0, x1)

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"Saved figure to {args.save}")
        plt.close("all")
        return

    backend = plt.get_backend().lower()
    if backend in {"agg", "pdf", "svg", "ps"}:
        sys.exit(
            f"Matplotlib has no GUI backend available (using '{backend}'), "
            "so no plot window can appear.\n"
            "Fix: make sure tkinter is installed with your Python, or use --save out.png."
        )

    from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
    fig.canvas.manager.set_window_title(path.name)
    NavigationToolbar2Tk(fig.canvas)  # adds the Zoom (box) / Pan / Home toolbar

    print(f"{len(samples)} samples | {x1 - x0:.1f} s")
    print("Scroll wheel = zoom time axis | toolbar: Zoom (box), Pan, Home (reset)")
    plt.show()


if __name__ == "__main__":
    main()
