#!/usr/bin/env python3
"""Gait-phase analysis for a shank-mounted BNO055 IMU (ESP32).

Phase 2 of the "gait-phase FSM assist" plan. Reads a CSV recorded by
``imu_collect_plot.py`` and, from the raw 9-axis stream, derives the signals a
shank exoskeleton needs:

    theta(t)   shank pitch angle about the standing posture (deg)
    omega(t)   shank pitch angular velocity (deg/s)
    a_v(t)     world-vertical linear acceleration (m/s^2) -> foot-strike peaks

and detects gait events / phases. Everything is derived OFFLINE from the raw
gravity + gyro + linear-acceleration vectors, so no firmware change was needed.

Axis convention is auto-detected from the data (robust to how the board was
mounted):
  * shank axis = body axis with the largest |mean gravity| (the ~vertical one)
  * AP axis    = of the two remaining axes, the one that swings most in the
                 gait band (anterior-posterior / forward-back swing)
  * ML axis    = the last one (medial-lateral; pitch rotates about it)

Usage:
    python gait_fsm.py data\\imu_data_..._case3-fast.csv --save analysis\\fast.png
    python gait_fsm.py data\\*.csv --diagnose          # quick per-file figures
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'numpy'. Install it with:  pip install numpy")

try:
    import matplotlib
    if not __import__("os").environ.get("MPLBACKEND"):
        try:
            import tkinter  # noqa: F401
            matplotlib.use("TkAgg")
        except ImportError:
            pass
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'matplotlib'. Install it with:  pip install matplotlib")

# --- Wire protocol (must match IMU_BNO055.ino) -------------------------------
NUM_FIELDS = 24
FIELDS = [
    "t_ms",
    "euler_x", "euler_y", "euler_z",            # deg
    "gyro_x", "gyro_y", "gyro_z",               # deg/s
    "accel_x", "accel_y", "accel_z",            # m/s^2 (raw, gravity included)
    "linacc_x", "linacc_y", "linacc_z",         # m/s^2 (gravity removed)
    "mag_x", "mag_y", "mag_z",                  # uT
    "grav_x", "grav_y", "grav_z",               # m/s^2
    "temp_c",                                   # degC
    "cal_sys", "cal_gyro", "cal_accel", "cal_mag",  # 0..3
]
IDX = {name: i for i, name in enumerate(FIELDS)}

AXES = ("x", "y", "z")


# --- Loading -----------------------------------------------------------------
def load_csv(path: Path) -> np.ndarray:
    """Return an (N, NUM_FIELDS) float array; skips header / malformed rows."""
    import csv
    rows = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) != NUM_FIELDS:
                continue
            try:
                rows.append([float(v) for v in row])
            except ValueError:
                continue
    if not rows:
        sys.exit(f"No valid data rows found in {path}")
    return np.asarray(rows, dtype=np.float64)


def to_frame(arr: np.ndarray) -> dict[str, np.ndarray]:
    """Column arrays plus a time axis in seconds (from the real t_ms stamps)."""
    t0 = arr[:, IDX["t_ms"]][0]
    out = {"t": (arr[:, IDX["t_ms"]] - t0) / 1000.0}
    for name in FIELDS[1:]:
        out[name] = arr[:, IDX[name]]
    return out


# --- Axis auto-detection -----------------------------------------------------
def detect_axes(df: dict[str, np.ndarray]) -> tuple[int, int, int]:
    """Return (shank_idx, ap_idx, ml_idx) into AXES.

    shank = axis with largest |mean gravity|; of the other two, AP is the one
    whose gravity component carries more gait-band energy (it swings forward/
    back), ML is the remainder."""
    grav = np.stack([df[f"grav_{a}"] for a in AXES], axis=1)  # (N,3)
    shank_idx = int(np.argmax(np.abs(grav.mean(axis=0))))

    others = [i for i in range(3) if i != shank_idx]
    # Gait-band energy: band-pass-ish via subtracting a long mean then power.
    def gait_power(sig: np.ndarray, t: np.ndarray) -> float:
        fs = 1.0 / np.median(np.diff(t))
        n = len(sig)
        x = sig - np.mean(sig)
        spec = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
        freqs = np.fft.rfftfreq(n, 1.0 / fs)
        band = (freqs >= 0.3) & (freqs <= 2.5)   # human gait range
        return float(spec[band].sum())

    p = [gait_power(grav[:, i], df["t"]) for i in others]
    ap_idx = others[int(np.argmax(p))]
    ml_idx = others[int(np.argmin(p))]
    return shank_idx, ap_idx, ml_idx


# --- Derived signals ---------------------------------------------------------
def _wrap180(a: np.ndarray) -> np.ndarray:
    return (a + 180.0) % 360.0 - 180.0


def _lp_zero_phase(x: np.ndarray, fs: float, fc: float = 5.0, order: int = 3) -> np.ndarray:
    """Zero-phase low-pass with a smooth Butterworth magnitude (numpy-only).

    No scipy dependency; the smooth |H(f)| avoids the ringing a brick-wall FFT
    filter would introduce right at the sharp foot-strike transients."""
    n = len(x)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    H = 1.0 / np.sqrt(1.0 + (f / max(fc, 1e-9)) ** (2 * order))
    return np.fft.irfft(X * H, n=n)


def derive_signals(df: dict[str, np.ndarray],
                   shank_idx: int | None = None,
                   ap_idx: int | None = None) -> dict[str, np.ndarray]:
    """Compute theta (deg), omega (deg/s), a_v (m/s^2) and the axis indices."""
    if shank_idx is None or ap_idx is None:
        s, a, m = detect_axes(df)
        shank_idx, ap_idx = s, a
    else:
        m = [i for i in range(3) if i not in (shank_idx, ap_idx)][0]

    gs = df[f"grav_{AXES[shank_idx]}"]
    ga = df[f"grav_{AXES[ap_idx]}"]
    t = df["t"]

    # Pitch angle about the standing posture: angle of the gravity vector in
    # the shank-AP plane, minus its mean (so rest ~ 0 deg). Convention-free.
    ang = np.degrees(np.arctan2(ga, gs))
    offset = float(np.median(ang))          # robust to a few odd samples
    theta = _wrap180(ang - offset)

    # Pitch angular velocity: numerical derivative of the low-pass-filtered
    # angle. The BNO055 fused gyro is unreliable in scale here (see analysis),
    # so d(theta)/dt from the gravity-derived pitch is used instead - this is
    # exactly the "shank forward-tilt rate" the assist law RPM = K*omega needs.
    fs = 1.0 / np.median(np.diff(t))
    theta_f = _lp_zero_phase(theta, fs, fc=5.0)   # keep gait band, kill noise
    omega = np.gradient(theta_f, t)

    # World-vertical linear acceleration: project gravity-removed accel onto the
    # instantaneous gravity direction (robust to shank tilt). Sign chosen so a
    # foot-strike impact is a positive peak.
    gvec = np.stack([df[f"grav_{a}"] for a in AXES], axis=1)
    lvec = np.stack([df[f"linacc_{a}"] for a in AXES], axis=1)
    gnorm = np.linalg.norm(gvec, axis=1, keepdims=True) + 1e-9
    a_v = -(lvec * (gvec / gnorm)).sum(axis=1)   # negative -> up is positive

    out = dict(df)
    out.update({"theta": theta, "omega": omega, "a_v": a_v,
                "shank_idx": np.full(len(t), shank_idx),
                "ap_idx": np.full(len(t), ap_idx)})
    return out


# --- Foot-strike detection ---------------------------------------------------
def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    return np.convolve(x, np.ones(n) / n, mode="same")


def detect_foot_strikes(a_v: np.ndarray, t: np.ndarray,
                        k_sigma: float = 2.0, min_sep_s: float = 0.45):
    """Return (strike_times, strike_idx).

    A foot strike is an impact peak in the vertical acceleration. Uses a simple
    local-maximum search above mean + k*std with a minimum time separation of
    ~0.45 s (no single-leg stride is shorter than that), which also rejects the
    double-detections that appear during speed transitions."""
    fs = 1.0 / np.median(np.diff(t))
    av = _smooth(a_v, max(3, int(round(0.03 * fs))))   # ~30 ms
    thr = float(av.mean() + k_sigma * av.std())
    min_sep = int(max(2, round(min_sep_s * fs)))

    idx = []
    n = len(av)
    for i in range(1, n - 1):
        if av[i] > thr and av[i] >= av[i - 1] and av[i] > av[i + 1]:
            if not idx or (i - idx[-1]) >= min_sep:
                idx.append(i)
            elif av[i] > av[idx[-1]]:                 # keep the taller peak
                idx[-1] = i
    return t[np.asarray(idx)], np.asarray(idx)


# --- Plotting ----------------------------------------------------------------
def plot_diagnose(df: dict[str, np.ndarray], path: Path, title: str) -> None:
    s_ax, a_ax = AXES[int(df["shank_idx"][0])], AXES[int(df["ap_idx"][0])]
    t = df["t"]
    st_t, _ = detect_foot_strikes(df["a_v"], t)

    fig, axes = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    ax = axes[0]
    for a in AXES:
        ax.plot(t, df[f"grav_{a}"], label=f"grav_{a}", linewidth=0.8)
    ax.set_title(f"Gravity vector (shank={s_ax}, AP={a_ax})", fontsize=9, loc="left")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=3, loc="upper right")

    ax = axes[1]
    ax.plot(t, df["theta"], color="tab:blue", label="theta (pitch)", linewidth=0.9)
    ax.set_ylabel("deg", color="tab:blue"); ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(t, df["omega"], color="tab:red", alpha=0.7, label="omega (pitch rate)", linewidth=0.8)
    ax2.set_ylabel("deg/s", color="tab:red"); ax2.tick_params(axis="y", labelcolor="tab:red")
    ax.set_title("Shank pitch angle theta & angular velocity omega", fontsize=9, loc="left")
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot(t, df["a_v"], color="tab:green", linewidth=0.8, label="a_v (vertical)")
    for ts in st_t:
        ax.axvline(ts, color="red", alpha=0.4, linewidth=0.7)
    ax.set_title(f"Vertical acceleration + detected foot strikes ({len(st_t)} events)",
                 fontsize=9, loc="left")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="upper right")

    ax = axes[3]
    for a in AXES:
        ax.plot(t, df[f"gyro_{a}"], label=f"gyro_{a}", linewidth=0.8)
    ax.set_title("Gyroscope (deg/s)", fontsize=9, loc="left")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=3, loc="upper right")

    ax = axes[4]
    for a in AXES:
        ax.plot(t, df[f"accel_{a}"], label=f"accel_{a}", linewidth=0.8)
    ax.set_title("Accelerometer raw (m/s^2)", fontsize=9, loc="left")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=3, loc="upper right")

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.985))
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=130)
        print(f"Saved -> {path}")


# --- Real-time (causal) gait-phase FSM ---------------------------------------
# States and the motor action each implies for a shank exoskeleton (per plan):
STATE_MOTOR = {
    "IDLE":     "no assist / free",          # standing still, not walking
    "LOADING":  "absorb - slight tension",   # heel strike impact cushioning
    "STANCE":   "HOLD / zero RPM",           # foot planted, weight over it
    "PUSH-OFF": "HIGH RPM retract (assist)", # toe-off: pull shank forward  <- core assist
    "SWING":    "reverse / release",         # foot in air, swing freely
}
STATE_ORDER = ["IDLE", "LOADING", "STANCE", "PUSH-OFF", "SWING"]


def _causal_rolling_std(x: np.ndarray, w: int) -> np.ndarray:
    """Standard deviation over a causal window of ``w`` samples ending at each i."""
    n = len(x)
    c1 = np.concatenate([[0.0], np.cumsum(x)])
    c2 = np.concatenate([[0.0], np.cumsum(x * x)])
    idx = np.arange(n)
    lo = np.maximum(0, idx - w + 1)
    cnt = (idx - lo + 1).astype(float)
    s = c1[idx + 1] - c1[lo]
    s2 = c2[idx + 1] - c2[lo]
    mean = s / cnt
    var = np.maximum(0.0, s2 / cnt - mean * mean)
    return np.sqrt(var)


def run_fsm(df: dict[str, np.ndarray]) -> tuple[np.ndarray, dict]:
    """Classify every sample into a gait phase using only causal (past) info.

    A short-term "walking" gate (rolling std of the pitch rate over ~2 s) keeps
    the FSM in IDLE while standing still, so random bumps are not misread as
    foot strikes. While walking, transitions are:
      * impact event  : a_v rising edge above mean + k*std   -> LOADING
      * timer         : ~15% of stride after impact          -> STANCE
      * omega up-cross: pitch rate rises past ~0.3*peak      -> PUSH-OFF (assist)
      * omega fall    : pitch rate drops below 0.4*peak      -> SWING

    Returns (states, info); states is a per-sample label array and info holds the
    adaptive thresholds used."""
    t = df["t"]; fs = 1.0 / np.median(np.diff(t))
    theta, omega, av = df["theta"], df["omega"], df["a_v"]
    n = len(t)

    # --- walking gate: rolling std of the pitch rate over ~2 s (causal) -------
    w_win = int(round(2.0 * fs))
    walk_sig = _causal_rolling_std(omega, w_win)
    walk_thr = max(3.0, 0.20 * float(np.percentile(walk_sig, 95)))
    walking = walk_sig > walk_thr

    st_t, _ = detect_foot_strikes(av, t)
    stride_s = float(np.median(np.diff(st_t))) if len(st_t) >= 3 else 1.0
    load_end   = int(round(0.15 * stride_s * fs))     # LOADING -> STANCE timer
    refractory = int(round(0.30 * fs))                # min gap between impacts

    av_thr   = float(av.mean() + 2.0 * av.std())      # impact threshold
    om_peak  = max(float(np.percentile(np.abs(omega), 95)), 1e-6)
    om_up    = max(20.0, 0.30 * om_peak)              # STANCE->PUSH-OFF entry
    om_swing = 0.40 * om_peak                         # PUSH-OFF->SWING exit

    state = "IDLE"
    out = np.empty(n, dtype=object)
    load_timer = 0
    last_impact = -10**9

    for i in range(n):
        if not walking[i]:                            # standing still -> IDLE
            state = "IDLE"; load_timer = 0
            out[i] = state; continue
        impact = (av[i] > av_thr and av[i - 1] <= av_thr
                  and (i - last_impact) >= refractory)
        if impact:
            state, load_timer, last_impact = "LOADING", load_end, i
        elif state == "LOADING":
            load_timer -= 1
            if load_timer <= 0:
                state = "STANCE"
        # STANCE -> PUSH-OFF when pitch rate rises past the entry threshold.
        if state == "STANCE" and omega[i] > om_up and omega[i - 1] <= om_up:
            state = "PUSH-OFF"
        elif state == "PUSH-OFF" and omega[i] < om_swing:
            state = "SWING"
        out[i] = state

    info = dict(stride_s=stride_s, av_thr=av_thr, om_peak=om_peak,
                om_up=om_up, om_swing=om_swing, walk_thr=walk_thr)
    return out, info


def plot_fsm(df: dict[str, np.ndarray], states: np.ndarray, path: Path | None,
             title: str) -> None:
    """Plot theta/omega/a_v with the FSM state track (colored bands) below."""
    t = df["t"]
    st_t, _ = detect_foot_strikes(df["a_v"], t)

    fig, axes = plt.subplots(4, 1, figsize=(13, 9), sharex=True,
                             gridspec_kw={"height_ratios": [2.2, 2.2, 2.2, 1.0]})
    ax = axes[0]
    ax.plot(t, df["theta"], color="tab:blue", linewidth=0.9)
    ax.set_ylabel("theta (deg)", color="tab:blue"); ax.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(t, df["omega"], color="tab:red", alpha=0.75, linewidth=0.8)
    ax2.set_ylabel("omega (deg/s)", color="tab:red"); ax2.tick_params(axis="y", labelcolor="tab:red")
    ax.set_title("Shank pitch theta & angular velocity omega", fontsize=9, loc="left")
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(t, df["a_v"], color="tab:green", linewidth=0.8)
    for ts in st_t:
        ax.axvline(ts, color="red", alpha=0.45, linewidth=0.7)
    ax.set_title(f"Vertical acceleration + foot strikes ({len(st_t)} events)", fontsize=9, loc="left")
    ax.grid(alpha=0.3)

    # State track: one colored band per contiguous run of the same state.
    ax = axes[2]
    colors = {"IDLE": "#cccccc", "LOADING": "#ff9f43", "STANCE": "#1e88e5",
              "PUSH-OFF": "#e53935", "SWING": "#43a047"}
    i = 0
    while i < len(states):
        j = i
        while j + 1 < len(states) and states[j + 1] == states[i]:
            j += 1
        ax.broken_barh([(t[i], t[j] - t[i])], (0, 1), facecolors=colors[states[i]], edgecolor="none")
        i = j + 1
    ax.set_ylim(-0.2, 1.4); ax.set_yticks([])
    handles = [plt.Rectangle((0, 0), 1, 1, fc=colors[s]) for s in STATE_ORDER]
    labels = [f"{s} - {STATE_MOTOR[s]}" for s in STATE_ORDER]
    ax.legend(handles, labels, fontsize=7, ncol=2, loc="upper right", framealpha=0.9)
    ax.set_title("FSM gait-phase state (real-time classification)", fontsize=9, loc="left")

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.985))
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=130)
        print(f"Saved -> {path}")


def plot_stride_overlay(df: dict[str, np.ndarray], path: Path | None, title: str) -> None:
    """Align every step by its foot-strike and overlay one normalized stride.

    Each panel shows theta / omega / a_v re-based so that 0 = this step's
    foot strike and 1 = the next foot strike. Overlaid thin lines are the
    individual steps; the thick line is their mean - the consistent gait-phase
    structure (stance -> push-off -> swing -> loading) shows up clearly, which
    is what we use to set the FSM thresholds."""
    t = df["t"]
    st_t, _ = detect_foot_strikes(df["a_v"], t)
    if len(st_t) < 3:
        print(f"  (only {len(st_t)} foot strikes - not enough for an overlay)")
        return

    # Build per-step segments between consecutive strikes.
    segs_theta, segs_omega, segs_av = [], [], []
    for i in range(len(st_t) - 1):
        t0, t1 = st_t[i], st_t[i + 1]
        if not (0.25 < (t1 - t0) < 2.0):      # reject implausible stride lengths
            continue
        m = (t >= t0) & (t <= t1)
        x = (t[m] - t0) / (t1 - t0)           # normalized 0..1 within the step
        segs_theta.append((x, df["theta"][m]))
        segs_omega.append((x, df["omega"][m]))
        segs_av.append((x, df["a_v"][m]))

    if not segs_theta:
        print("  (no valid stride segments)")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    panels = [("theta (pitch, deg)", segs_theta, "tab:blue"),
              ("omega (pitch rate, deg/s)", segs_omega, "tab:red"),
              ("a_v (vertical accel, m/s^2)", segs_av, "tab:green")]
    for ax, (lab, segs, col) in zip(axes, panels):
        xs = np.linspace(0.0, 1.0, 400)
        means = []
        for x, y in segs:                       # thin per-step lines
            ax.plot(x, y, color=col, alpha=0.12, linewidth=0.6)
            means.append(np.interp(xs, x, y))   # resample each step onto a grid
        mean_curve = np.nanmean(means, axis=0)  # thick mean stride
        ax.plot(xs, mean_curve, color=col, linewidth=2.4, label="mean stride")
        ax.set_ylabel(lab, fontsize=9); ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    axes[-1].set_xlabel("Normalized stride (0 = foot strike -> 1 = next foot strike)")
    fig.suptitle(f"{title} - {len(segs_theta)} strides aligned by foot strike", fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.985))
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=130)
        print(f"Saved -> {path}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Gait-phase FSM analysis (offline).")
    ap.add_argument("files", nargs="+", help="CSV file(s) from imu_collect_plot.py")
    ap.add_argument("--save", metavar="PNG", default=None,
                    help="Save the diagnose figure to this PNG instead of showing it.")
    ap.add_argument("--diagnose", action="store_true",
                    help="Produce per-file diagnostic figures (one each).")
    ap.add_argument("--overlay", action="store_true",
                    help="Produce stride-aligned overlay figures (for FSM phases).")
    ap.add_argument("--fsm", action="store_true",
                    help="Run the real-time gait-phase FSM and plot its state track.")
    args = ap.parse_args()

    paths = [Path(p) for p in args.files]
    if not all(p.is_file() for p in paths):
        missing = [str(p) for p in paths if not p.is_file()]
        sys.exit(f"File(s) not found: {missing}")

    do_diagnose = (args.diagnose or (len(paths) == 1 and not args.save
                                     and not args.overlay and not args.fsm))
    for p in paths:
        arr = load_csv(p)
        df = to_frame(arr)
        df = derive_signals(df)
        s_ax, a_ax = AXES[int(df["shank_idx"][0])], AXES[int(df["ap_idx"][0])]
        st_t, _ = detect_foot_strikes(df["a_v"], df["t"])
        n = len(df["t"]); dur = df["t"][-1]
        print(f"{p.name}: {n} samples | {dur:.1f}s | shank={s_ax} AP={a_ax} "
              f"| foot strikes={len(st_t)}")

        if do_diagnose:
            plot_diagnose(df, Path("analysis") / (p.stem + "_diagnose.png"), p.name)
        if args.overlay or (args.save and len(paths) == 1):
            ov = (Path(args.save).with_name(Path(args.save).stem + "_overlay.png")
                  if (args.save and len(paths) == 1)
                  else Path("analysis") / (p.stem + "_overlay.png"))
            plot_stride_overlay(df, ov, p.name)
        if args.fsm:
            states, info = run_fsm(df)
            # Per-state duration summary (fraction of all samples in each state).
            n = len(states)
            print(f"  FSM: stride={info['stride_s']:.2f}s av_thr={info['av_thr']:+.1f} "
                  f"om_peak={info['om_peak']:.0f} om_up={info['om_up']:.0f}")
            for s in STATE_ORDER:
                m = states == s
                if m.any():
                    print(f"    {s:<9} {m.sum():5d} samples  ({100.0*m.sum()/n:4.1f}% of time)")
            fsm_path = (Path(args.save).with_name(Path(args.save).stem + "_fsm.png")
                        if (args.save and len(paths) == 1)
                        else Path("analysis") / (p.stem + "_fsm.png"))
            plot_fsm(df, states, fsm_path, p.name)

    if args.save and len(paths) == 1:
        arr = load_csv(paths[0]); df = to_frame(arr); df = derive_signals(df)
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        plot_diagnose(df, Path(args.save), paths[0].name)


if __name__ == "__main__":
    main()
