#!/usr/bin/env python3
"""Back-calculate the ideal FSM constants from mode-'a' advanced capture logs.

The firmware (IMU_BNO055_RT.ino) mode 'a' streams every 100 Hz sample while
you walk at different speeds:

    ADV,S,t_ms,raw_pitch,filtpitch,omega,pitch_offset,theta,metric,ref_state

This script post-processes that log and searches for the best values of the
four firmware constants:

    OMEGA_WEIGHT        metric = |theta| + w * |omega|
    OFF_RANGE_FRACTION  off_thr = min + OFF * (max - min)
    ON_RANGE_FRACTION   on_thr  = min + ON  * (max - min)
    MIN_STATE_TIME_MS   minimum dwell time in a state (anti-chatter)

Method (反計算):
  1. Split the log into walking segments (rolling std gate) and drop the
     still parts (the 3 s still calibration and pauses).
  2. For each candidate OMEGA_WEIGHT, build the metric and measure how
     cleanly it separates stance vs. swing (Otsu between-class variance
     criterion). The weight with the best bimodal separation wins.
  3. The offline (median-filtered, non-causal) threshold at the Otsu level
     is treated as the pseudo ground-truth swing label.
  4. Grid-search (OFF_RANGE_FRACTION, ON_RANGE_FRACTION, MIN_STATE_TIME_MS):
     simulate the exact causal firmware FSM on the metric, score it against
     the pseudo labels (IoU), penalise chatter vs. the expected stride
     count, and add a per-8-second-window score that mimics the firmware's
     'w' walk-calibration at a single speed (robustness across speeds).
  5. Report the recommended constants + plots + JSON.

Usage:
    # 1) record (script drives the ESP32: sends 'a' to start/stop)
    python imu_adv_tune.py --port COM5
    #    ... stand still 3 s, then walk slow / normal / fast ...
    #    Enter  = mark a speed change (segment label in the log)
    #    q+Enter = stop recording, save log, analyse

    # 2) analyse an existing log (Serial Monitor copy-paste also works)
    python imu_adv_tune.py data\\adv_20261022_131142.log
    python imu_adv_tune.py data\\adv_*.log --no-show

Dependencies:
    pip install numpy matplotlib pyserial   (pyserial only for --port)
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'numpy'. Install it with:  pip install numpy")

import matplotlib

try:
    import tkinter  # noqa: F401
    matplotlib.use("TkAgg")
except ImportError:
    pass
import matplotlib.pyplot as plt

# --- Tuning grid (search space for the four constants) -----------------------
W_GRID = np.round(np.arange(0.00, 0.501, 0.025), 4)          # OMEGA_WEIGHT
OFF_GRID = np.round(np.arange(0.25, 0.86, 0.05), 4)          # OFF_RANGE_FRACTION
ON_STEP = 0.05                                               # ON grid step
MT_GRID_MS = [0, 20, 40, 60, 80, 100, 120, 160, 200]         # MIN_STATE_TIME_MS

WALK_CAL_MS = 8000      # firmware 'w' calibration window (per-speed robustness)
GAIT_BAND = (0.3, 2.5)  # human stride frequency band, Hz
RANGE_FLOOR = 1.0       # firmware clamps the metric range to >= this


# --- Log parsing --------------------------------------------------------------
def parse_log(path: Path) -> dict:
    """Parse ADV lines from a serial log. Returns column arrays + metadata."""
    t, raw, filtp, omega, poff, theta, metric, ref = [], [], [], [], [], [], [], []
    params: dict[str, float] = {}
    marks: list[float] = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("ADV,S,"):
                parts = line.split(",")
                if len(parts) != 10:
                    continue
                try:
                    vals = [float(v) for v in parts[2:]]
                except ValueError:
                    continue
                t.append(vals[0])
                raw.append(vals[1])
                filtp.append(vals[2])
                omega.append(vals[3])
                poff.append(vals[4])
                theta.append(vals[5])
                metric.append(vals[6])
                ref.append(vals[7])
            elif line.startswith("ADV,PARAM,"):
                parts = line.split(",")
                if len(parts) == 4:
                    try:
                        params[parts[2]] = float(parts[3])
                    except ValueError:
                        pass
            elif line.startswith("ADV,MARK,"):
                try:
                    marks.append(float(line.split(",")[2]))
                except (IndexError, ValueError):
                    pass
    if len(t) < 300:
        sys.exit(f"{path}: only {len(t)} ADV,S samples found - need >= 300 (3 s of walking).")
    return {
        "path": path,
        "t_ms": np.asarray(t),
        "raw_pitch": np.asarray(raw),
        "filtpitch": np.asarray(filtp),
        "omega": np.asarray(omega),
        "pitch_offset": np.asarray(poff),
        "theta": np.asarray(theta),
        "metric_fw": np.asarray(metric),
        "ref_state": np.asarray(ref),          # -1 = firmware had no thresholds
        "params": params,
        "marks": marks,
    }


def load_many(paths: list[Path]) -> dict:
    """Parse and concatenate several logs (time axis re-based, gaps kept)."""
    if len(paths) == 1:
        d = parse_log(paths[0])
    else:
        d = parse_log(paths[0])
        for p in paths[1:]:
            b = parse_log(p)
            shift = d["t_ms"][-1] + 1000.0
            for key in ("t_ms", "raw_pitch", "filtpitch", "omega",
                        "pitch_offset", "theta", "metric_fw", "ref_state"):
                d[key] = np.concatenate([d[key], b[key]])
            d["t_ms"][len(d["t_ms"]) - len(b["t_ms"]):] += shift
            d["params"].update(b["params"])
    d["t"] = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    d["fs"] = 1.0 / max(1e-6, float(np.median(np.diff(d["t_ms"]))) / 1000.0)
    return d


# --- Signal helpers -----------------------------------------------------------
def rolling_std(x: np.ndarray, win: int) -> np.ndarray:
    """Centred rolling std (numpy only, edge-safe)."""
    win = max(3, win)
    pad = win // 2
    xp = np.concatenate([np.full(pad, x[0]), x, np.full(pad, x[-1])])
    c1 = np.concatenate([[0.0], np.cumsum(xp)])
    c2 = np.concatenate([[0.0], np.cumsum(xp * xp)])
    idx = np.arange(pad, pad + len(x))
    lo = idx - pad
    cnt = (idx + pad + 1) - lo
    mean = (c1[idx + pad + 1] - c1[lo]) / cnt
    var = np.maximum(0.0, (c2[idx + pad + 1] - c2[lo]) / cnt - mean * mean)
    return np.sqrt(var)


def median_filter(x: np.ndarray, k: int) -> np.ndarray:
    """Centred median filter via stride tricks (offline smoothing)."""
    k = max(3, k | 1)
    pad = k // 2
    xp = np.concatenate([np.full(pad, x[0]), x, np.full(pad, x[-1])])
    sw = np.lib.stride_tricks.sliding_window_view(xp, k)
    return np.median(sw, axis=1)[: len(x)]


def otsu(x: np.ndarray, bins: int = 256) -> tuple[float, float]:
    """Otsu threshold + between-class variance criterion (0..1) on samples x."""
    if len(x) < 16 or float(np.ptp(x)) <= 1e-9:
        return 0.0, 0.0
    hist, edges = np.histogram(x, bins=bins)
    p = hist.astype(np.float64)
    p /= p.sum()
    centers = 0.5 * (edges[1:] + edges[:-1])
    w0 = np.cumsum(p)
    w1 = 1.0 - w0
    m0 = np.cumsum(p * centers) / np.maximum(w0, 1e-12)
    m1 = (np.cumsum(p * centers)[-1] - m0 * w0) / np.maximum(w1, 1e-12)
    var = w0 * w1 * (m0 - m1) ** 2
    k = int(np.argmax(var))
    total = float(np.var(x)) + 1e-12
    return float(centers[k]), float(var[k] / total)


def dominant_freq(x: np.ndarray, fs: float) -> float:
    """Dominant frequency of x inside the gait band (Hz)."""
    x = x - x.mean()
    if len(x) < int(4 * fs):
        return 1.0 / max(1e-6, len(x) / fs)
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    freqs = np.fft.rfftfreq(len(x), 1.0 / fs)
    band = (freqs >= GAIT_BAND[0]) & (freqs <= GAIT_BAND[1])
    if not band.any():
        return 1.0
    return float(freqs[band][int(np.argmax(spec[band]))])


def segments_from_mask(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous True runs of mask as (start, end) index pairs."""
    segs = []
    i = 0
    n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            segs.append((i, j + 1))
            i = j + 1
        else:
            i += 1
    return segs


# --- Firmware FSM replica ------------------------------------------------------
def simulate_fsm(metric: np.ndarray, off_thr: float, on_thr: float,
                 min_dt_samples: int) -> np.ndarray:
    """Exact replica of the firmware FSM (causal, hysteresis + min dwell).

    Returns a 0/1 array (0 = FOOT_ON_GROUND, 1 = FOOT_OFF_GROUND), starting
    on-ground like the firmware does. Event-driven so the grid search stays fast.
    """
    n = len(metric)
    out = np.zeros(n, dtype=np.int8)
    state = 0
    seg_start = 0
    last = -(10 ** 9)
    i = 0
    while i < n:
        cond = metric[i:] >= off_thr if state == 0 else metric[i:] <= on_thr
        nz = np.flatnonzero(cond)
        if nz.size == 0:
            break
        j = i + int(nz[0])
        inv = np.flatnonzero(~cond[nz[0]:])
        run_end = j + int(inv[0]) if inv.size else n
        if j - last < min_dt_samples:
            # dwell time not expired yet: the firmware keeps checking and fires
            # as soon as the condition is still true once the timer runs out.
            k = last + min_dt_samples
            if k >= n:
                break
            cond2 = metric[k:] >= off_thr if state == 0 else metric[k:] <= on_thr
            nz2 = np.flatnonzero(cond2)
            if nz2.size == 0:
                break
            j = k + int(nz2[0])
            inv2 = np.flatnonzero(~cond2[nz2[0]:])
            run_end = j + int(inv2[0]) if inv2.size else n
        out[seg_start:j] = state
        state ^= 1
        seg_start = j
        last = j
        i = run_end
    out[seg_start:] = state
    return out


def iou(a: np.ndarray, b: np.ndarray) -> float:
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter) / float(union) if union else 1.0


def count_switches(states: np.ndarray) -> int:
    return int(np.count_nonzero(np.diff(states.astype(np.int16))))


# --- Tuning --------------------------------------------------------------------
def tune(d: dict) -> dict:
    fs = d["fs"]
    theta, omega, t = d["theta"], d["omega"], d["t"]
    n = len(t)

    # 1) walking gate on the pitch-rate energy (same style as gait_fsm.py)
    sig = rolling_std(omega, int(round(1.5 * fs)))
    walk_thr = max(3.0, 0.20 * float(np.percentile(sig, 95)))
    walk = sig > walk_thr
    # keep pauses inside a walk session: bridge gaps < 2 s, drop runs < 3 s
    segs = segments_from_mask(walk)
    merged = []
    for s, e in segs:
        if merged and (s - merged[-1][1]) < 2 * fs:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    walk = np.zeros(n, dtype=bool)
    for s, e in merged:
        if e - s >= 3 * fs:
            walk[s:e] = True
    if walk.mean() < 0.15:
        print("WARNING: <15% of the log looks like walking - using all samples.")
        walk[:] = True
    walk_idx = np.flatnonzero(walk)
    dur = float(t[walk_idx[-1]] - t[walk_idx[0]])

    # 2) pick OMEGA_WEIGHT by stance/swing separability of the metric
    w_scores = []
    for w in W_GRID:
        m = np.abs(theta) + w * np.abs(omega)
        _, crit = otsu(m[walk])
        w_scores.append(crit)
    w_scores = np.asarray(w_scores)
    w_best = float(W_GRID[int(np.argmax(w_scores))])
    metric = np.abs(theta) + w_best * np.abs(omega)

    # 3) pseudo ground truth: offline median-filtered metric vs Otsu level
    otsu_thr, _ = otsu(metric[walk])
    m_smooth = median_filter(metric, int(round(0.15 * fs)))
    pseudo = (m_smooth >= otsu_thr) & walk

    stride_hz = dominant_freq(metric[walk], fs)
    exp_switches = max(2.0, 2.0 * stride_hz * dur)

    # 4) grid search over the threshold fractions + min dwell time
    m_walk = metric[walk]
    g_min = float(m_walk.min())
    g_max = float(m_walk.max())
    g_range = max(RANGE_FLOOR, g_max - g_min)

    # per-8s calibration windows (mimics the firmware 'w' calibration)
    win_n = int(round(WALK_CAL_MS / 1000.0 * fs))
    windows: list[tuple[int, int]] = []
    for s, e in segments_from_mask(walk):
        pos = s
        while pos + win_n <= e:
            windows.append((pos, pos + win_n))
            pos += win_n

    results = []
    for off_f in OFF_GRID:
        off_thr = g_min + off_f * g_range
        on_vals = np.round(np.arange(0.0, off_f - ON_STEP / 2 + 1e-9, ON_STEP), 4)
        for on_f in on_vals:
            on_thr = g_min + on_f * g_range
            if on_thr >= off_thr:
                continue
            states = simulate_fsm(metric, off_thr, on_thr, 0)
            sw = count_switches(states)
            chatter = max(0.0, (sw - exp_switches) / exp_switches)
            for mt_ms in MT_GRID_MS:
                min_dt = int(round(mt_ms / 1000.0 * fs))
                st = states if min_dt == 0 else \
                    simulate_fsm(metric, off_thr, on_thr, min_dt)
                giou = iou(st.astype(bool), pseudo)
                # per-window robustness across speeds
                wi = []
                for s, e in windows:
                    wseg = metric[s:e]
                    rng = max(RANGE_FLOOR, float(wseg.max() - wseg.min()))
                    o = float(wseg.min()) + off_f * rng
                    on = float(wseg.min()) + on_f * rng
                    if on >= o:
                        continue
                    sw_st = simulate_fsm(wseg, o, on, min_dt)
                    wi.append(iou(sw_st.astype(bool), pseudo[s:e]))
                wmiou = float(np.mean(wi)) if wi else 0.0
                score = 0.6 * giou + 0.4 * wmiou - 0.3 * chatter
                results.append((score, float(off_f), float(on_f),
                                int(mt_ms), giou, wmiou, chatter))

    results.sort(key=lambda r: r[0], reverse=True)
    best = results[0]

    # MIN_STATE_TIME_MS: smallest dwell time reaching >= 98% of the best score
    best_score = best[0]
    near = [r for r in results if r[0] >= 0.98 * best_score
            and abs(r[1] - best[1]) < 1e-9 and abs(r[2] - best[2]) < 1e-9]
    mt_best = min(r[3] for r in near) if near else best[3]
    # report the metrics of the exact (off, on, mt_best) row we will recommend
    chosen = next((r for r in results if r[3] == mt_best
                   and abs(r[1] - best[1]) < 1e-9 and abs(r[2] - best[2]) < 1e-9), best)

    # final state sequence with the recommended constants
    off_thr = g_min + best[1] * g_range
    on_thr = g_min + best[2] * g_range
    states = simulate_fsm(metric, off_thr, on_thr,
                          int(round(mt_best / 1000.0 * fs)))

    # score of the constants currently loaded in the firmware (if logged)
    cur = None
    p = d["params"]
    if {"OMEGA_WEIGHT", "OFF_RANGE_FRACTION", "ON_RANGE_FRACTION",
            "MIN_STATE_TIME_MS"} <= set(p):
        m_cur = np.abs(theta) + p["OMEGA_WEIGHT"] * np.abs(omega)
        c_min = float(m_cur[walk].min())
        c_rng = max(RANGE_FLOOR, float(m_cur[walk].max()) - c_min)
        o_c = c_min + p["OFF_RANGE_FRACTION"] * c_rng
        n_c = c_min + p["ON_RANGE_FRACTION"] * c_rng
        if n_c < o_c:
            st_c = simulate_fsm(m_cur, o_c, n_c,
                                int(round(p["MIN_STATE_TIME_MS"] / 1000.0 * fs)))
            cur = dict(iou=iou(st_c.astype(bool), pseudo),
                       switches=count_switches(st_c))

    return {
        "fs": fs,
        "duration_s": dur,
        "stride_hz": stride_hz,
        "walk_frac": float(walk.mean()),
        "w_grid": W_GRID,
        "w_scores": w_scores,
        "w_best": w_best,
        "metric_best": metric,
        "otsu_thr": otsu_thr,
        "off_thr": off_thr,
        "on_thr": on_thr,
        "metric_min": g_min,
        "metric_max": g_max,
        "pseudo": pseudo,
        "walk": walk,
        "states": states,
        "best": {"score": chosen[0], "off_f": chosen[1], "on_f": chosen[2],
                 "mt_ms": mt_best, "giou": chosen[4], "window_iou": chosen[5],
                 "chatter": chosen[6]},
        "top": results[:10],
        "current_fw": cur,
        "exp_switches": exp_switches,
        "switches": count_switches(states),
    }


# --- Plotting ------------------------------------------------------------------
def plot_results(d: dict, r: dict, out_png: Path, show: bool) -> None:
    t = d["t"]
    metric = r["metric_best"]
    fig, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=False)

    ax = axes[0]
    ax.plot(t, d["theta"], color="tab:blue", lw=0.8, label="theta (deg)")
    ax.set_ylabel("theta (deg)", color="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(t, d["omega"], color="tab:red", lw=0.7, alpha=0.7, label="omega (deg/s)")
    ax2.set_ylabel("omega (deg/s)", color="tab:red")
    ax.set_title("Shank pitch theta & angular velocity omega (raw capture)",
                 fontsize=9, loc="left")
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(t, metric, color="tab:purple", lw=0.7)
    ax.axhline(r["otsu_thr"], color="k", ls=":", lw=0.8,
               label=f"Otsu {r['otsu_thr']:.2f}")
    ax.axhline(r["off_thr"], color="tab:red", ls="--", lw=0.9,
               label=f"off_thr {r['off_thr']:.2f}")
    ax.axhline(r["on_thr"], color="tab:green", ls="--", lw=0.9,
               label=f"on_thr {r['on_thr']:.2f}")
    ax.set_title(f"metric = |theta| + {r['w_best']:.3f}*|omega|  "
                 f"(min={r['metric_min']:.2f}, max={r['metric_max']:.2f})",
                 fontsize=9, loc="left")
    ax.legend(fontsize=7, ncol=3, loc="upper right")
    ax.grid(alpha=0.3)

    ax = axes[2]
    bands = [("pseudo (offline)", r["pseudo"], "#9e9e9e"),
             ("tuned FSM", r["states"].astype(bool), "#e53935")]
    if np.any(d["ref_state"] >= 0):
        bands.append(("firmware FSM (old params)", d["ref_state"] > 0, "#1e88e5"))
    for k, (name, mask, color) in enumerate(bands):
        for s, e in segments_from_mask(mask):
            ax.broken_barh([(t[s], t[e] - t[s])], (k * 1.2, 1.0),
                           facecolors=color, edgecolor="none")
    ax.set_ylim(-0.3, len(bands) * 1.2)
    ax.set_yticks([k * 1.2 + 0.5 for k in range(len(bands))])
    ax.set_yticklabels([b[0] for b in bands], fontsize=7)
    ax.set_title("Swing (FOOT_OFF_GROUND) segments: offline reference vs FSM",
                 fontsize=9, loc="left")

    ax = axes[3]
    ax.plot(r["w_grid"], r["w_scores"], "o-", color="tab:blue", ms=3)
    ax.axvline(r["w_best"], color="red", ls="--", lw=0.8)
    ax.set_xlabel("OMEGA_WEIGHT")
    ax.set_ylabel("separation score")
    ax.set_title(f"OMEGA_WEIGHT scan (best = {r['w_best']:.3f})", fontsize=9, loc="left")
    ax.grid(alpha=0.3)

    ax = axes[4]
    mts = sorted({row[3] for row in r["top"]})
    for off_f in sorted({row[1] for row in r["top"]}):
        for on_f in sorted({row[2] for row in r["top"]}):
            pts = [(row[3], row[0]) for row in r["top"]
                   if abs(row[1] - off_f) < 1e-9 and abs(row[2] - on_f) < 1e-9]
            if len(pts) > 1:
                ax.plot(*zip(*pts), "o-", ms=3, lw=0.8,
                        label=f"off={off_f:.2f}, on={on_f:.2f}")
    ax.axvline(r["best"]["mt_ms"], color="red", ls="--", lw=0.8)
    ax.set_xlabel("MIN_STATE_TIME_MS")
    ax.set_ylabel("score")
    ax.set_title("Top parameter combos vs MIN_STATE_TIME_MS", fontsize=9, loc="left")
    ax.legend(fontsize=6, ncol=2)
    ax.grid(alpha=0.3)

    fig.suptitle(f"Advanced capture tuning - {d['path'].name} "
                 f"({r['duration_s']:.0f} s walking, stride {r['stride_hz']:.2f} Hz)",
                 fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.985))
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=130)
    print(f"Saved plot -> {out_png}")
    if show:
        plt.show()


# --- Serial recording ------------------------------------------------------------
def record_serial(port: str, baud: int, out_dir: Path) -> Path:
    try:
        import serial
    except ImportError:
        sys.exit("Missing dependency 'pyserial' for --port. "
                 "Install it with:  pip install pyserial")

    ser = serial.Serial(port, baud, timeout=0.2)
    time.sleep(2.0)          # let the ESP32 finish booting
    ser.reset_input_buffer()

    out_path = out_dir / f"adv_{datetime.now():%Y%m%d_%H%M%S}.log"
    stop = threading.Event()
    log_f = open(out_path, "w", encoding="utf-8")

    def reader() -> None:
        buf = b""
        while not stop.is_set():
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").rstrip()
                log_f.write(text + "\n")
                log_f.flush()
                if text.startswith(("ADV,STILL", "ADV,START", "ADV,STOP",
                                    "ADV,STAT", "CAL,", "REJECTED", "READY")):
                    print(f"  < {text}")

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    print("\n=== Mode 'a' recording ===")
    print("The ESP32 must already be IMU-calibrated (press 'i' in the Serial")
    print("Monitor beforehand if needed - this script only sends 'a').\n")
    input("Press Enter to send 'a' -> 3 s still calibration, then start walking...")
    ser.write(b"a")
    print("Sent 'a'. Stand still, then walk.")
    print("During recording:  Enter = mark a speed change | q + Enter = stop\n")
    mark_no = 1
    while True:
        try:
            ans = input().strip().lower()
        except EOFError:
            ans = "q"
        if ans == "q":
            break
        mark_no += 1
        log_f.write(f"ADV,MARK,{time.time() * 1000.0:.0f},speed_{mark_no}\n")
        print(f"  > marked speed segment {mark_no}")
    ser.write(b"a")
    print("Sent 'a' (stop). Waiting for the firmware to flush...")
    time.sleep(1.0)
    stop.set()
    th.join(timeout=1.0)
    log_f.close()
    ser.close()
    print(f"Log saved -> {out_path}")
    return out_path


# --- Main ------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="*", type=Path,
                    help="ADV log file(s); default: newest data/adv_*.log")
    ap.add_argument("--port", help="record from this serial port first (e.g. COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-show", action="store_true", help="save plots, do not open windows")
    ap.add_argument("--out-dir", type=Path, default=Path("analysis"))
    args = ap.parse_args()

    if args.port:
        log_paths = [record_serial(args.port, args.baud, Path("data"))]
    elif args.logs:
        log_paths = args.logs
    else:
        data = Path(__file__).resolve().parent / "data"
        cands = sorted(data.glob("adv_*.log"), key=lambda p: p.stat().st_mtime)
        if not cands:
            sys.exit("No logs given and no data/adv_*.log found. "
                     "Use --port COM5 to record or pass a log file.")
        log_paths = [cands[-1]]

    d = load_many(log_paths)
    print(f"Loaded {len(d['t_ms'])} samples "
          f"({d['t_ms'][-1] - d['t_ms'][0]:.0f} ms, ~{d['fs']:.0f} Hz) "
          f"from {', '.join(p.name for p in log_paths)}")
    if d["params"]:
        cur = ", ".join(f"{k}={v:g}" for k, v in d["params"].items())
        print(f"Firmware constants at capture: {cur}")

    r = tune(d)
    b = r["best"]
    stem = re.sub(r"[^\w]+", "_", "_".join(p.stem for p in log_paths))[:60]
    png = args.out_dir / f"adv_tune_{stem}.png"
    plot_results(d, r, png, show=not args.no_show)

    print("\n================ RECOMMENDED FIRMWARE CONSTANTS ================")
    print(f"  OMEGA_WEIGHT        = {r['w_best']:.3f}")
    print(f"  OFF_RANGE_FRACTION  = {b['off_f']:.2f}")
    print(f"  ON_RANGE_FRACTION   = {b['on_f']:.2f}")
    print(f"  MIN_STATE_TIME_MS   = {b['mt_ms']}")
    print("  --------------------------------------------------------------")
    print(f"  metric range        : {r['metric_min']:.2f} .. {r['metric_max']:.2f}"
          f"  -> off_thr={r['off_thr']:.2f}, on_thr={r['on_thr']:.2f}")
    print(f"  swing IoU (global)  : {b['giou']:.3f}")
    print(f"  swing IoU (per 8 s) : {b['window_iou']:.3f}   (speed robustness)")
    print(f"  FSM switches        : {r['switches']} "
          f"(expected ~{r['exp_switches']:.0f} from {r['stride_hz']:.2f} Hz stride)")
    if r["current_fw"]:
        print(f"  current firmware    : IoU {r['current_fw']['iou']:.3f}, "
              f"{r['current_fw']['switches']} switches")
    print("================================================================\n")

    out_json = args.out_dir / f"adv_tune_{stem}.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    with open(out_json, "w") as f:
        json.dump({
            "logs": [str(p) for p in log_paths],
            "generated": datetime.now().isoformat(timespec="seconds"),
            "recommended": {
                "OMEGA_WEIGHT": r["w_best"],
                "OFF_RANGE_FRACTION": b["off_f"],
                "ON_RANGE_FRACTION": b["on_f"],
                "MIN_STATE_TIME_MS": b["mt_ms"],
            },
            "scores": {
                "score": b["score"], "iou_global": b["giou"],
                "iou_per_window": b["window_iou"],
                "switches": r["switches"],
                "expected_switches": r["exp_switches"],
            },
            "thresholds": {"metric_min": r["metric_min"], "metric_max": r["metric_max"],
                           "off_thr": r["off_thr"], "on_thr": r["on_thr"],
                           "otsu": r["otsu_thr"]},
            "capture": {"samples": int(len(d["t_ms"])), "fs": d["fs"],
                        "walk_duration_s": r["duration_s"],
                        "stride_hz": r["stride_hz"]},
            "firmware_at_capture": d["params"],
            "current_firmware_score": r["current_fw"],
            "top10": [{"score": round(x[0], 4), "off_f": x[1], "on_f": x[2],
                       "mt_ms": x[3], "iou": round(x[4], 4)} for x in r["top"]],
        }, f, indent=2)
    print(f"Saved results -> {out_json}")


if __name__ == "__main__":
    main()
