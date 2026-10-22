#!/usr/bin/env python3
"""
plot_ext_nav_delay.py — how late the lighthouse position reaches the flight controller,
measured from an ArduPilot DataFlash log and shown as a picture.

Test: move the drone back and forth by hand (or fly it) for 20-60 s, with clear reversals,
then download the .bin. The script compares two velocities of the same motion:

  * from the accelerometer (IMU, rotated to the earth frame and integrated): no delay;
  * from the lighthouse position the flight controller received (VISP): late.

Shifting one curve against the other until they match best gives the delay. Two numbers:

  * total delay T: motion -> position arrives at the flight controller (VISP.TimeUS);
  * delay of ArduPilot's own sample time Lc: motion -> VISP.CTimeMS, the capture stamp the
    firmware puts in ODOMETRY, translated by ArduPilot to its clock. ArduPilot sees T - Lc from
    the stamps by itself; VISO_DELAY_MS must cover Lc. Residual Lc - VISO_DELAY_MS ~ 0 = right.

The figure and the terminal also list the real readings of the log, one row per position received
(VISP): arrival, capture time per ArduPilot, age seen from the stamps, the moment the EKF uses
(capture - VISO_DELAY_MS), age assumed by the EKF, real age (age seen + Lc) and their difference;
every reading goes to <figure>_readings.csv.

The figure is written as a PNG next to the log and opened with the image viewer (--window for an
interactive matplotlib window instead, --no-show to only write it).

Usage (Windows: double-click plot_delay.bat, or drop a .bin on it):
    python plot_ext_nav_delay.py                 # newest .bin in Documents\\Mission Planner\\logs
    python plot_ext_nav_delay.py flight.bin --axis N --zoom 10 --out delay.png --lang en
Needs numpy, matplotlib and pymavlink.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np

DT = 0.005  # resampling step [s]


# --------------------------------------------------------------------------- #
# Log reading
# --------------------------------------------------------------------------- #

def newest_log() -> str | None:
    root = os.path.join(os.path.expanduser("~"), "Documents", "Mission Planner", "logs")
    logs = glob.glob(os.path.join(root, "**", "*.bin"), recursive=True)
    return max(logs, key=os.path.getmtime) if logs else None


def read_log(path: str) -> dict:
    from pymavlink import mavutil
    log = mavutil.mavlink_connection(path)
    visp, imu, att, params = [], [], [], {}
    while True:
        m = log.recv_match(type=["VISP", "IMU", "ATT", "PARM"])
        if m is None:
            break
        kind, t = m.get_type(), m.TimeUS / 1e6
        if kind == "VISP":
            visp.append((t, m.PX, m.PY, m.PZ, m.RTimeUS / 1e6, m.CTimeMS / 1e3))
        elif kind == "IMU" and getattr(m, "I", 0) == 0:
            imu.append((t, m.AccX, m.AccY, m.AccZ))
        elif kind == "ATT":
            att.append((t, np.radians(m.Roll), np.radians(m.Pitch), np.radians(m.Yaw)))
        elif kind == "PARM":
            params[m.Name] = m.Value
    if len(visp) < 50 or len(imu) < 500 or len(att) < 100:
        sys.exit(f"{os.path.basename(path)}: not enough VISP / IMU / ATT data "
                 f"({len(visp)} / {len(imu)} / {len(att)}). Is LOG_DISARMED = 1 and was the lighthouse position arriving?")
    return {"visp": np.array(visp), "imu": np.array(imu), "att": np.array(att), "params": params}


# --------------------------------------------------------------------------- #
# Signals
# --------------------------------------------------------------------------- #

def smooth(x, n):
    return np.convolve(x, np.ones(n) / n, "same")


def highpass(x, seconds=1.5):
    n = int(seconds / DT) | 1
    return x - np.convolve(x, np.ones(n) / n, "same")


def signals(d: dict, t0: float, t1: float, visp_time: np.ndarray) -> dict:
    """Resampled earth-frame IMU acceleration / velocity and lighthouse position / velocity."""
    t = np.arange(t0, t1, DT)
    att, imu, visp = d["att"], d["imu"], d["visp"]
    r, p, y = (np.interp(t, att[:, 0], np.unwrap(att[:, k])) for k in (1, 2, 3))
    ax, ay, az = (np.interp(t, imu[:, 0], imu[:, k]) for k in (1, 2, 3))
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    acc = {"N": cy * cp * ax + (cy * sp * sr - sy * cr) * ay + (cy * sp * cr + sy * sr) * az,
           "E": sy * cp * ax + (sy * sp * sr + cy * cr) * ay + (sy * sp * cr - cy * sr) * az}
    out = {"t": t}
    for k, col in (("N", 1), ("E", 2)):
        a = smooth(acc[k], 21)
        out["acc_" + k] = a - np.mean(a)
        out["vimu_" + k] = highpass(np.cumsum(a) * DT)                      # IMU velocity, no delay
        pos = np.interp(t, visp_time, visp[:, col])
        out["pos_" + k] = pos
        out["vlh_" + k] = highpass(np.gradient(smooth(pos, 21), t))          # lighthouse velocity
    return out


def correlate(a, b, lags_ms):
    """Correlation of a(t) with b(t + lag) for each lag."""
    out = []
    for lag in lags_ms:
        sh = int(round(lag / 1000 / DT))
        if sh >= 0:
            out.append(np.corrcoef(a[:len(a) - sh], b[sh:])[0, 1])
        else:
            out.append(np.corrcoef(a[-sh:], b[:len(b) + sh])[0, 1])
    return np.array(out)


def peak(lags, corr):
    i = int(np.argmax(corr))
    if 0 < i < len(corr) - 1:                       # parabola through the top three points
        y0, y1, y2 = corr[i - 1], corr[i], corr[i + 1]
        off = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
        return lags[i] + off * (lags[1] - lags[0]), corr[i]
    return lags[i], corr[i]


def motion_window(d: dict) -> tuple[float, float]:
    """Span where the lighthouse position moves (speed > 0.15 m/s), padded by 1 s."""
    v = d["visp"]
    speed = np.hypot(np.gradient(v[:, 1], v[:, 0]), np.gradient(v[:, 2], v[:, 0]))
    moving = v[speed > 0.15, 0]
    lo = max(d["imu"][0, 0], d["att"][0, 0], v[0, 0]) + 0.5
    hi = min(d["imu"][-1, 0], d["att"][-1, 0], v[-1, 0]) - 0.5
    if len(moving) < 20:
        return lo, hi
    return max(lo, moving[0] - 1.0), min(hi, moving[-1] + 1.0)


# --------------------------------------------------------------------------- #
# Texts (Italian by default, --lang en)
# --------------------------------------------------------------------------- #

TEXT = {
    "it": {
        "title": "Ritardo della posizione lighthouse — {log}  ({dur:.0f} s di movimento, asse {axis})",
        "p1": "1. Dati grezzi: posizione dalle lighthouse e accelerazione dall'IMU (in opposizione in un avanti-indietro)",
        "pos": "posizione lighthouse {axis} (ricevuta)", "acc": "accelerometro {axis} (Nord/Est)",
        "pos_y": "posizione [m]", "acc_y": "accelerazione [m/s²]", "time": "tempo [s]",
        "p2": "2. Stesso movimento, due fonti: la curva delle lighthouse arriva dopo — di {lag:.0f} ms",
        "vimu": "velocità dall'IMU (senza ritardo)", "vlh": "velocità dalle lighthouse (come arriva)",
        "p3": "3. Spostando la curva delle lighthouse {lag:.0f} ms indietro le due coincidono: quello è il ritardo",
        "vlh_back": "velocità lighthouse spostata di {lag:.0f} ms", "vel_y": "velocità [m/s]",
        "p4": "4. Quanto coincidono le curve per ogni spostamento (tutta la registrazione, N ed E): il picco è il ritardo",
        "c_arr": "come arriva al Pixhawk (ritardo totale T)", "c_ct": "all'istante che gli dà ArduPilot (Lc)",
        "shift": "spostamento della velocità lighthouse [ms]", "corr": "correlazione",
        "corr_help": "correlazione = quanto si somigliano le due curve dopo lo spostamento: 1 = identiche, 0 = nessuna somiglianza. "
                     "Si provano tutti gli spostamenti; quello con la somiglianza massima (il picco) è il ritardo.",
        "c_peak": "picco", 
        "tab_title": "Letture reali dal log (posizioni ricevute dal Pixhawk, messaggio VISP)",
        "cols": ["#", "arrivo al\nPixhawk [s]", "cattura secondo\nArduPilot [s]", "età vista dai\ntimbri [ms]",
                 "istante usato\ndall'EKF [s]", "età assunta\ndall'EKF [ms]", "età reale\n[ms]", "differenza\n[ms]"],
        "col_help": "arrivo = VISP.TimeUS · cattura = VISP.CTimeMS (il timbro del firmware sull'orologio del Pixhawk) · "
                    "età vista = arrivo − cattura · istante EKF = cattura − VISO_DELAY_MS ({v}) · età assunta = arrivo − istante EKF · "
                    "età reale = età vista + Lc ({lc:.0f} ms, misurato su tutta la registrazione) · differenza = assunta − reale",
        "summary": "Tutte le {n} letture: età vista dai timbri mediana {am:.0f} ms (min {amin:.0f}, max {amax:.0f}) · età assunta dall'EKF "
                   "mediana {sm:.0f} ms · età reale mediana {rm:.0f} ms · differenza {d:+.0f} ms",
        "ok": "Differenza ≈ 0: l'EKF usa l'istante giusto. VISO_DELAY_MS = {v:.0f} va bene.",
        "up": "L'EKF usa un istante troppo recente: aumenta VISO_DELAY_MS a circa {rec:.0f}.",
        "down": "L'EKF guarda troppo indietro: diminuisci VISO_DELAY_MS a circa {rec:.0f}.",
        "nov": "VISO_DELAY_MS non è nel log: impostalo a circa {rec:.0f}.",
        "csv": "tutte le letture: {path}",
    },
    "en": {
        "title": "Lighthouse position delay — {log}  ({dur:.0f} s of motion, axis {axis})",
        "p1": "1. Raw data: position from the lighthouses and acceleration from the IMU (opposite phase in a back-and-forth)",
        "pos": "lighthouse position {axis} (received)", "acc": "accelerometer {axis} (North/East)",
        "pos_y": "position [m]", "acc_y": "acceleration [m/s²]", "time": "time [s]",
        "p2": "2. Same motion, two sources: the lighthouse curve comes later — by {lag:.0f} ms",
        "vimu": "velocity from the IMU (no delay)", "vlh": "velocity from the lighthouses (as received)",
        "p3": "3. Moving the lighthouse curve {lag:.0f} ms earlier makes the two overlap: that is the delay",
        "vlh_back": "lighthouse velocity moved {lag:.0f} ms earlier", "vel_y": "velocity [m/s]",
        "p4": "4. How well the curves match for every shift (whole recording, N and E): the peak is the delay",
        "c_arr": "as it reaches the Pixhawk (total delay T)", "c_ct": "at the time ArduPilot gives it (Lc)",
        "shift": "shift of the lighthouse velocity [ms]", "corr": "correlation",
        "corr_help": "correlation = how alike the two curves are after the shift: 1 = identical, 0 = not alike at all. "
                     "Every shift is tried; the one with the highest similarity (the peak) is the delay.",
        "c_peak": "peak", 
        "tab_title": "Real readings from the log (positions received by the Pixhawk, VISP message)",
        "cols": ["#", "arrival at\nPixhawk [s]", "capture per\nArduPilot [s]", "age seen from\nstamps [ms]",
                 "time used\nby the EKF [s]", "age assumed\nby the EKF [ms]", "real age\n[ms]", "difference\n[ms]"],
        "col_help": "arrival = VISP.TimeUS · capture = VISP.CTimeMS (the firmware stamp on the Pixhawk clock) · "
                    "age seen = arrival − capture · EKF time = capture − VISO_DELAY_MS ({v}) · age assumed = arrival − EKF time · "
                    "real age = age seen + Lc ({lc:.0f} ms, measured on the whole recording) · difference = assumed − real",
        "summary": "All {n} readings: age seen from the stamps median {am:.0f} ms (min {amin:.0f}, max {amax:.0f}) · age assumed by the EKF "
                   "median {sm:.0f} ms · real age median {rm:.0f} ms · difference {d:+.0f} ms",
        "ok": "Difference ≈ 0: the EKF uses the right moment. VISO_DELAY_MS = {v:.0f} is right.",
        "up": "The EKF uses too recent a moment: raise VISO_DELAY_MS to about {rec:.0f}.",
        "down": "The EKF looks too far back: lower VISO_DELAY_MS to about {rec:.0f}.",
        "nov": "VISO_DELAY_MS is not in the log: set it to about {rec:.0f}.",
        "csv": "all readings: {path}",
    },
}


def readings(visp, t0, t1, lc, viso):
    """Per received position: arrival, capture time per ArduPilot, ages and the residual [s / ms]."""
    v = visp[(visp[:, 0] >= t0) & (visp[:, 0] <= t1)]
    arrival, capture = v[:, 0], v[:, 5]
    seen = 1000 * (arrival - capture)                   # age ArduPilot gets from the stamps
    vd = viso if viso is not None else 0.0
    ekf_time = capture - vd / 1000                      # the moment the EKF compares it with
    assumed = 1000 * (arrival - ekf_time)               # = seen + VISO_DELAY_MS
    real = seen + lc                                    # stamp -> arrival, plus motion -> stamp (Lc)
    return {"arrival": arrival, "capture": capture, "seen": seen, "ekf_time": ekf_time,
            "assumed": assumed, "real": real, "diff": assumed - real, "pos": v[:, 1:4]}


def verdict(tx, lc, viso):
    rec = 5 * round(lc / 5)
    if viso is None:
        return tx["nov"].format(rec=rec), False
    if abs(lc - viso) <= 15:
        return tx["ok"].format(v=viso), True
    return (tx["up"] if lc > viso else tx["down"]).format(rec=rec), False


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="DataFlash .bin (default: newest in Documents\\Mission Planner\\logs)")
    ap.add_argument("--axis", choices=["auto", "N", "E"], default="auto", help="axis to draw (default: the one that moves most)")
    ap.add_argument("--start", type=float, help="analysis start [s of log time]")
    ap.add_argument("--end", type=float, help="analysis end [s of log time]")
    ap.add_argument("--zoom", type=float, default=8.0, help="seconds shown in the time plots (default 8)")
    ap.add_argument("--out", help="PNG to write (default: next to the log)")
    ap.add_argument("--no-show", action="store_true", help="only write the PNG, do not open it")
    ap.add_argument("--window", action="store_true", help="show an interactive matplotlib window instead of opening the PNG")
    ap.add_argument("--lang", choices=["it", "en"], default="it", help="language of the figure and the steps (default it)")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(errors="replace")   # never fail on a character the console cannot show
    except (AttributeError, ValueError):
        pass

    path = args.log or newest_log()
    if not path or not os.path.exists(path):
        sys.exit("no .bin given and none found in Documents\\Mission Planner\\logs")
    print(f"log: {path}")
    d = read_log(path)
    t0, t1 = motion_window(d)
    t0, t1 = args.start or t0, args.end or t1
    if t1 - t0 < 8:
        sys.exit(f"only {t1 - t0:.1f} s of motion: record at least 20 s of back-and-forth")

    lags = np.arange(-100, 600 + 1e-9, 5.0)
    arrival = signals(d, t0, t1, d["visp"][:, 0])                       # VISP.TimeUS: as received
    ctime = signals(d, t0, t1, d["visp"][:, 5])                         # VISP.CTimeMS: ArduPilot's sample time
    res = {}
    for name, s in (("arrival", arrival), ("ctime", ctime)):
        cn = correlate(s["vimu_N"], s["vlh_N"], lags)
        ce = correlate(s["vimu_E"], s["vlh_E"], lags)
        res[name] = {"N": peak(lags, cn), "E": peak(lags, ce), "both": peak(lags, (cn + ce) / 2), "curve": (cn + ce) / 2}
    axis = args.axis if args.axis != "auto" else ("N" if np.std(arrival["pos_N"]) >= np.std(arrival["pos_E"]) else "E")
    lag, corr = res["arrival"]["both"]
    lag_ct, corr_ct = res["ctime"]["both"]
    viso = d["params"].get("VISO_DELAY_MS")
    tx = TEXT[args.lang]
    rd = readings(d["visp"], t0, t1, lag_ct, viso)
    verdict_text, good = verdict(tx, lag_ct, viso)
    out = args.out or os.path.splitext(path)[0] + "_delay.png"
    csv_path = os.path.splitext(out)[0] + "_readings.csv"
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("arrival_s,capture_arducopter_s,age_seen_ms,visodelay_ms,ekf_time_s,age_assumed_ms,lc_ms,age_real_ms,"
                "difference_ms,pos_n_m,pos_e_m,pos_d_m\n")
        for k in range(len(rd["arrival"])):
            f.write(f"{rd['arrival'][k]:.4f},{rd['capture'][k]:.4f},{rd['seen'][k]:.1f},{viso if viso is not None else ''},"
                    f"{rd['ekf_time'][k]:.4f},{rd['assumed'][k]:.1f},{lag_ct:.1f},{rd['real'][k]:.1f},{rd['diff'][k]:.1f},"
                    f"{rd['pos'][k, 0]:.4f},{rd['pos'][k, 1]:.4f},{rd['pos'][k, 2]:.4f}\n")
    summary = tx["summary"].format(n=len(rd["seen"]), am=np.median(rd["seen"]), amin=rd["seen"].min(), amax=rd["seen"].max(),
                                   sm=np.median(rd["assumed"]), rm=np.median(rd["real"]),
                                   d=(viso - lag_ct) if viso is not None else float("nan"))

    print(f"analysed {t1 - t0:.0f} s ({t0:.0f}-{t1:.0f} s)")
    print(f"total delay T (motion -> arrival at the FC, VISP.TimeUS): {lag:.0f} ms  (correlation {corr:.2f}; "
          f"N {res['arrival']['N'][0]:.0f} ms, E {res['arrival']['E'][0]:.0f} ms)")
    print(f"ArduPilot's sample time (VISP.CTimeMS) lags the motion by Lc = {lag_ct:.0f} ms (correlation {corr_ct:.2f}): "
          f"VISO_DELAY_MS must cover this")
    print(f"VISO_DELAY_MS in the log: {viso}\n")
    plain = lambda text: text.replace("−", "-").replace("≈", "~").replace("\n", " ")   # Windows consoles (cp1252)
    print(plain(tx["tab_title"]) + " (first 10 in the zoomed stretch):")
    hdr = [plain(c) for c in tx["cols"]]
    print("  " + " | ".join(hdr))
    for k in range(min(10, len(rd["arrival"]))):
        print(f"  {k + 1:4d} | {rd['arrival'][k]:9.3f} | {rd['capture'][k]:9.3f} | {rd['seen'][k]:6.0f} | {rd['ekf_time'][k]:9.3f} | "
              f"{rd['assumed'][k]:6.0f} | {rd['real'][k]:6.0f} | {rd['diff'][k]:+5.0f}")
    print("  " + plain(summary))
    print("  " + plain(verdict_text))
    print(plain(tx["csv"].format(path=csv_path)))

    import matplotlib
    if not args.window:
        matplotlib.use("Agg")      # draw to the PNG at its full size, then open the PNG
    import matplotlib.pyplot as plt

    # zoom on the stretch with the most motion
    t = arrival["t"]
    energy = np.convolve(arrival["vimu_" + axis] ** 2, np.ones(int(args.zoom / DT)), "same")
    centre = t[int(np.argmax(energy))]
    z0, z1 = max(t[0], centre - args.zoom / 2), min(t[-1], centre + args.zoom / 2)
    zs = (t >= z0) & (t <= z1)
    tz = t[zs] - z0
    shift = int(round(lag / 1000 / DT))

    fig = plt.figure(figsize=(13, 17))
    gs = fig.add_gridspec(5, 1, height_ratios=[1, 1, 1, 1.5, 1.6], hspace=0.6)
    fig.suptitle(tx["title"].format(log=os.path.basename(path), dur=t1 - t0, axis=axis), fontsize=13, fontweight="bold", y=0.995)
    fig.subplots_adjust(top=0.95)

    a1 = fig.add_subplot(gs[0])
    a1.plot(tz, arrival["pos_" + axis][zs], color="tab:blue", lw=2, label=tx["pos"].format(axis=axis))
    a1.set_ylabel(tx["pos_y"], color="tab:blue")
    a1b = a1.twinx()
    a1b.plot(tz, arrival["acc_" + axis][zs], color="tab:orange", lw=1, alpha=0.8, label=tx["acc"].format(axis=axis))
    a1b.set_ylabel(tx["acc_y"], color="tab:orange")
    a1.set_title(tx["p1"])
    a1.legend(loc="upper left", fontsize=8); a1b.legend(loc="upper right", fontsize=8)
    a1.set_xlabel(tx["time"])

    a2 = fig.add_subplot(gs[1], sharex=a1)
    a2.plot(tz, arrival["vimu_" + axis][zs], color="black", lw=1.6, label=tx["vimu"])
    a2.plot(tz, arrival["vlh_" + axis][zs], color="tab:blue", lw=1.6, label=tx["vlh"])
    a2.set_title(tx["p2"].format(lag=lag))
    a2.set_ylabel(tx["vel_y"]); a2.legend(loc="upper right", fontsize=8); a2.set_xlabel(tx["time"])

    a3 = fig.add_subplot(gs[2], sharex=a1)
    vl = arrival["vlh_" + axis]
    vl_back = np.r_[vl[shift:], np.full(shift, np.nan)] if shift > 0 else vl
    a3.plot(tz, arrival["vimu_" + axis][zs], color="black", lw=1.6, label=tx["vimu"])
    a3.plot(tz, vl_back[zs], color="tab:green", lw=1.6, label=tx["vlh_back"].format(lag=lag))
    a3.set_title(tx["p3"].format(lag=lag))
    a3.set_ylabel(tx["vel_y"]); a3.legend(loc="upper right", fontsize=8); a3.set_xlabel(tx["time"])

    a4 = fig.add_subplot(gs[3])
    show = (lags >= -50) & (lags <= 350)                 # the peaks live here
    c_arr, c_ct = res["arrival"]["curve"], res["ctime"]["curve"]
    a4.plot(lags[show], c_arr[show], color="tab:blue", lw=2.5, label=tx["c_arr"])
    a4.plot(lags[show], c_ct[show], color="tab:purple", lw=2.5, ls="--", label=tx["c_ct"])
    if viso is not None:
        a4.axvline(viso, color="tab:red", lw=2, ls=":", label=f"VISO_DELAY_MS = {viso:.0f}")
    bottom = max(0.0, min(np.nanmin(c_arr[show]), np.nanmin(c_ct[show])) - 0.05)
    for x, y, col, name, dx, ha in ((lag, corr, "tab:blue", "T", 12, "left"), (lag_ct, corr_ct, "tab:purple", "Lc", -12, "right")):
        a4.plot([x], [y], "o", color=col, ms=9, zorder=5)
        a4.vlines(x, bottom, y, color=col, lw=1.2, ls="--")
        a4.annotate(f"{tx['c_peak']} {name} = {x:.0f} ms\n(corr {y:.2f})", (x, y), xytext=(dx, 12), textcoords="offset points",
                    ha=ha, color=col, fontweight="bold", fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="none", alpha=0.85))
    a4.set_title(tx["p4"])
    a4.text(0.5, -0.30, tx["corr_help"], transform=a4.transAxes, ha="center", va="top", fontsize=9, color="0.25", wrap=True)
    a4.set_xlabel(tx["shift"]); a4.set_ylabel(tx["corr"])
    a4.set_xlim(-50, 350); a4.set_ylim(bottom, 1.08); a4.grid(alpha=0.3)
    a4.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9, borderaxespad=0)

    a5 = fig.add_subplot(gs[4]); a5.axis("off")
    a5.set_title(tx["tab_title"], fontsize=11, fontweight="bold", loc="left")
    sel = np.where((rd["arrival"] >= z0) & (rd["arrival"] <= z1))[0][:10]   # readings of the zoomed stretch
    rows = [[f"{k + 1}", f"{rd['arrival'][k]:.3f}", f"{rd['capture'][k]:.3f}", f"{rd['seen'][k]:.0f}",
             f"{rd['ekf_time'][k]:.3f}", f"{rd['assumed'][k]:.0f}", f"{rd['real'][k]:.0f}", f"{rd['diff'][k]:+.0f}"] for k in sel]
    table = a5.table(cellText=rows, colLabels=tx["cols"], loc="upper center", cellLoc="center", bbox=[0.0, 0.24, 1.0, 0.70])
    table.auto_set_font_size(False); table.set_fontsize(8.5)
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("0.8")
        if r == 0:                                   # two-line headings: taller row, drawn above the body
            cell.set_height(cell.get_height() * 2.0); cell.set_y(cell.get_y())
            cell.set_facecolor("#e8eef7"); cell.set_text_props(fontweight="bold")
        elif c == 7:
            cell.set_text_props(color="tab:green" if good else "tab:red", fontweight="bold")
    help_text = tx["col_help"].format(v=f"{viso:.0f} ms" if viso is not None else "?", lc=lag_ct)
    cut = help_text.find("·", len(help_text) // 2)    # two lines
    a5.text(0.0, 0.20, help_text[:cut].strip() + "\n" + help_text[cut + 1:].strip(),
            transform=a5.transAxes, fontsize=7.8, color="0.3", va="top")
    colour = "tab:green" if good else "tab:red"
    a5.text(0.0, 0.06, summary, transform=a5.transAxes, fontsize=9.5, va="top", fontweight="bold", color="0.15")
    a5.text(0.0, -0.04, verdict_text, transform=a5.transAxes, fontsize=10.5, va="top", fontweight="bold", color=colour)

    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"figure: {out}")
    if args.window:
        plt.show()
    elif not args.no_show:                                # open the PNG with the system viewer
        try:
            if sys.platform.startswith("win"):
                os.startfile(out)                         # noqa: S606 (Windows only)
            else:
                import subprocess
                subprocess.Popen(["xdg-open" if sys.platform.startswith("linux") else "open", out])
        except OSError as error:
            print(f"open the figure by hand: {error}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
