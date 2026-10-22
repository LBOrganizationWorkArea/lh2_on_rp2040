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

The figure and the terminal also give the same as a step-by-step example with this log's numbers.

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
        "steps_title": "Esempio con un messaggio (valori medi di questo log)",
        "s1": "t = 0 ms: il drone è nel punto P.",
        "s2": "t = {lc:.0f} ms: il firmware ha calcolato P (sensori + filtro) e lo timbra \"catturato ora\".",
        "s3": "t = {t:.0f} ms: il messaggio arriva al Pixhawk.",
        "s4": "ArduPilot legge il timbro e capisce che il dato è stato catturato {a:.0f} ms fa, cioè a {t:.0f} − {a:.0f} = {lc:.0f} ms.",
        "s5": "Toglie VISO_DELAY_MS: {lc:.0f} − {v:.0f} = {res:+.0f} ms.",
        "ok": "≈ 0: l'EKF usa l'istante giusto. VISO_DELAY_MS = {v:.0f} va bene.",
        "up": "L'EKF usa un istante troppo recente: aumenta VISO_DELAY_MS a circa {rec:.0f}.",
        "down": "L'EKF guarda troppo indietro: diminuisci VISO_DELAY_MS a circa {rec:.0f}.",
        "nov": "VISO_DELAY_MS non è nel log: impostalo a circa {rec:.0f}.",
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
        "steps_title": "Example with one message (average values of this log)",
        "s1": "t = 0 ms: the drone is at point P.",
        "s2": "t = {lc:.0f} ms: the firmware has computed P (sensors + filter) and stamps it \"captured now\".",
        "s3": "t = {t:.0f} ms: the message reaches the Pixhawk.",
        "s4": "ArduPilot reads the stamp: the data was captured {a:.0f} ms ago, i.e. at {t:.0f} − {a:.0f} = {lc:.0f} ms.",
        "s5": "It subtracts VISO_DELAY_MS: {lc:.0f} − {v:.0f} = {res:+.0f} ms.",
        "ok": "≈ 0: the EKF uses the right moment. VISO_DELAY_MS = {v:.0f} is right.",
        "up": "The EKF uses too recent a moment: raise VISO_DELAY_MS to about {rec:.0f}.",
        "down": "The EKF looks too far back: lower VISO_DELAY_MS to about {rec:.0f}.",
        "nov": "VISO_DELAY_MS is not in the log: set it to about {rec:.0f}.",
    },
}


def steps(tx, total, lc, viso):
    """The step-by-step example with this log's numbers, and the verdict."""
    rec = 5 * round(lc / 5)
    lines = [tx["s1"], tx["s2"].format(lc=lc), tx["s3"].format(t=total),
             tx["s4"].format(a=total - lc, t=total, lc=lc)]
    if viso is None:
        return lines + [tx["nov"].format(rec=rec)]
    res = lc - viso
    lines.append(tx["s5"].format(lc=lc, v=viso, res=res) + " " +
                 (tx["ok"].format(v=viso) if abs(res) <= 15 else (tx["up"] if res > 0 else tx["down"]).format(rec=rec)))
    return lines


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
    ap.add_argument("--no-show", action="store_true", help="only write the PNG")
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
    lines = steps(tx, lag, lag_ct, viso)

    print(f"analysed {t1 - t0:.0f} s ({t0:.0f}-{t1:.0f} s)")
    print(f"total delay T (motion -> arrival at the FC, VISP.TimeUS): {lag:.0f} ms  (correlation {corr:.2f}; "
          f"N {res['arrival']['N'][0]:.0f} ms, E {res['arrival']['E'][0]:.0f} ms)")
    print(f"ArduPilot's sample time (VISP.CTimeMS) lags the motion by Lc = {lag_ct:.0f} ms (correlation {corr_ct:.2f}): "
          f"VISO_DELAY_MS must cover this")
    print(f"VISO_DELAY_MS in the log: {viso}\n")
    plain = lambda text: text.replace("−", "-").replace("≈", "~")   # Windows consoles (cp1252)
    print(plain(tx["steps_title"]) + ":")
    for i, line in enumerate(lines, 1):
        print(f"  {i}. {plain(line)}")

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # zoom on the stretch with the most motion
    t = arrival["t"]
    energy = np.convolve(arrival["vimu_" + axis] ** 2, np.ones(int(args.zoom / DT)), "same")
    centre = t[int(np.argmax(energy))]
    z0, z1 = max(t[0], centre - args.zoom / 2), min(t[-1], centre + args.zoom / 2)
    zs = (t >= z0) & (t <= z1)
    tz = t[zs] - z0
    shift = int(round(lag / 1000 / DT))

    fig = plt.figure(figsize=(13, 12.5))
    gs = fig.add_gridspec(5, 1, height_ratios=[1, 1, 1, 0.9, 0.75], hspace=0.6)
    fig.suptitle(tx["title"].format(log=os.path.basename(path), dur=t1 - t0, axis=axis), fontsize=13, fontweight="bold")

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
    a4.plot(lags, res["arrival"]["curve"], color="tab:blue", lw=2, label=tx["c_arr"])
    a4.plot(lags, res["ctime"]["curve"], color="tab:purple", lw=2, ls="--", label=tx["c_ct"])
    a4.axvline(lag, color="tab:blue", lw=1); a4.axvline(lag_ct, color="tab:purple", lw=1, ls="--")
    if viso is not None:
        a4.axvline(viso, color="tab:red", lw=1.5, ls=":", label=f"VISO_DELAY_MS = {viso:.0f}")
    a4.annotate(f"T = {lag:.0f} ms", (lag, corr), xytext=(8, -18), textcoords="offset points", color="tab:blue", fontweight="bold")
    a4.annotate(f"Lc = {lag_ct:.0f} ms", (lag_ct, corr_ct), xytext=(-80, -18), textcoords="offset points", color="tab:purple", fontweight="bold")
    a4.set_title(tx["p4"])
    a4.set_xlabel(tx["shift"]); a4.set_ylabel(tx["corr"])
    a4.set_xlim(lags[0], lags[-1]); a4.set_ylim(min(0, np.nanmin(res["arrival"]["curve"])), 1.0); a4.grid(alpha=0.3)
    a4.legend(loc="lower left", fontsize=8)

    a5 = fig.add_subplot(gs[4]); a5.axis("off")
    a5.text(0.0, 1.0, tx["steps_title"], fontsize=11, fontweight="bold", va="top", transform=a5.transAxes)
    for i, line in enumerate(lines, 1):
        a5.text(0.0, 1.0 - 0.19 * i, f"{i}.  {line}", fontsize=10, va="top", transform=a5.transAxes,
                fontweight="bold" if i == len(lines) else "normal",
                color="black" if i < len(lines) else
                ("tab:green" if viso is not None and abs(lag_ct - viso) <= 15 else "tab:red"))

    out = args.out or os.path.splitext(path)[0] + "_delay.png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"figure: {out}")
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
