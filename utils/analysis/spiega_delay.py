#!/usr/bin/env python3
"""
spiega_delay.py — come si ricava il ritardo "nascosto" (Lc) della posizione lighthouse,
passo per passo e con i dati del log, in una sola figura.

  Passo 1  i due dati grezzi: accelerometro (istante vero) e posizioni lighthouse (all'istante del timbro)
  Passo 2  entrambi trasformati in velocità
  Passo 3  le due velocità a confronto: le inversioni del moto arrivano dopo nelle lighthouse
  Passo 4  la curva lighthouse spostata di vari millisecondi: quanto somiglia a quella dell'accelerometro
  Passo 5  la somiglianza per ogni spostamento: il picco è il ritardo nascosto Lc
  Passo 6  risultato e confronto con VISO_DELAY_MS

Gli stessi calcoli di plot_ext_nav_delay.py (che deve stare nella stessa cartella).

Uso (Windows: doppio clic su spiega_delay.bat, oppure trascinaci sopra un .bin):
    python spiega_delay.py                  # il .bin più recente in Documenti\\Mission Planner\\logs
    python spiega_delay.py volo.bin --zoom 6 --out spiegazione.png
    python spiega_delay.py volo.bin --lang en      # figure in English
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_ext_nav_delay as P  # noqa: E402  (same folder)

DT = P.DT


TEXT = {
    "it": {
        "title": "Come si ricava il ritardo nascosto Lc — {log}  (asse {axis}, {dur:.0f} s di movimento)",
        "time": "tempo [s]", "acc_y": "accelerazione [m/s²]", "pos_y": "posizione [m]", "vel_y": "velocità [m/s]",
        "acc_body": "accelerometro nel sistema del drone (IMU.Acc{c})",
        "acc_room": "stessa accelerazione ruotata nella stanza (asse {axis}, con l'assetto ATT)",
        "s1a": "PASSO 1a — Accelerometro del Pixhawk: misura nell'istante VERO del movimento, nel sistema del drone → lo ruoto nel sistema della stanza",
        "pos_pts": "posizione lighthouse, messa all'istante del suo timbro (VISP.CTimeMS)",
        "pos_line": "unite (ricampionate ogni 5 ms)",
        "s1b": "PASSO 1b — Posizioni dalle lighthouse (VISP.P{c}): un punto ogni 100 ms, ognuno all'istante del timbro",
        "vint_raw": "accelerazione × tempo, sommata (deriva)", "vint": "tolta la media degli ultimi 1.5 s → velocità",
        "s2a": "PASSO 2a — Dall'accelerometro alla velocità: somma di accelerazione × tempo",
        "why": "Perché la velocità?\n\nAccelerazione e posizione non si\npossono confrontare direttamente.\n"
               "Le trasformo nella stessa cosa:\nla velocità.\n\nLa deriva: integrando l'accelerometro\n"
               "i piccoli errori si accumulano;\ntogliendo la media degli ultimi 1.5 s\nresta solo il movimento.",
        "fd": "(posizione successiva − posizione) / tempo fra i due timbri",
        "vlh": "ricampionata ogni 5 ms, lisciata, tolta la media → velocità",
        "s2b": "PASSO 2b — Dalle posizioni lighthouse alla velocità: spostamento / tempo",
        "two": "Ora ho due curve della\nSTESSA velocità:\n\n● nera: dall'accelerometro,\n   all'istante vero\n"
               "● viola: dalle lighthouse,\n   all'istante del timbro\n\nEntrambe ricampionate ogni 5 ms:\n{n} punti in {dur:.1f} s.",
        "v_true": "velocità dall'accelerometro (istante vero)", "v_stamp": "velocità dalle lighthouse (istante del timbro)",
        "s3": "PASSO 3 — Le due velocità insieme: ogni inversione del moto (passaggio per lo zero) arriva DOPO nella curva viola",
        "hist_t": "distanza fra le inversioni,\ntutte le {n} della registrazione", "hist_y": "quante",
        "hist_txt": "mediana {med:.0f} ms\nmolto variabile:\nun ciclo solo non basta",
        "shift": "spostamento {sh:.0f} ms", "shift_best": "spostamento {sh:.0f} ms (il migliore)", "sim": "somiglianza",
        "s4": "PASSO 4 — Sposto la curva viola indietro di 0, 5, 10, … 600 ms e ogni volta misuro quanto somiglia\n"
              "alla nera su tutti i punti (1 = identiche, 0 = per niente). Tre esempi:",
        "parab": "parabola sui 3 valori più alti", "peak": "picco: Lc = {lc:.0f} ms\nsomiglianza {c:.3f}",
        "s5": "PASSO 5 — La somiglianza per ogni spostamento: lo spostamento con la somiglianza più alta è il ritardo nascosto",
        "shift_x": "spostamento della curva viola [ms]", "sim_y": "somiglianza (correlazione)",
        "tab_cols": ["spostamento", "somiglianza"],
        "v_back": "velocità lighthouse spostata di {lc:.0f} ms",
        "s6": "PASSO 6 — Spostata di {lc:.0f} ms, la curva lighthouse coincide con quella vera: ogni posizione descrive il drone "
              "di {lc:.0f} ms PRIMA del suo timbro",
        "res": "Risultato\n\nritardo nascosto Lc = {lc:.0f} ms\n(sensori + filtro, prima del timbro:\n"
               "non è scritto nel messaggio)\n\nVISO_DELAY_MS = {v:.0f}\ndifferenza {d:+.0f} ms\n\n",
        "res_ok": "→ va bene: l'EKF usa\n   l'istante giusto", "res_set": "→ imposta VISO_DELAY_MS\n   a circa {rec:.0f}",
        "res_nov": "Risultato\n\nritardo nascosto Lc = {lc:.0f} ms\n\n→ imposta VISO_DELAY_MS a circa {rec:.0f}",
        "c_axis": "asse {axis}, {dur:.0f} s di movimento, {n} punti ogni 5 ms",
        "c_rev": "inversioni: {n}, distanza mediana {med:.0f} ms (da {lo:.0f} a {hi:.0f})",
        "c_peak": "somiglianza massima {c:.3f} con spostamento Lc = {lc:.0f} ms;  VISO_DELAY_MS = {v}",
        "c_fig": "figura: {path}", "c_open": "apri la figura a mano",
    },
    "en": {
        "title": "How the hidden delay Lc is found — {log}  (axis {axis}, {dur:.0f} s of motion)",
        "time": "time [s]", "acc_y": "acceleration [m/s²]", "pos_y": "position [m]", "vel_y": "velocity [m/s]",
        "acc_body": "accelerometer in the drone frame (IMU.Acc{c})",
        "acc_room": "same acceleration turned into the room (axis {axis}, with the ATT attitude)",
        "s1a": "STEP 1a — Pixhawk accelerometer: measures at the TRUE moment of the motion, in the drone frame → turned into the room frame",
        "pos_pts": "lighthouse position, placed at the moment of its stamp (VISP.CTimeMS)",
        "pos_line": "joined (resampled every 5 ms)",
        "s1b": "STEP 1b — Positions from the lighthouses (VISP.P{c}): one point every 100 ms, each at the moment of its stamp",
        "vint_raw": "acceleration × time, summed (drift)", "vint": "minus the mean of the last 1.5 s → velocity",
        "s2a": "STEP 2a — From the accelerometer to velocity: sum of acceleration × time",
        "why": "Why velocity?\n\nAcceleration and position cannot\nbe compared directly.\n"
               "Both are turned into the same\nthing: velocity.\n\nDrift: integrating the accelerometer\n"
               "adds up small errors; removing\nthe mean of the last 1.5 s\nleaves only the motion.",
        "fd": "(next position − position) / time between the two stamps",
        "vlh": "resampled every 5 ms, smoothed, mean removed → velocity",
        "s2b": "STEP 2b — From the lighthouse positions to velocity: displacement / time",
        "two": "Now there are two curves of the\nSAME velocity:\n\n● black: from the accelerometer,\n   at the true moment\n"
               "● purple: from the lighthouses,\n   at the moment of the stamp\n\nBoth resampled every 5 ms:\n{n} points in {dur:.1f} s.",
        "v_true": "velocity from the accelerometer (true moment)", "v_stamp": "velocity from the lighthouses (stamp moment)",
        "s3": "STEP 3 — The two velocities together: every motion reversal (zero crossing) comes LATER in the purple curve",
        "hist_t": "gap between reversals,\nall {n} of the recording", "hist_y": "count",
        "hist_txt": "median {med:.0f} ms\nvery variable:\none cycle is not enough",
        "shift": "shift {sh:.0f} ms", "shift_best": "shift {sh:.0f} ms (the best)", "sim": "similarity",
        "s4": "STEP 4 — The purple curve is moved back by 0, 5, 10, … 600 ms and each time its similarity to the black one\n"
              "is measured over all points (1 = identical, 0 = not at all). Three examples:",
        "parab": "parabola through the 3 highest values", "peak": "peak: Lc = {lc:.0f} ms\nsimilarity {c:.3f}",
        "s5": "STEP 5 — Similarity for every shift: the shift with the highest similarity is the hidden delay",
        "shift_x": "shift of the purple curve [ms]", "sim_y": "similarity (correlation)",
        "tab_cols": ["shift", "similarity"],
        "v_back": "lighthouse velocity moved by {lc:.0f} ms",
        "s6": "STEP 6 — Moved by {lc:.0f} ms, the lighthouse curve matches the true one: each position describes the drone "
              "{lc:.0f} ms BEFORE its stamp",
        "res": "Result\n\nhidden delay Lc = {lc:.0f} ms\n(sensors + filter, before the stamp:\n"
               "not written in the message)\n\nVISO_DELAY_MS = {v:.0f}\ndifference {d:+.0f} ms\n\n",
        "res_ok": "→ right: the EKF uses\n   the correct moment", "res_set": "→ set VISO_DELAY_MS\n   to about {rec:.0f}",
        "res_nov": "Result\n\nhidden delay Lc = {lc:.0f} ms\n\n→ set VISO_DELAY_MS to about {rec:.0f}",
        "c_axis": "axis {axis}, {dur:.0f} s of motion, {n} points every 5 ms",
        "c_rev": "reversals: {n}, median gap {med:.0f} ms (from {lo:.0f} to {hi:.0f})",
        "c_peak": "highest similarity {c:.3f} at shift Lc = {lc:.0f} ms;  VISO_DELAY_MS = {v}",
        "c_fig": "figure: {path}", "c_open": "open the figure by hand",
    },
}


def zero_crossings(t, v):
    """Instants where v goes from positive to negative (linear interpolation)."""
    i = np.where((v[:-1] > 0) & (v[1:] <= 0))[0]
    return t[i] + v[i] / (v[i] - v[i + 1]) * (t[i + 1] - t[i])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="file .bin (default: il più recente in Documenti\\Mission Planner\\logs)")
    ap.add_argument("--axis", choices=["auto", "N", "E"], default="auto")
    ap.add_argument("--zoom", type=float, default=6.0, help="secondi mostrati nei grafici nel tempo (default 6)")
    ap.add_argument("--out", help="PNG da scrivere (default: accanto al log)")
    ap.add_argument("--no-show", action="store_true", help="scrive solo il PNG, senza aprirlo")
    ap.add_argument("--lang", choices=["it", "en"], default="it", help="lingua della figura (default it; en = English)")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

    path = args.log or P.newest_log()
    if not path or not os.path.exists(path):
        sys.exit("nessun .bin indicato e nessuno trovato in Documenti\\Mission Planner\\logs")
    print(f"log: {path}")
    d = P.read_log(path)
    t0, t1 = P.motion_window(d)
    if t1 - t0 < 8:
        sys.exit(f"solo {t1 - t0:.1f} s di movimento: registra almeno 20 s di avanti-indietro")
    visp = d["visp"]
    stamp = visp[:, 5]                                    # VISP.CTimeMS: the stamp on the Pixhawk clock
    s = P.signals(d, t0, t1, stamp)                       # lighthouse placed at the stamp
    t = s["t"]
    axis = args.axis if args.axis != "auto" else ("N" if np.std(s["pos_N"]) >= np.std(s["pos_E"]) else "E")
    col = 1 if axis == "N" else 2
    viso = d["params"].get("VISO_DELAY_MS")

    # similarity for every shift (N and E averaged, as in plot_ext_nav_delay)
    lags = np.arange(-100, 600 + 1e-9, 5.0)
    cn = P.correlate(s["vimu_N"], s["vlh_N"], lags)
    ce = P.correlate(s["vimu_E"], s["vlh_E"], lags)
    curve = (cn + ce) / 2
    lc, cmax = P.peak(lags, curve)
    ib = int(np.argmax(curve))

    # zoomed stretch: the one with the most motion on the chosen axis
    energy = np.convolve(s["vimu_" + axis] ** 2, np.ones(int(args.zoom / DT)), "same")
    centre = t[int(np.argmax(energy))]
    z0, z1 = max(t[0], centre - args.zoom / 2), min(t[-1], centre + args.zoom / 2)
    zs = (t >= z0) & (t <= z1)

    # step 1 extras: body-frame acceleration on the same axis
    imu = d["imu"]
    body_col = 1 if axis == "N" else 2
    body = np.interp(t, imu[:, 0], imu[:, body_col])
    # step 2 extras: integrated velocity before the drift removal, raw finite differences of the positions
    v_int_raw = np.cumsum(s["acc_" + axis]) * DT
    vz = visp[(stamp >= z0) & (stamp <= z1)]
    fd_t = 0.5 * (vz[1:, 5] + vz[:-1, 5])
    with np.errstate(divide="ignore", invalid="ignore"):
        fd_v = np.diff(vz[:, col]) / np.diff(vz[:, 5])
    ok = np.isfinite(fd_v) & (np.abs(fd_v) < 10)
    # step 3: zero crossings of both velocities (whole recording) and their gaps
    vi, vl = s["vimu_" + axis], s["vlh_" + axis]
    vl_s = np.convolve(vl, np.ones(15) / 15, "same")
    ci, cl = zero_crossings(t, vi), zero_crossings(t, vl_s)
    pairs = [(a, cl[(cl > a - 0.05) & (cl < a + 0.25)][0]) for a in ci if np.any((cl > a - 0.05) & (cl < a + 0.25))]
    gaps = np.array([1000 * (b - a) for a, b in pairs])

    tx = TEXT[args.lang]
    ax_body, ax_pos = ("X", "X") if axis == "N" else ("Y", "Y")
    print(tx["c_axis"].format(axis=axis, dur=t1 - t0, n=len(t)))
    print(tx["c_rev"].format(n=len(gaps), med=np.median(gaps), lo=np.percentile(gaps, 10), hi=np.percentile(gaps, 90)))
    print(tx["c_peak"].format(c=cmax, lc=lc, v=viso))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tz = t[zs] - z0
    fig = plt.figure(figsize=(14, 30))
    gs = fig.add_gridspec(8, 3, height_ratios=[1, 1, 1, 1, 1.1, 0.9, 1.2, 1.1], hspace=0.95, wspace=0.25)
    fig.suptitle(tx["title"].format(log=os.path.basename(path), axis=axis, dur=t1 - t0), fontsize=15, fontweight="bold", y=0.995)
    fig.subplots_adjust(top=0.965, bottom=0.03, left=0.06, right=0.98)
    title = dict(fontsize=11, loc="left")

    # ---- step 1
    a = fig.add_subplot(gs[0, :])
    a.plot(tz, body[zs], color="0.6", lw=1, label=tx["acc_body"].format(c=ax_body))
    a.plot(tz, s["acc_" + axis][zs], color="tab:orange", lw=1.8, label=tx["acc_room"].format(axis=axis))
    a.set_title(tx["s1a"], **title)
    a.set_ylabel(tx["acc_y"]); a.set_xlabel(tx["time"]); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[1, :], sharex=fig.axes[0])
    m = (stamp >= z0) & (stamp <= z1)
    a.plot(stamp[m] - z0, visp[m, col], "o", color="tab:purple", ms=4, label=tx["pos_pts"])
    a.plot(tz, s["pos_" + axis][zs], color="tab:purple", lw=1, alpha=0.5, label=tx["pos_line"])
    a.set_title(tx["s1b"].format(c=ax_pos), **title)
    a.set_ylabel(tx["pos_y"]); a.set_xlabel(tx["time"]); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)

    # ---- step 2
    a = fig.add_subplot(gs[2, :2], sharex=fig.axes[0])
    a.plot(tz, v_int_raw[zs] - v_int_raw[zs].mean(), color="0.6", lw=1, label=tx["vint_raw"])
    a.plot(tz, vi[zs], color="black", lw=1.8, label=tx["vint"])
    a.set_title(tx["s2a"], **title)
    a.set_ylabel(tx["vel_y"]); a.set_xlabel(tx["time"]); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[2, 2]); a.axis("off")
    a.text(0, 0.95, tx["why"], va="top", fontsize=9.5)
    a = fig.add_subplot(gs[3, :2], sharex=fig.axes[0])
    a.plot(fd_t[ok] - z0, fd_v[ok], "o", color="tab:purple", ms=3, alpha=0.6, label=tx["fd"])
    a.plot(tz, vl[zs], color="tab:purple", lw=1.8, label=tx["vlh"])
    a.set_title(tx["s2b"], **title)
    a.set_ylabel(tx["vel_y"]); a.set_xlabel(tx["time"]); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[3, 2]); a.axis("off")
    a.text(0, 0.95, tx["two"].format(n=len(t), dur=t1 - t0), va="top", fontsize=9.5)

    # ---- step 3
    a = fig.add_subplot(gs[4, :2], sharex=fig.axes[0])
    a.axhline(0, color="0.5", lw=0.8)
    a.plot(tz, vi[zs], color="black", lw=1.8, label=tx["v_true"])
    a.plot(tz, vl_s[zs], color="tab:purple", lw=1.8, label=tx["v_stamp"])
    for x, y in pairs:
        if z0 <= x <= z1:
            a.plot([x - z0], [0], "o", color="black", ms=7, zorder=5)
            a.plot([y - z0], [0], "o", color="tab:purple", ms=7, zorder=5)
            a.annotate(f"{1000 * (y - x):.0f} ms", ((x + y) / 2 - z0, 0), xytext=(0, 14), textcoords="offset points",
                       ha="center", fontsize=9, fontweight="bold", color="tab:red",
                       arrowprops=dict(arrowstyle="-", color="tab:red", lw=0.8))
    a.set_title(tx["s3"], **title)
    a.set_ylabel(tx["vel_y"]); a.set_xlabel(tx["time"]); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[4, 2])
    a.hist(gaps, bins=np.arange(-60, 260, 20), color="tab:red", alpha=0.7, edgecolor="white")
    a.axvline(np.median(gaps), color="tab:red", lw=2)
    a.set_title(tx["hist_t"].format(n=len(gaps)), fontsize=10)
    a.set_xlabel("ms"); a.set_ylabel(tx["hist_y"])
    a.text(0.97, 0.95, tx["hist_txt"].format(med=np.median(gaps)), transform=a.transAxes, ha="right", va="top", fontsize=9)

    # ---- step 4: three shifts
    for k, sh in enumerate((0.0, lc, 150.0)):
        a = fig.add_subplot(gs[5, k])
        n = int(round(sh / 1000 / DT))
        vl_shift = np.r_[vl_s[n:], np.full(n, np.nan)] if n > 0 else vl_s
        sim = np.interp(sh, lags, curve)
        a.plot(tz, vi[zs], color="black", lw=1.5)
        a.plot(tz, vl_shift[zs], color="tab:purple" if k != 1 else "tab:green", lw=1.5)
        head = tx["shift_best"].format(sh=sh) if k == 1 else tx["shift"].format(sh=sh)
        a.set_title(f"{head}\n{tx['sim']} {sim:.3f}", fontsize=10, color="tab:green" if k == 1 else "black",
                    fontweight="bold" if k == 1 else "normal")
        a.set_xlabel(tx["time"]); a.grid(alpha=0.3)
        if k == 0:
            a.set_ylabel(tx["vel_y"])
            step4_top = a.get_position().y1
    fig.text(0.06, step4_top + 0.018, tx["s4"], fontsize=11, va="bottom")

    # ---- step 5
    a = fig.add_subplot(gs[6, :2])
    a.plot(lags, curve, color="tab:purple", lw=2.2)
    shown = [0, 25, 40, 50, 55, 60, 75, 100, 150, 200]
    for L in shown:
        a.plot([L], [curve[list(lags).index(L)]], "o", color="tab:purple", ms=5)
    xs = lags[ib - 1:ib + 2]; ys = curve[ib - 1:ib + 2]          # parabola through the best three points
    if len(xs) == 3:
        cf = np.polyfit(xs, ys, 2); xp = np.linspace(xs[0] - 10, xs[-1] + 10, 50)
        a.plot(xp, np.polyval(cf, xp), color="tab:green", lw=2.5, label=tx["parab"])
    a.plot([lc], [cmax], "*", color="tab:green", ms=16, zorder=6)
    a.annotate(tx["peak"].format(lc=lc, c=cmax), (lc, cmax), xytext=(25, -40), textcoords="offset points",
               fontsize=11, fontweight="bold", color="tab:green", arrowprops=dict(arrowstyle="->", color="tab:green"))
    if viso is not None:
        a.axvline(viso, color="tab:red", ls=":", lw=2, label=f"VISO_DELAY_MS = {viso:.0f}")
    a.set_xlim(-100, 400); a.set_ylim(max(0, np.min(curve[(lags >= -100) & (lags <= 400)]) - 0.03), 1.0)
    a.set_title(tx["s5"], **title)
    a.set_xlabel(tx["shift_x"]); a.set_ylabel(tx["sim_y"])
    a.legend(fontsize=9, loc="lower left"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[6, 2]); a.axis("off")
    rows = [[f"{L} ms", f"{curve[list(lags).index(L)]:.3f}"] for L in shown]
    best = int(np.argmax([curve[list(lags).index(L)] for L in shown]))
    tab = a.table(cellText=rows, colLabels=tx["tab_cols"], loc="upper center", cellLoc="center", bbox=[0.1, 0.05, 0.8, 0.9])
    tab.auto_set_font_size(False); tab.set_fontsize(10)
    for (r, c), cell in tab.get_celld().items():
        cell.set_edgecolor("0.8")
        if r == 0:
            cell.set_facecolor("#e8eef7"); cell.set_text_props(fontweight="bold")
        elif r - 1 == best:
            cell.set_facecolor("#dff3e3"); cell.set_text_props(fontweight="bold")

    # ---- step 6
    a = fig.add_subplot(gs[7, :2], sharex=fig.axes[0])
    n = int(round(lc / 1000 / DT))
    vl_back = np.r_[vl_s[n:], np.full(n, np.nan)] if n > 0 else vl_s
    a.plot(tz, vi[zs], color="black", lw=1.8, label=tx["v_true"])
    a.plot(tz, vl_back[zs], color="tab:green", lw=1.8, label=tx["v_back"].format(lc=lc))
    a.set_title(tx["s6"].format(lc=lc), **title)
    a.set_ylabel(tx["vel_y"]); a.set_xlabel(tx["time"]); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[7, 2]); a.axis("off")
    good = viso is not None and abs(lc - viso) <= 15
    rec = 5 * round(lc / 5)
    if viso is None:
        concl = tx["res_nov"].format(lc=lc, rec=rec)
    else:
        concl = tx["res"].format(lc=lc, v=viso, d=viso - lc) + (tx["res_ok"] if good else tx["res_set"].format(rec=rec))
    a.text(0, 0.95, concl, va="top", fontsize=11, fontweight="bold", color="tab:green" if good else "tab:red")

    out = args.out or os.path.splitext(path)[0] + ("_spiegazione.png" if args.lang == "it" else "_explained.png")
    fig.savefig(out, dpi=110)
    print(tx["c_fig"].format(path=out))
    if not args.no_show:
        try:
            if sys.platform.startswith("win"):
                os.startfile(out)  # noqa: S606
            else:
                import subprocess
                subprocess.Popen(["xdg-open" if sys.platform.startswith("linux") else "open", out])
        except OSError as error:
            print(f"{tx['c_open']}: {error}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
