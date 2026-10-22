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
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_ext_nav_delay as P  # noqa: E402  (same folder)

DT = P.DT


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

    print(f"asse {axis}, {t1 - t0:.0f} s di movimento, {len(t)} punti ogni 5 ms")
    print(f"inversioni: {len(gaps)}, distanza mediana {np.median(gaps):.0f} ms (da {np.percentile(gaps, 10):.0f} a {np.percentile(gaps, 90):.0f})")
    print(f"somiglianza massima {cmax:.3f} con spostamento Lc = {lc:.0f} ms;  VISO_DELAY_MS = {viso}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tz = t[zs] - z0
    fig = plt.figure(figsize=(14, 30))
    gs = fig.add_gridspec(8, 3, height_ratios=[1, 1, 1, 1, 1.1, 0.9, 1.2, 1.1], hspace=0.95, wspace=0.25)
    fig.suptitle(f"Come si ricava il ritardo nascosto Lc — {os.path.basename(path)}  (asse {axis}, {t1 - t0:.0f} s di movimento)",
                 fontsize=15, fontweight="bold", y=0.995)
    fig.subplots_adjust(top=0.965, bottom=0.03, left=0.06, right=0.98)

    # ---- Passo 1
    a = fig.add_subplot(gs[0, :])
    a.plot(tz, body[zs], color="0.6", lw=1, label=f"accelerometro nel sistema del drone (IMU.Acc{'X' if axis == 'N' else 'Y'})")
    a.plot(tz, s["acc_" + axis][zs], color="tab:orange", lw=1.8, label=f"stessa accelerazione ruotata nella stanza (asse {axis}, con l'assetto ATT)")
    a.set_title("PASSO 1a — Accelerometro del Pixhawk: misura nell'istante VERO del movimento, nel sistema del drone → lo ruoto nel sistema della stanza",
                fontsize=11, loc="left")
    a.set_ylabel("accelerazione [m/s²]"); a.set_xlabel("tempo [s]"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[1, :], sharex=fig.axes[0])
    m = (stamp >= z0) & (stamp <= z1)
    a.plot(stamp[m] - z0, visp[m, col], "o", color="tab:purple", ms=4, label="posizione lighthouse, messa all'istante del suo timbro (VISP.CTimeMS)")
    a.plot(tz, s["pos_" + axis][zs], color="tab:purple", lw=1, alpha=0.5, label="unite (ricampionate ogni 5 ms)")
    a.set_title(f"PASSO 1b — Posizioni dalle lighthouse (VISP.P{'X' if axis == 'N' else 'Y'}): un punto ogni 100 ms, ognuno all'istante del timbro",
                fontsize=11, loc="left")
    a.set_ylabel("posizione [m]"); a.set_xlabel("tempo [s]"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)

    # ---- Passo 2
    a = fig.add_subplot(gs[2, :2], sharex=fig.axes[0])
    a.plot(tz, v_int_raw[zs] - v_int_raw[zs].mean(), color="0.6", lw=1, label="accelerazione × tempo, sommata (deriva)")
    a.plot(tz, vi[zs], color="black", lw=1.8, label="tolta la media degli ultimi 1.5 s → velocità")
    a.set_title("PASSO 2a — Dall'accelerometro alla velocità: somma di accelerazione × tempo", fontsize=11, loc="left")
    a.set_ylabel("velocità [m/s]"); a.set_xlabel("tempo [s]"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[2, 2])
    a.axis("off")
    a.text(0, 0.95, "Perché la velocità?\n\nAccelerazione e posizione non si\npossono confrontare direttamente.\n"
           "Le trasformo nella stessa cosa:\nla velocità.\n\nLa deriva: integrando l'accelerometro\n"
           "i piccoli errori si accumulano;\ntogliendo la media degli ultimi 1.5 s\nresta solo il movimento.",
           va="top", fontsize=9.5)
    a = fig.add_subplot(gs[3, :2], sharex=fig.axes[0])
    a.plot(fd_t[ok] - z0, fd_v[ok], "o", color="tab:purple", ms=3, alpha=0.6, label="(posizione successiva − posizione) / tempo fra i due timbri")
    a.plot(tz, vl[zs], color="tab:purple", lw=1.8, label="ricampionata ogni 5 ms, lisciata, tolta la media → velocità")
    a.set_title("PASSO 2b — Dalle posizioni lighthouse alla velocità: spostamento / tempo", fontsize=11, loc="left")
    a.set_ylabel("velocità [m/s]"); a.set_xlabel("tempo [s]"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[3, 2])
    a.axis("off")
    a.text(0, 0.95, f"Ora ho due curve della\nSTESSA velocità:\n\n● nera: dall'accelerometro,\n   all'istante vero\n"
           f"● viola: dalle lighthouse,\n   all'istante del timbro\n\nEntrambe ricampionate ogni 5 ms:\n{len(t)} punti in {t1 - t0:.1f} s.",
           va="top", fontsize=9.5)

    # ---- Passo 3
    a = fig.add_subplot(gs[4, :2], sharex=fig.axes[0])
    a.axhline(0, color="0.5", lw=0.8)
    a.plot(tz, vi[zs], color="black", lw=1.8, label="velocità dall'accelerometro (istante vero)")
    a.plot(tz, vl_s[zs], color="tab:purple", lw=1.8, label="velocità dalle lighthouse (istante del timbro)")
    for x, y in pairs:
        if z0 <= x <= z1:
            a.plot([x - z0], [0], "o", color="black", ms=7, zorder=5)
            a.plot([y - z0], [0], "o", color="tab:purple", ms=7, zorder=5)
            a.annotate(f"{1000 * (y - x):.0f} ms", ((x + y) / 2 - z0, 0), xytext=(0, 14), textcoords="offset points",
                       ha="center", fontsize=9, fontweight="bold", color="tab:red",
                       arrowprops=dict(arrowstyle="-", color="tab:red", lw=0.8))
    a.set_title("PASSO 3 — Le due velocità insieme: ogni inversione del moto (passaggio per lo zero) arriva DOPO nella curva viola",
                fontsize=11, loc="left")
    a.set_ylabel("velocità [m/s]"); a.set_xlabel("tempo [s]"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[4, 2])
    a.hist(gaps, bins=np.arange(-60, 260, 20), color="tab:red", alpha=0.7, edgecolor="white")
    a.axvline(np.median(gaps), color="tab:red", lw=2)
    a.set_title(f"distanza fra le inversioni,\ntutte le {len(gaps)} della registrazione", fontsize=10)
    a.set_xlabel("ms"); a.set_ylabel("quante")
    a.text(0.97, 0.95, f"mediana {np.median(gaps):.0f} ms\nmolto variabile:\nun ciclo solo non basta",
           transform=a.transAxes, ha="right", va="top", fontsize=9)

    # ---- Passo 4: three shifts
    for k, (sh, title) in enumerate(((0.0, "spostamento 0 ms"), (lc, f"spostamento {lc:.0f} ms (il migliore)"), (150.0, "spostamento 150 ms"))):
        a = fig.add_subplot(gs[5, k])
        n = int(round(sh / 1000 / DT))
        vl_shift = np.r_[vl_s[n:], np.full(n, np.nan)] if n > 0 else vl_s
        sim = np.interp(sh, lags, curve)
        a.plot(tz, vi[zs], color="black", lw=1.5)
        a.plot(tz, vl_shift[zs], color="tab:purple" if k != 1 else "tab:green", lw=1.5)
        a.set_title(f"{title}\nsomiglianza {sim:.3f}", fontsize=10, color="tab:green" if k == 1 else "black",
                    fontweight="bold" if k == 1 else "normal")
        a.set_xlabel("tempo [s]"); a.grid(alpha=0.3)
        if k == 0:
            a.set_ylabel("velocità [m/s]")
        if k == 0:
            step4_top = a.get_position().y1
    fig.text(0.06, step4_top + 0.022, "PASSO 4 — Sposto la curva viola indietro di 0, 5, 10, … 600 ms e ogni volta misuro quanto somiglia "
             "alla nera su tutti i punti (1 = identiche, 0 = per niente). Tre esempi:", fontsize=11)

    # ---- Passo 5
    a = fig.add_subplot(gs[6, :2])
    a.plot(lags, curve, color="tab:purple", lw=2.2)
    shown = [0, 25, 40, 50, 55, 60, 75, 100, 150, 200]
    for L in shown:
        y = curve[list(lags).index(L)]
        a.plot([L], [y], "o", color="tab:purple", ms=5)
    # parabola through the best three points
    xs = lags[ib - 1:ib + 2]; ys = curve[ib - 1:ib + 2]
    if len(xs) == 3:
        cf = np.polyfit(xs, ys, 2); xp = np.linspace(xs[0] - 10, xs[-1] + 10, 50)
        a.plot(xp, np.polyval(cf, xp), color="tab:green", lw=2.5, label="parabola sui 3 valori più alti")
    a.plot([lc], [cmax], "*", color="tab:green", ms=16, zorder=6)
    a.annotate(f"picco: Lc = {lc:.0f} ms\nsomiglianza {cmax:.3f}", (lc, cmax), xytext=(25, -40), textcoords="offset points",
               fontsize=11, fontweight="bold", color="tab:green", arrowprops=dict(arrowstyle="->", color="tab:green"))
    if viso is not None:
        a.axvline(viso, color="tab:red", ls=":", lw=2, label=f"VISO_DELAY_MS = {viso:.0f}")
    a.set_xlim(-100, 400); a.set_ylim(max(0, np.min(curve[(lags >= -100) & (lags <= 400)]) - 0.03), 1.0)
    a.set_title("PASSO 5 — La somiglianza per ogni spostamento: lo spostamento con la somiglianza più alta è il ritardo nascosto",
                fontsize=11, loc="left")
    a.set_xlabel("spostamento della curva viola [ms]"); a.set_ylabel("somiglianza (correlazione)")
    a.legend(fontsize=9, loc="lower left"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[6, 2]); a.axis("off")
    rows = [[f"{L} ms", f"{curve[list(lags).index(L)]:.3f}"] for L in shown]
    best = int(np.argmax([curve[list(lags).index(L)] for L in shown]))
    tab = a.table(cellText=rows, colLabels=["spostamento", "somiglianza"], loc="upper center", cellLoc="center", bbox=[0.1, 0.05, 0.8, 0.9])
    tab.auto_set_font_size(False); tab.set_fontsize(10)
    for (r, c), cell in tab.get_celld().items():
        cell.set_edgecolor("0.8")
        if r == 0:
            cell.set_facecolor("#e8eef7"); cell.set_text_props(fontweight="bold")
        elif r - 1 == best:
            cell.set_facecolor("#dff3e3"); cell.set_text_props(fontweight="bold")

    # ---- Passo 6
    a = fig.add_subplot(gs[7, :2], sharex=fig.axes[0])
    n = int(round(lc / 1000 / DT))
    vl_back = np.r_[vl_s[n:], np.full(n, np.nan)] if n > 0 else vl_s
    a.plot(tz, vi[zs], color="black", lw=1.8, label="velocità dall'accelerometro (istante vero)")
    a.plot(tz, vl_back[zs], color="tab:green", lw=1.8, label=f"velocità lighthouse spostata di {lc:.0f} ms")
    a.set_title(f"PASSO 6 — Spostata di {lc:.0f} ms, la curva lighthouse coincide con quella vera: ogni posizione descrive il drone "
                f"di {lc:.0f} ms PRIMA del suo timbro", fontsize=11, loc="left")
    a.set_ylabel("velocità [m/s]"); a.set_xlabel("tempo [s]"); a.legend(fontsize=8, loc="upper right"); a.grid(alpha=0.3)
    a = fig.add_subplot(gs[7, 2]); a.axis("off")
    good = viso is not None and abs(lc - viso) <= 15
    concl = (f"Risultato\n\nritardo nascosto Lc = {lc:.0f} ms\n(sensori + filtro, prima del timbro:\n"
             f"non è scritto nel messaggio)\n\nVISO_DELAY_MS = {viso:.0f}\ndifferenza {viso - lc:+.0f} ms\n\n"
             + ("→ va bene: l'EKF usa\n   l'istante giusto" if good else f"→ imposta VISO_DELAY_MS\n   a circa {5 * round(lc / 5):.0f}")
             if viso is not None else f"Risultato\n\nritardo nascosto Lc = {lc:.0f} ms\n\n→ imposta VISO_DELAY_MS a circa {5 * round(lc / 5):.0f}")
    a.text(0, 0.95, concl, va="top", fontsize=11, fontweight="bold", color="tab:green" if good else "tab:red")

    out = args.out or os.path.splitext(path)[0] + "_spiegazione.png"
    fig.savefig(out, dpi=110)
    print(f"figura: {out}")
    if not args.no_show:
        try:
            if sys.platform.startswith("win"):
                os.startfile(out)  # noqa: S606
            else:
                import subprocess
                subprocess.Popen(["xdg-open" if sys.platform.startswith("linux") else "open", out])
        except OSError as error:
            print(f"apri la figura a mano: {error}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
