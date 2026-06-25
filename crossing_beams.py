import serial
import math
import time
import sys
import serial.tools.list_ports
from pathlib import Path

# ============================================================
# AUTO DETECT PICO
# ============================================================
def auto_detect_pico():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.hwid is not None and "2E8A" in port.hwid:
            return port.device
    return None

# ============================================================
# CONFIGURATION
# ============================================================
SERIAL_PORT = auto_detect_pico()
BAUD_RATE = 115200
LOG_FILE = Path("tools/history_calibration.txt")

SENSORS = [0, 1, 2, 3]
EMA_ALPHA = 0.15
TAN_30 = 0.577350269

# CONFIGURATION PARALLÈLE : Origine (0,0,0) au centre du segment des BS
BS_POSES = {
    4:  {"pos": (0.0, 0.0, 0.0)},  # BS 4 à gauche (-50cm)
    10: {"pos": (1.0, 0.0, 0.0)}   # BS 10 à droite (+50cm)
}

def load_bs_coefficients(target_bs):
    coeffs = None
    if not LOG_FILE.exists(): return None
    with open(LOG_FILE, "r") as f:
        for line in f:
            if "DATE_TIME" in line or line.startswith("-") or not line.strip(): continue
            parts = [p.strip() for p in line.split("|")]
            if len(parts) >= 7:
                try:
                    if int(parts[1]) == target_bs:
                        coeffs = (float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6]))
                except ValueError: continue
    return coeffs

# ============================================================
# GÉOMÉTRIE : CONSTRUCTION DU RAYON VECTORIEL UNITAIRE
# ============================================================
def build_ray_spherical(az_deg, el_deg):
    """ Crée un vecteur unitaire 3D parfait (Norme = 1.0) """
    az_rad = math.radians(az_deg)
    el_rad = math.radians(el_deg)
    
    # Coordonnées sphériques pures pour éviter les distorsions planes
    vx = math.sin(az_rad) * math.cos(el_rad)
    vy = math.cos(az_rad) * math.cos(el_rad)
    vz = math.sin(el_rad)
    return vx, vy, vz

# ============================================================
# SOLVEUR DE CROSSING BEAMS OPTIMISÉ
# ============================================================
def crossing_beams(az4, el4, az10, el10):
    # 1. Obtenir les rayons unitaires (a=1 et c=1 intrinsèquement)
    d1 = build_ray_spherical(az4, el4)
    d2 = build_ray_spherical(az10, el10)

    o1 = BS_POSES[4]["pos"]
    o2 = BS_POSES[10]["pos"]

    # Vector w0 = O1 - O2
    w0x = o1[0] - o2[0]
    w0y = o1[1] - o2[1]
    w0z = o1[2] - o2[2]

    # Produits scalaires (a=1 et c=1 car les vecteurs sont sphériques unitaires)
    b = d1[0]*d2[0] + d1[1]*d2[1] + d1[2]*d2[2]
    d = d1[0]*w0x + d1[1]*w0y + d1[2]*w0z
    e = d2[0]*w0x + d2[1]*w0y + d2[2]*w0z

    denom = 1.0 - b*b
    if abs(denom) < 1e-6:
        return None

    # Calcul des distances le long des rayons
    t1 = (b*e - d) / denom
    t2 = (e - b*d) / denom

    # Rejet des intersections inversées (derrière les caméras)
    if t1 < 0 or t2 < 0:
        return None

    # Points les plus proches sur chaque rayon
    c1x, c1y, c1z = o1[0] + t1*d1[0], o1[1] + t1*d1[1], o1[2] + t1*d1[2]
    c2x, c2y, c2z = o2[0] + t2*d2[0], o2[1] + t2*d2[1], o2[2] + t2*d2[2]

    # Calcul du milieu (Barycentre des deux points)
    X = (c1x + c2x) * 0.5
    Y = (c1y + c2y) * 0.5
    Z = (c1z + c2z) * 0.5

    # Indicateur de qualité : Erreur de croisement réelle en 3D
    err = math.sqrt((c1x - c2x)**2 + (c1y - c2y)**2 + (c1z - c2z)**2)
    return X, Y, Z, err

# ============================================================
# MAIN LOOP
# ============================================================
def main():
    print("\n===================================================")
    print(" LBEES - PARALLEL CROSSING BEAMS FIX")
    print("===================================================\n")

    coeffs_bs4 = load_bs_coefficients(4)
    coeffs_bs10 = load_bs_coefficients(10)
    
    if not coeffs_bs4 or not coeffs_bs10:
        print("[ERROR] Calibration coefficients missing in tools/history_calibration.txt")
        return

    if SERIAL_PORT is None:
        print("[ERROR] Pico not found.")
        return

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[SERIAL ERROR] {e}")
        return

    state = {s: {4:  {"angle_0": None, "angle_1": None, "ema_az": None, "ema_el": None, "last": 0},
                 10: {"angle_0": None, "angle_1": None, "ema_az": None, "ema_el": None, "last": 0}} for s in SENSORS}

    last_ui = time.time()

    while True:
        try:
            while ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("LH2,"): continue

                parts = line.split(",")
                if len(parts) != 6: continue

                try:
                    s_id = int(parts[1])
                    sweep = int(parts[2])
                    poly = int(parts[4])
                    lfsr = int(parts[5])
                except ValueError: continue

                if s_id not in SENSORS: continue
                bs_id = 4 if poly in (8, 9) else (10 if poly in (20, 21) else None)
                if bs_id is None: continue

                coeffs = coeffs_bs4 if bs_id == 4 else coeffs_bs10

                # Restauration de la calibration polynomiale indispensable
                if sweep == 0:
                    state[s_id][bs_id]["angle_0"] = (coeffs[0] * lfsr) + coeffs[1]
                elif sweep == 1:
                    state[s_id][bs_id]["angle_1"] = (coeffs[2] * lfsr) + coeffs[3]

                a0 = state[s_id][bs_id]["angle_0"]
                a1 = state[s_id][bs_id]["angle_1"]

                if a0 is not None and a1 is not None:
                    raw_az = (a0 + a1) * 0.5
                    diff = a0 - a1
                    swap_constant = 2.0 * (coeffs[1] - coeffs[3])
                    
                    if abs(diff) > 90.0: 
                        diff = swap_constant - diff
                    
                    # Restauration du calcul d'élévation optique exact
                    diff_rad = math.radians(diff / 2.0)
                    azimut_rad = math.radians(raw_az)
                    y_projection = math.tan(diff_rad) / TAN_30 * (1.0 / math.cos(azimut_rad))
                    raw_el = math.degrees(math.atan(y_projection))

                    # Application du filtre EMA
                    if state[s_id][bs_id]["ema_az"] is None:
                        state[s_id][bs_id]["ema_az"] = raw_az
                        state[s_id][bs_id]["ema_el"] = raw_el
                    else:
                        state[s_id][bs_id]["ema_az"] = EMA_ALPHA * raw_az + (1.0 - EMA_ALPHA) * state[s_id][bs_id]["ema_az"]
                        state[s_id][bs_id]["ema_el"] = EMA_ALPHA * raw_el + (1.0 - EMA_ALPHA) * state[s_id][bs_id]["ema_el"]

                    state[s_id][bs_id]["last"] = time.time()
                    state[s_id][bs_id]["angle_0"] = None
                    state[s_id][bs_id]["angle_1"] = None

            # Affichage et calcul des coordonnées
            now = time.time()
            if now - last_ui > 0.05:
                valid = []
                for s in SENSORS:
                    if (now - state[s][4]["last"] < 0.5 and now - state[s][10]["last"] < 0.5):
                        az4, el4 = state[s][4]["ema_az"], state[s][4]["ema_el"]
                        az10, el10 = state[s][10]["ema_az"], state[s][10]["ema_el"]

                        result = crossing_beams(az4, el4, az10, el10)
                        if result is not None:
                            x, y, z, err = result
                            valid.append((x, y, z, err))

                if len(valid) > 0:
                    cx = sum(v[0] for v in valid) / len(valid)
                    cy = sum(v[1] for v in valid) / len(valid)
                    cz = sum(v[2] for v in valid) / len(valid)
                    avg_err = sum(v[3] for v in valid) / len(valid)

                    txt = f"\rX:{cx:+06.3f}  Y:{cy:+06.3f}  Z:{cz:+06.3f}  ERR:{avg_err*100:05.1f} cm  [{len(valid)}/4]"
                    sys.stdout.write(txt + " \033[K")
                    sys.stdout.flush()
                else:
                    sys.stdout.write("\rWaiting lighthouse... \033[K")
                    sys.stdout.flush()
                last_ui = now

        except KeyboardInterrupt:
            print("\nStopped.")
            break
        except Exception: continue

if __name__ == "__main__":
    main()
