import serial
import math
from pathlib import Path

"""
  _     _    _ __  __ _____ _   _  ____  _    _  _____   ____  ______ ______  _____ 
 | |   | |  | |  \/  |_   _| \ | |/ __ \| |  | |/ ____| |  _ \|  ____|  ____|/ ____|
 | |   | |  | | \  / | | | |  \| | |  | | |  | | (___   | |_) | |__  | |__  | (___  
 | |   | |  | | |\/| | | | | . ` | |  | | |  | |\___ \  |  _ <|  __| |  __|  \___ \ 
 | |___| |__| | |  | |_| |_| |\  | |__| | |__| |____) | | |_) | |____| |____ ____) |
 |______\____/|_|  |_|_____|_| \_|\____/ \____/|_____/  |____/|______|______|_____/ 

 Giorgio Rinolfi
 Victor Bianchi
 Antoine el Kahi
 Eduardo Gonzalez
"""

# --- CONFIGURATION ---
SERIAL_PORT   = "/dev/ttyACM0"
BAUD_RATE     = 115200
TARGET_SENSOR = 2
LOG_FILE      = Path("/home/vbianchi029/lbees/indoor/history_calibration.txt")
Y_OFFSET 	  = 0.2

# Géométrie du setup
# BS1 (base=4)  en (0, 0) — gauche
# BS2 (base=10) en (D, 0) — droite
D = 1.0  # distance entre les 2 BS en mètres
""" en étant derrière les BS et en regardant vers le capteur : BS n°5 (10 dans le code) à droite, BS n°11 (4 dans le code) à gauche. 1m de distance entre les 2"""

BASESTATIONS = {
    4:  (8,  9),   # BS1 — gauche
    10: (20, 21),  # BS2 — droite
}

def load_coefficients():
    if not LOG_FILE.exists():
        print(f"[ERREUR] Fichier introuvable : {LOG_FILE}")
        return None

    coeffs = {}
    with open(LOG_FILE, "r") as f:
        for line in f:
            if "DATE_TIME" in line or line.startswith("-") or not line.strip():
                continue
            parts = [p.strip() for p in line.split("|")]
            if len(parts) >= 7:
                try:
                    b_id = int(parts[1])
                    s_id = int(parts[2])
                    if s_id == TARGET_SENSOR and b_id in BASESTATIONS:
                        coeffs[b_id] = (float(parts[3]), float(parts[4]),
                                        float(parts[5]), float(parts[6]))
                except ValueError:
                    continue
    return coeffs if coeffs else None

def triangulate(alpha_deg, beta_deg, d):
    """
    Calcule la position X, Y du capteur par intersection de droites.
    Convention : angle positif = capteur à gauche (vers les X négatifs)
    """
    # 1. Conversion en radians (sans inverser de signes arbitrairement)
    a_rad = math.radians(alpha_deg)
    b_rad = math.radians(beta_deg)

    tan_a = math.tan(a_rad)
    tan_b = math.tan(b_rad)

    # 2. Le bon dénominateur, robuste sur toute la zone
    denom = tan_b - tan_a
    
    # 3. Sécurité : si les lasers sont parfaitement parallèles
    if abs(denom) < 1e-6:
        return None, None

    # 4. Calcul de Y (profondeur) puis de X (latéral)
    Y = d / denom
    X = -Y * tan_a

    return X, Y - Y_OFFSET

def run_monitor():
    print("\n" + "="*60)
    print("   LBEES - TRIANGULATION 2D EN TEMPS RÉEL")
    print("="*60)
    print(f"\n   Setup : BS1(base=4) ←——{D}m——→ BS2(base=10)")
    print(f"   BS1 en (0, 0) | BS2 en ({D}, 0)\n")

    coeffs = load_coefficients()
    if coeffs is None:
        print("[ERREUR] Aucun coefficient trouvé.")
        return

    missing = [b for b in BASESTATIONS if b not in coeffs]
    if missing:
        print(f"[ERREUR] Coefficients manquants pour BS : {missing}")
        return

    for b_id, (A0, B0, A1, B1) in coeffs.items():
        print(f"   BS{b_id} → A0={A0:.8f} | B0={B0:.4f} | A1={A1:.8f} | B1={B1:.4f}")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        ser.reset_input_buffer()
        print(f"\n[*] Connecté sur {SERIAL_PORT}")
        print("[*] En attente des faisceaux... (Ctrl+C pour quitter)\n")
    except Exception as e:
        print(f"[ERREUR] {e}")
        return

    last_sweep0  = {b_id: None for b_id in BASESTATIONS}
    last_angles  = {b_id: None for b_id in BASESTATIONS}

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if not line.startswith("LH2,"):
                continue

            parts = line.split(",")
            if len(parts) != 6:
                continue

            try:
                s_id  = int(parts[1])
                sweep = int(parts[2])
                b_id  = int(parts[3])
                poly  = int(parts[4])
                lfsr  = int(parts[5])
            except ValueError:
                continue

            if s_id != TARGET_SENSOR:
                continue
            if b_id not in BASESTATIONS:
                continue
            if poly not in BASESTATIONS[b_id]:
                continue

            A0, B0, A1, B1 = coeffs[b_id]

            if sweep == 0:
                last_sweep0[b_id] = (A0 * lfsr) + B0

            elif sweep == 1 and last_sweep0[b_id] is not None:
                angle_1 = (A1 * lfsr) + B1
                angle_reel = (last_sweep0[b_id] + angle_1) / 2
                last_angles[b_id] = angle_reel
                last_sweep0[b_id] = None

                # Calcul de position dès qu'on a les 2 angles
                if all(v is not None for v in last_angles.values()):
                    alpha = last_angles[4]   # BS1
                    beta  = last_angles[10]  # BS2

                    X, Y = triangulate(alpha, beta, D)

                    if X is not None:
                        print(f"\r[BS1]{alpha:+6.1f}° [BS2]{beta:+6.1f}° → X={X:+6.3f}m  Y={Y:6.3f}m      ", end="", flush=True)
                    else:
                        print(f"\r[BS1]{alpha:+6.1f}° [BS2]{beta:+6.1f}° → Position incalculable (angles parallèles)      ", end="", flush=True)

        except KeyboardInterrupt:
            print("\n\nArrêt. Fermeture propre.")
            ser.close()
            break
        except Exception:
            continue

if __name__ == "__main__":
    run_monitor()
