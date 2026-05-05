import serial
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

# Base stations config : base_id → polynômes valides
BASESTATIONS = {
    4:  (8,  9),   # BS1
    10: (20, 21),  # BS2
}

def load_coefficients():
    """Charge les derniers coefficients pour chaque BS depuis l'historique."""
    if not LOG_FILE.exists():
        print(f"[ERREUR] Fichier introuvable : {LOG_FILE}")
        return None

    # On veut les derniers coeffs pour chaque base
    coeffs = {}  # base_id → (A0, B0, A1, B1)

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

def run_monitor():
    print("\n" + "="*60)
    print("   LBEES - DUAL BASESTATION ANGLE TRACKER")
    print("="*60)

    # --- Chargement des coefficients ---
    print(f"\n[*] Chargement des coefficients depuis {LOG_FILE.name}...")
    coeffs = load_coefficients()

    if coeffs is None:
        print("[ERREUR] Aucun coefficient trouvé. Lancez calibrate4.py d'abord.")
        return

    for b_id, (A0, B0, A1, B1) in coeffs.items():
        print(f"   BS{b_id} → A0={A0:.8f} | B0={B0:.4f} | A1={A1:.8f} | B1={B1:.4f}")

    # Vérifier que les 2 BS sont calibrées
    missing = [b for b in BASESTATIONS if b not in coeffs]
    if missing:
        print(f"\n[ATTENTION] BS manquantes dans l'historique : {missing}")
        print("-> Lancez calibrate4.py pour chaque BS manquante.")
        return

    # --- Connexion série ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        ser.reset_input_buffer()
        print(f"\n[*] Connecté sur {SERIAL_PORT}")
        print("[*] En attente des faisceaux... (Ctrl+C pour quitter)\n")
    except Exception as e:
        print(f"[ERREUR] Impossible d'ouvrir {SERIAL_PORT} : {e}")
        return

    # Mémoire du sweep=0 pour chaque BS
    last_sweep0 = {b_id: None for b_id in BASESTATIONS}

    # Derniers angles calculés pour chaque BS (pour affichage)
    last_angles = {b_id: None for b_id in BASESTATIONS}

    # --- Boucle principale ---
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

            # Filtres
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

                # Affichage dès qu'on a les 2 angles
                if all(v is not None for v in last_angles.values()):
                    a1 = last_angles[4]
                    a2 = last_angles[10]
                    print(f"\r[BS1] {a1:+6.2f}°  |  [BS2] {a2:+6.2f}°      ", end="", flush=True)

        except KeyboardInterrupt:
            print("\n\n[*] Arrêt. Fermeture propre.")
            ser.close()
            break
        except Exception:
            continue

if __name__ == "__main__":
    run_monitor()
