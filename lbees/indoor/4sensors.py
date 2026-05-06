import serial
import math
import time
from pathlib import Path

"""
  _     _    _ __  __ _____ _   _  ____  _    _  _____   ____  ______ ______  _____ 
 | |   | |  | |  \/  |_   _| \ | |/ __ \| |  | |/ ____| |  _ \|  ____|  ____|/ ____|
 | |   | |  | | \  / | | | |  \| | |  | | |  | | (___   | |_) | |__  | |__  | (___  
 | |   | |  | | |\/| | | | | . ` | |  | | |  | |\___ \  |  _ <|  __| |  __|  \___ \ 
 | |___| |__| | |  | |_| |_| |\  | |__| | |__| |____) | | |_) | |____| |____ ____) |
 |______\____/|_|  |_|_____|_| \_|\____/ \____/|_____/  |____/|______|______|_____/ 

        LBEES - MULTITRACKING 4 CAPTEURS (TEMPS RÉEL)
"""

# --- CONFIGURATION ---
SERIAL_PORT   = "/dev/ttyACM0"
BAUD_RATE     = 115200
# On charge les coefficients calibrés avec le capteur 2 (ils sont valables pour tous)
CALIB_SENSOR  = 2 
LOG_FILE      = Path("/home/vbianchi029/lbees/indoor/history_calibration.txt")

D = 1.0  # Distance entre BS1 et BS2
BASESTATIONS = {4: (8, 9), 10: (20, 21)}
SENSORS = [0, 1, 2, 3]

def load_coefficients():
    if not LOG_FILE.exists():
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
                    if s_id == CALIB_SENSOR and b_id in BASESTATIONS:
                        coeffs[b_id] = (float(parts[3]), float(parts[4]),
                                        float(parts[5]), float(parts[6]))
                except ValueError:
                    continue
    return coeffs if len(coeffs) == 2 else None

def triangulate(alpha_deg, beta_deg, d):
    # Trim de correction (au cas où tu as du pincement, tu peux l'ajuster ici)
    trim_correction = 0.0 
    a_rad = math.radians(alpha_deg - trim_correction)
    b_rad = math.radians(beta_deg + trim_correction)

    tan_a = math.tan(a_rad)
    tan_b = math.tan(b_rad)
    denom = tan_b - tan_a
    
    if abs(denom) < 1e-6:
        return None, None

    Y = d / denom
    X = -Y * tan_a
    return X, Y

def run_multimonitor():
    print("\n" + "="*60)
    print("   LBEES - MULTITRACKING 4 CAPTEURS (X, Y)")
    print("="*60 + "\n")

    coeffs = load_coefficients()
    if coeffs is None:
        print("[ERREUR] Coefficients introuvables. Vérifiez le fichier de calibration.")
        return

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[ERREUR] {e}")
        return

    # Mémoire 2D : [sensor_id][base_id]
    last_sweep0 = {s: {b: None for b in BASESTATIONS} for s in SENSORS}
    last_angles = {s: {b: None for b in BASESTATIONS} for s in SENSORS}
    
    # Stockage des positions finales pour l'affichage
    positions = {s: {"X": None, "Y": None, "last_update": 0} for s in SENSORS}
    last_ui_update = time.time()

    print("\n\n\n\n\n") # Prépare l'espace pour l'affichage des 4 lignes

    try:
        while True:
            # Vidange rapide du buffer pour rester en pur temps réel
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith("LH2,"): continue

                parts = line.split(",")
                if len(parts) != 6: continue

                try:
                    s_id  = int(parts[1])
                    sweep = int(parts[2])
                    b_id  = int(parts[3])
                    poly  = int(parts[4])
                    lfsr  = int(parts[5])
                except ValueError:
                    continue

                if s_id not in SENSORS or b_id not in BASESTATIONS or poly not in BASESTATIONS[b_id]:
                    continue

                A0, B0, A1, B1 = coeffs[b_id]

                if sweep == 0:
                    last_sweep0[s_id][b_id] = (A0 * lfsr) + B0

                elif sweep == 1 and last_sweep0[s_id][b_id] is not None:
                    angle_1 = (A1 * lfsr) + B1
                    last_angles[s_id][b_id] = (last_sweep0[s_id][b_id] + angle_1) / 2
                    last_sweep0[s_id][b_id] = None # Reset pour le prochain tour

                    # Si on a l'angle des deux BS pour CE capteur, on triangule !
                    if all(v is not None for v in last_angles[s_id].values()):
                        X, Y = triangulate(last_angles[s_id][4], last_angles[s_id][10], D)
                        if X is not None:
                            positions[s_id]["X"] = X
                            positions[s_id]["Y"] = Y
                            positions[s_id]["last_update"] = time.time()
                        
                        # Optionnel : réinitialiser les angles après calcul pour éviter les données fantômes
                        # last_angles[s_id] = {b: None for b in BASESTATIONS}

           # --- MISE À JOUR DE L'INTERFACE (10 fps) ---
            now = time.time()
            if now - last_ui_update > 0.1:
                # On remonte maintenant de 6 lignes (4 capteurs + ligne de séparation + le centre)
                print("\033[6A", end="") 
                
                active_x = []
                active_y = []
                
                for s_id in SENSORS:
                    pos = positions[s_id]
                    # Si la donnée date de moins d'une demi-seconde, on l'affiche et on la garde pour le centre
                    if pos["X"] is not None and (now - pos["last_update"] < 0.5):
                        print(f"   [Capteur {s_id}] 🟢 Tracking OK  | X = {pos['X']:+6.3f} m  |  Y = {pos['Y']:6.3f} m       ")
                        active_x.append(pos["X"])
                        active_y.append(pos["Y"])
                    else:
                        print(f"   [Capteur {s_id}] 🔴 Hors champ   | X = ------ m  |  Y = ------ m       ")
                
                print("-" * 60)
                
                # Calcul et affichage du Centre (Barycentre)
                if len(active_x) == 4:
                    center_x = sum(active_x) / 4
                    center_y = sum(active_y) / 4
                    print(f"   [ CENTRE ] 🎯 Barycentre   | X = {center_x:+6.3f} m  |  Y = {center_y:6.3f} m       ")
                else:
                    print(f"   [ CENTRE ] ⏳ Attente des 4 capteurs pour le calcul...                     ")
                    
                last_ui_update = now
                
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n[*] Arrêt du tracking. Fermeture du port série.")
        ser.close()

if __name__ == "__main__":
    run_multimonitor()
