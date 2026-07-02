import serial
import math
import time
from pathlib import Path

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
LOG_FILE = Path("~/lbees/2indoor_ubuntu/history_calibration.txt")
DATA_LOG_FILE = Path("tools/flight_recording.txt")

# Constantes géométriques
TAN_30 = 0.577350269
D_BS = 2.26  # Distance entre la BS 4 (X=0) et la BS 10 (X=2.26) en mètres
SENSORS = [0, 1, 2, 3]

# Filtre EMA (Barycentre temporel pour la stabilité brute)
EMA_ALPHA = 0.2 

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

def compute_3d_position(az4_deg, el4_deg, az10_deg, el10_deg):
    """ Moteur 2.5D : Calcule (X, Y, Z) à partir des angles lissés """
    # 1. PLAN X/Y
    a_rad = math.radians(az4_deg)
    b_rad = math.radians(az10_deg)
    
    tan_a = math.tan(a_rad)
    tan_b = math.tan(b_rad)
    denom = tan_b - tan_a
    
    if abs(denom) < 1e-6:
        return None, None, None
        
    Y = D_BS / denom
    X = -Y * tan_a
    
    # 2. EXTRUSION Z
    el4_rad = math.radians(el4_deg)
    el10_rad = math.radians(el10_deg)
    
    # Hauteur vue par BS 4
    dist_xy4 = math.sqrt(X**2 + Y**2)
    z4 = dist_xy4 * math.tan(el4_rad)
    
    # Hauteur vue par BS 10
    dist_xy10 = math.sqrt((X - D_BS)**2 + Y**2)
    z10 = dist_xy10 * math.tan(el10_rad)
    
    # 3. MOYENNE DES HAUTEURS -- TODO est-il pertinent de faire la moyenne des 2 sachant que l'un des 2 peut-être décalé de 5-7° ?
    Z = -(z4 + z10) / 2.0 # signe moins pour orienter l'axe dans la bonne direction
    
    return X, Y, Z

def main():
    print("\n" + "="*95)
    print("   LBEES - MULTITRACKING 3D : BARYCENTRE SPATIAL + DIAGNOSTIC ÉLÉVATION")
    print("="*95)
    
    coeffs_bs4 = load_bs_coefficients(4)
    coeffs_bs10 = load_bs_coefficients(10)
    
    if not coeffs_bs4 or not coeffs_bs10:
        print("[ERREUR] Coefficients introuvables dans l'historique.")
        return
        
    print(f"\n[OK] Coefficients chargés. Force du filtre EMA : {EMA_ALPHA}")
    print("En attente des lasers... (Ctrl+C pour quitter)\n")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[ERREUR SÉRIE] {e}")
        return
    
    # Structure d'état : Stocke les angles, le temps de dernière mise à jour, et la pos 3D
    state = {
        s: {
            4:  {"angle_0": None, "angle_1": None, "ema_az": None, "ema_el": None, "last_update": 0},
            10: {"angle_0": None, "angle_1": None, "ema_az": None, "ema_el": None, "last_update": 0},
            "pos_3d": None
        } for s in SENSORS
    }

    last_ui_update = time.time()
    
    # Prépare 8 lignes vides dans le terminal
    print("\n\n\n\n\n\n\n\n")

    # Ouverture du fichier de log en mode écriture
    with open(DATA_LOG_FILE, "w") as f_log:
        f_log.write("timestamp,x,y,z\n")

        # BOUCLE PRINCIPALE
        while True:
            try:
                # 1. PARSING DU PORT SÉRIE ET MATHS DE BASE
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line.startswith("LH2,"): continue
                    
                    parts = line.split(",")
                    if len(parts) != 6: continue
                    
                    s_id = int(parts[1])
                    sweep = int(parts[2])
                    poly = int(parts[4])
                    lfsr = int(parts[5])

                    if s_id not in SENSORS: continue
                    
                    bs_id = 4 if poly in (8, 9) else (10 if poly in (20, 21) else None)
                    if not bs_id: continue

                    coeffs = coeffs_bs4 if bs_id == 4 else coeffs_bs10

                    if sweep == 0:
                        state[s_id][bs_id]["angle_0"] = (coeffs[0] * lfsr) + coeffs[1]
                    elif sweep == 1:
                        state[s_id][bs_id]["angle_1"] = (coeffs[2] * lfsr) + coeffs[3]

                    # Dès qu'on a un "V" complet (sweep 0 + 1)
                    if state[s_id][bs_id]["angle_0"] is not None and state[s_id][bs_id]["angle_1"] is not None:
                        
                        a0 = state[s_id][bs_id]["angle_0"]
                        a1 = state[s_id][bs_id]["angle_1"]
                        
                        azimut_brut = (a0 + a1) / 2.0
                        diff = a0 - a1
                        swap_constant = 2.0 * (coeffs[1] - coeffs[3])
                        
                        if abs(diff) > 90.0:
                            diff = swap_constant - diff
                        
                        diff_rad = math.radians(diff / 2.0)
                        azimut_rad = math.radians(azimut_brut)
                        y_projection = math.tan(diff_rad) / TAN_30 * (1.0 / math.cos(azimut_rad))
                        elevation_brute = math.degrees(math.atan(y_projection))
                        
                        # Application du filtre EMA
                        if state[s_id][bs_id]["ema_az"] is None:
                            state[s_id][bs_id]["ema_az"] = azimut_brut
                            state[s_id][bs_id]["ema_el"] = elevation_brute
                        else:
                            state[s_id][bs_id]["ema_az"] = EMA_ALPHA * azimut_brut + (1.0 - EMA_ALPHA) * state[s_id][bs_id]["ema_az"]
                            state[s_id][bs_id]["ema_el"] = EMA_ALPHA * elevation_brute + (1.0 - EMA_ALPHA) * state[s_id][bs_id]["ema_el"]
                        
                        state[s_id][bs_id]["last_update"] = time.time()
                        state[s_id][bs_id]["angle_0"] = None
                        state[s_id][bs_id]["angle_1"] = None

                # 2. RAFRAÎCHISSEMENT DE L'INTERFACE ET CALCUL DE LA 3D (10 FPS)
                now = time.time()
                if now - last_ui_update > 0.1:
                    
                    active_sensors = []
                    bloc_affichage = []
                    
                    # A. Calcul de la 3D pour chaque capteur
                    for s in SENSORS:
                        # Si les données datent de moins de 0.5s pour les DEUX bases
                        if (now - state[s][4]["last_update"] < 0.5) and (now - state[s][10]["last_update"] < 0.5):
                            az4, el4 = state[s][4]["ema_az"], state[s][4]["ema_el"]
                            az10, el10 = state[s][10]["ema_az"], state[s][10]["ema_el"]
                            
                            x, y, z = compute_3d_position(az4, el4, az10, el10)
                            
                            if x is not None:
                                state[s]["pos_3d"] = (x, y, z)
                                active_sensors.append(s)
                                bloc_affichage.append(f"   [Capteur {s}] 🟢 | X: {x:+06.3f} m | Y: {y:+06.3f} m | Z: {z:+06.3f} m  ||  El4: {el4:+06.2f}° | El10: {el10:+06.2f}°")
                                continue
                                
                        # Si on arrive ici, le capteur est inactif ou n'a pas les deux bases
                        state[s]["pos_3d"] = None
                        bloc_affichage.append(f"   [Capteur {s}] 🔴 |      Hors champ (Attente laser...)       ||  El4:  ...    | El10:  ...   ")
                    
                    # B. Ligne de séparation
                    bloc_affichage.append("   " + "-" * 90)
                    
                    # C. Calcul du Barycentre (Moyenne des capteurs actifs)
                    if len(active_sensors) > 0:
                        cx = sum(state[s]["pos_3d"][0] for s in active_sensors) / len(active_sensors)
                        cy = sum(state[s]["pos_3d"][1] for s in active_sensors) / len(active_sensors)
                        cz = sum(state[s]["pos_3d"][2] for s in active_sensors) / len(active_sensors)

                        # --- ENREGISTREMENT ---
                        timestamp = time.time()
                        f_log.write(f"{timestamp},{cx:.4f},{cy:.4f},{cz:.4f}\n")
                        f_log.flush() # Sauvegarde immédiate
                        
                        cel4 = sum(state[s][4]["ema_el"] for s in active_sensors) / len(active_sensors)
                        cel10 = sum(state[s][10]["ema_el"] for s in active_sensors) / len(active_sensors)
                        
                        bloc_affichage.append(f"   🎯 BARYCENTRE  | X: {cx:+06.3f} m | Y: {cy:+06.3f} m | Z: {cz:+06.3f} m  ({len(active_sensors)}/4 capt.)")
                        bloc_affichage.append(f"   🔍 DIAGNOSTIC  | Moyenne Élévation -> BS 4 : {cel4:+06.2f}°  |  BS 10 : {cel10:+06.2f}°")
                    else:
                        bloc_affichage.append("   🎯 BARYCENTRE  |      Attente de données valides...                                  ")
                        bloc_affichage.append("   🔍 DIAGNOSTIC  |      Attente de données valides...                                  ")
                    
                    # D. Impression propre sans scintillement
                    lignes_a_monter = len(bloc_affichage)
                    print(f"\033[{lignes_a_monter}A", end="") # On monte du nombre exact de lignes
                    print("\n".join(ligne + "\033[K" for ligne in bloc_affichage)) # \033[K nettoie la fin de la ligne
                    
                    last_ui_update = now
                    
                time.sleep(0.005)

            except KeyboardInterrupt:
                print("\n" * 8 + "\n[OK] Arrêt du tracking.\033[K") 
                break
            except Exception as e:
                continue

if __name__ == "__main__":
    main()
