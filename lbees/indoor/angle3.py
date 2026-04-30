import serial

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TARGET_SENSOR = 2
TARGET_BASE   = 4
TARGET_POLYS  = (8, 9)

# --- TES 4 COEFFICIENTS DE CALIBRATION ---
# Remplace les zéros par tes vraies valeurs fraîchement calculées !

A0 = 0.00293771   # Coefficient A pour le Sweep 0
B0 = -224.1032       # Coefficient B pour le Sweep 0

A1 = 0.00301241   # Coefficient A pour le Sweep 1
B1 = -117.7621       # Coefficient B pour le Sweep 1

def run_monitor():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        ser.reset_input_buffer()
        print(f"--- Connexion établie sur {SERIAL_PORT} ---")
        print(f"--- Sensor={TARGET_SENSOR} | Base={TARGET_BASE} ---")
        print("Appuyez sur Ctrl+C pour arrêter.\n")
    except Exception as e:
        print(f"Erreur de connexion : {e}")
        return

    # Dictionnaire pour mémoriser le dernier passage du sweep 0
    last_sweep0 = {}  

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

            # FILTRE STRICT
            if s_id != TARGET_SENSOR or b_id != TARGET_BASE or poly not in TARGET_POLYS:
                continue

            # --- APPLICATION DES DEUX MATHÉMATIQUES ---
            if sweep == 0:
                # On utilise A0 et B0
                angle_deg_0 = (A0 * lfsr) + B0
                last_sweep0[poly] = angle_deg_0

            elif sweep == 1 and poly in last_sweep0:
                # On utilise A1 et B1
                angle_deg_1 = (A1 * lfsr) + B1
                
                # LA MOYENNE DÉFINITIVE (LE MILIEU DU V)
                angle_reel = (last_sweep0[poly] + angle_deg_1) / 2
                
                print(f"\rPoly={poly:2d} | Angle 0: {last_sweep0[poly]:+5.1f}° | Angle 1: {angle_deg_1:+5.1f}° | ---> Angle Réel : {angle_reel:+6.2f}°   ", end="", flush=True)
                
                # On nettoie la mémoire pour le prochain tour du rotor
                del last_sweep0[poly]

        except KeyboardInterrupt:
            print("\nArrêt du moniteur.")
            break
        except Exception:
            continue

if __name__ == "__main__":
    run_monitor()
