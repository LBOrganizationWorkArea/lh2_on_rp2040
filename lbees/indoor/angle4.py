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
# To be changed on another computer
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TARGET_SENSOR = 2
TARGET_BASE   = 4
TARGET_POLYS  = (8, 9)
LOG_FILE = Path("/home/vbianchi029/lbees/indoor/calibration_history.txt")

def load_latest_coefficients(target_base, target_sensor):
    if not LOG_FILE.exists():
        print(f"[ERREUR] File {LOG_FILE.name} Not found. Try calibrating first and make sure the calibration history file is in your files.")
        return None
        
    latest_coeffs = None
    
    with open(LOG_FILE, "r") as f:
        lines = f.readlines()
        for line in lines:
            if "DATE_TIME" in line or line.startswith("-") or line.strip() == "":
                continue
            
            parts = [p.strip() for p in line.split("|")]
            if len(parts) >= 7:
                try:
                    b_id = int(parts[1])
                    s_id = int(parts[2])
                    
                    if b_id == target_base and s_id == target_sensor:
                        latest_coeffs = (float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6]))
                except ValueError:
                    continue
                    
    return latest_coeffs

def run_monitor():
    # --- CHARGEMENT AUTOMATIQUE ---
    print("Loading the coefficients from the history")
    coeffs = load_latest_coefficients(TARGET_BASE, TARGET_SENSOR)
    
    if coeffs is None:
        print(f"[ERROR] No coefficients found for {TARGET_BASE} base and {TARGET_SENSOR} sensor.")
        return
        
    A0, B0, A1, B1 = coeffs
    print(f"[SUCCES] Coefficients loaded !")
    print(f"-> Sweep 0 : A0={A0:.8f}, B0={B0:.4f}")
    print(f"-> Sweep 1 : A1={A1:.8f}, B1={B1:.4f}\n")

    # --- DÉMARRAGE DU CAPTEUR ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        ser.reset_input_buffer()
        print(f"--- Connection made on {SERIAL_PORT} ---")
        print("Ctrl+C to stop.\n")
    except Exception as e:
        print(f"Connection error : {e}")
        return

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

            if s_id != TARGET_SENSOR or b_id != TARGET_BASE or poly not in TARGET_POLYS:
                continue

            if sweep == 0:
                angle_deg_0 = (A0 * lfsr) + B0
                last_sweep0[poly] = angle_deg_0

            elif sweep == 1 and poly in last_sweep0:
                angle_deg_1 = (A1 * lfsr) + B1
                angle_reel = (last_sweep0[poly] + angle_deg_1) / 2
                
                print(f"\rPoly={poly:2d} | Angle 0: {last_sweep0[poly]:+5.1f}° | Angle 1: {angle_deg_1:+5.1f}° | ---> Real Angle : {angle_reel:+6.2f}°   ", end="", flush=True)
                del last_sweep0[poly]

        except KeyboardInterrupt:
            print("\nStop.")
            break
        except Exception:
            continue

if __name__ == "__main__":
    run_monitor()
