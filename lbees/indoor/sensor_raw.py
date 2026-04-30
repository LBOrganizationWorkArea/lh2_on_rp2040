import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TARGET_SENSOR = 2

def run_monitor():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"--- Connexion établie sur {SERIAL_PORT} ---")
        print(f"--- Filtrage sur le SENSOR {TARGET_SENSOR} ---")
        print("Appuyez sur Ctrl+C pour arrêter.\n")
    except Exception as e:
        print(f"Erreur de connexion : {e}")
        print("Vérifiez le branchement et faites : sudo chmod a+rw /dev/ttyACM0")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line.startswith("LH2,"):
                parts = line.split(",")
                
                if len(parts) == 6:
                    sensor_id = int(parts[1])
                    
                    # On affiche TOUT pour vérifier (on a enlevé le if sensor_id)
                    base_id = parts[3]
                    poly = parts[4]
                    lfsr = parts[5]
                    print("Données brutes du capteur")
                    print(f"{{sensor_id = {sensor_id}}} || {{base = {base_id}}} || {{poly = {poly}}} || {{lfsr = {lfsr}}}")
        
        except KeyboardInterrupt:
            print("\nArrêt du moniteur.")
            break
        except Exception:
            continue

if __name__ == "__main__":
    run_monitor()