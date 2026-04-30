import serial
import statistics
from datetime import datetime
from pathlib import Path

""" version avec 2 modèles linéaires, un pour chaque laser du 'V' de la BS """

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TARGET_SENSOR = 2       
TARGET_BASE = 4         
WORK_DIR = Path("/home/vbianchi029/lbees/indoor")
LOG_FILE = WORK_DIR / "calibration_history_double.txt"

CALIBRATION_ANGLES = [-30.0, 0.0, 30.0]
SAMPLES_PER_POINT = 50

def collect_samples(ser, target_angle):
    print(f"\n[POINT {target_angle}°] Positionnez le capteur à {target_angle}°...")
    input(">>> Appuyez sur ENTRÉE pour capturer...")
    samples_0 = []
    samples_1 = []

    # On boucle tant qu'on n'a pas 50 échantillons POUR CHAQUE axe
    while len(samples_0) < SAMPLES_PER_POINT or len(samples_1) < SAMPLES_PER_POINT:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line.startswith("LH2,"):
            parts = line.split(",")
            if len(parts) == 6:
                try:
                    s_id  = int(parts[1])
                    sweep = int(parts[2])
                    b_id  = int(parts[3])
                    poly  = int(parts[4])   
                    lfsr  = int(parts[5])

                    if s_id == TARGET_SENSOR and b_id == TARGET_BASE and poly in (8, 9):
                        if sweep == 0 and len(samples_0) < SAMPLES_PER_POINT:
                            samples_0.append(lfsr)
                        elif sweep == 1 and len(samples_1) < SAMPLES_PER_POINT:
                            samples_1.append(lfsr)
                            
                        print(f"\rCapture -> Sweep 0: {len(samples_0)}/{SAMPLES_PER_POINT} | Sweep 1: {len(samples_1)}/{SAMPLES_PER_POINT}", end="")

                except ValueError:
                    pass

    print() 
    med_0 = statistics.median(samples_0)
    med_1 = statistics.median(samples_1)
    print(f"  → Médiane LFSR Sweep 0 : {med_0:.1f}")
    print(f"  → Médiane LFSR Sweep 1 : {med_1:.1f}")
    return med_0, med_1

def calculate_coefficients(x_values, y_values):
    x_m = statistics.mean(x_values)
    y_m = statistics.mean(y_values)
    a = sum((xi - x_m) * (yi - y_m) for xi, yi in zip(x_values, y_values)) / sum((xi - x_m)**2 for xi in x_values)
    b = y_m - a * x_m
    return a, b

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        print(f"Connexion établie sur {SERIAL_PORT}")
        print(f"Capteur cible : sensor_id={TARGET_SENSOR} | base={TARGET_BASE}\n")
    except Exception as e:
        print(f"Erreur : {e}")
        return

    lfsr_0_results = []
    lfsr_1_results = []
    
    for angle in CALIBRATION_ANGLES:
        med_0, med_1 = collect_samples(ser, angle)
        lfsr_0_results.append(med_0)
        lfsr_1_results.append(med_1)

    # Calcul des deux paires de coefficients
    a0, b0 = calculate_coefficients(lfsr_0_results, CALIBRATION_ANGLES)
    a1, b1 = calculate_coefficients(lfsr_1_results, CALIBRATION_ANGLES)

    print(f"\n{'='*40}")
    print("--- RESULTATS SWEEP 0 ---")
    print(f"A0 = {a0:.8f}")
    print(f"B0 = {b0:.4f}")
    print("\n--- RESULTATS SWEEP 1 ---")
    print(f"A1 = {a1:.8f}")
    print(f"B1 = {b1:.4f}")
    print(f"{'='*40}\n")

if __name__ == "__main__":
    main()
