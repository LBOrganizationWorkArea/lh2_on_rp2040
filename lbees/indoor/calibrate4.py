import serial
import statistics
from datetime import datetime
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
SERIAL_PORT = "/dev/ttyACM0" # port on linux, to be changed if on another OS
BAUD_RATE = 115200
TARGET_SENSOR = 2 # sensor id, to be changed if another sensor is used   
TARGET_BASE = 4 # same
WORK_DIR = Path("/home/vbianchi029/lbees/indoor") # needs to be changed for your own path
LOG_FILE = WORK_DIR / "calibration_history.txt" # same

"""
Description of the setup :
This calibration works with 3 points. 1° precision can be achieved with such calibration.
The Base Station is placed in a fixed position, it shall never move. Then, 3 markers should be placed on the ground.
One in front of the BS at 1m distance, one at 57.7 cm at the left from the first point, and the last one at 57.7 cm at the right from the first point. This 57.7 cm distance comes from basic trogonometry to achieve 3 angles to sample : +/- 30° and 0°
"""
CALIBRATION_ANGLES = [-30.0, 0.0, 30.0]
SAMPLES_PER_POINT = 50

def collect_samples(ser, target_angle):
    print(f"\n[POINT {target_angle}°] Place the sensor at {target_angle}°...")
    input(">>> Press ENTER to start sampling ...")
    samples_0 = []
    samples_1 = []

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
    print(f"  → median LFSR Sweep 0 : {med_0:.1f}")
    print(f"  → median LFSR Sweep 1 : {med_1:.1f}")
    return med_0, med_1

def calculate_coefficients(x_values, y_values):
    x_m = statistics.mean(x_values)
    y_m = statistics.mean(y_values)
    a = sum((xi - x_m) * (yi - y_m) for xi, yi in zip(x_values, y_values)) / sum((xi - x_m)**2 for xi in x_values)
    b = y_m - a * x_m
    return a, b

def save_to_log(base_id, sensor_id, a0, b0, a1, b1):
    file_exists = LOG_FILE.exists()
    
    with open(LOG_FILE, "a") as f:
        # # Create header if file is new
        if not file_exists:
            f.write("     DATE_TIME      | B | S |     A0     |    B0     |     A1     |    B1    |\n")
            f.write("-" * 80 + "\n")

        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"{now} | {base_id} | {sensor_id} | {a0:.8f} | {b0:.4f} | {a1:.8f} | {b1:.4f}\n"
        f.write(line)
        
    print(f"\n[SUCCESS] The 4 coefficients have been loaded into {LOG_FILE.name}")

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        print(f"Connection made on {SERIAL_PORT}")
        print(f"Target sensor : sensor_id={TARGET_SENSOR} | base={TARGET_BASE}\n")
    except Exception as e:
        print(f"Error : {e}")
        return

    lfsr_0_results = []
    lfsr_1_results = []
    
    for angle in CALIBRATION_ANGLES:
        med_0, med_1 = collect_samples(ser, angle)
        lfsr_0_results.append(med_0)
        lfsr_1_results.append(med_1)

    a0, b0 = calculate_coefficients(lfsr_0_results, CALIBRATION_ANGLES)
    a1, b1 = calculate_coefficients(lfsr_1_results, CALIBRATION_ANGLES)

    print(f"\n{'='*40}")
    print("--- RESULTS SWEEP 0 ---")
    print(f"A0 = {a0:.8f} | B0 = {b0:.4f}")
    print("--- RESULTS SWEEP 1 ---")
    print(f"A1 = {a1:.8f} | B1 = {b1:.4f}")
    print(f"{'='*40}")

    # --- AUTO SAVE ---
    save_to_log(TARGET_BASE, TARGET_SENSOR, a0, b0, a1, b1)

if __name__ == "__main__":
    main()
