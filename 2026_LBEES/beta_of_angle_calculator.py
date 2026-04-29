import serial
import math

# --- SETTINGS ---
SERIAL_PORT = '/dev/ttyACM0'  # Change to 'COM3' for Windows
BAUD_RATE = 115200
BASE_DISTANCE = 0.20          # 20cm between LH 5 and LH 11

ID_A = 5
ID_B = 11

# Conversion constant (Adjust if the distance on screen doesn't match reality)
TICKS_PER_REV = 833333 

# Stores the last known angles
angles = {ID_A: None, ID_B: None}

def to_deg(ts):
    """Convert raw timestamp to degrees (-60 to 60)"""
    return (((ts % TICKS_PER_REV) / TICKS_PER_REV) * 120) - 60

# --- MAIN ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Tracking sen_0 using LH {ID_A} and {ID_B}...")
except:
    print("Error: Check your Serial Port!")
    exit()

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    
    if "sen_0" in line:
        try:
            # Extract data from sen_0 only
            # Format: sen_0 (5-12345 11-67890)
            clean_data = line.split('(')[1].split(')')[0].split()
            
            for item in clean_data:
                lh_id, ts = map(int, item.split('-'))
                if lh_id in angles:
                    angles[lh_id] = to_deg(ts)

            # Calculate X, Y only when both Lighthouses are seen
            if angles[ID_A] is not None and angles[ID_B] is not None:
                # Triangulation Math
                a = math.radians(90 - angles[ID_A])
                b = math.radians(90 + angles[ID_B])
                
                x = (BASE_DISTANCE * math.tan(b)) / (math.tan(b) - math.tan(a))
                y = x * math.tan(a)
                
                print(f"X: {x:.3f}m | Y: {y:.3f}m")

        except:
            continue