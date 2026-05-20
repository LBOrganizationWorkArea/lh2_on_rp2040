import serial

PORT = "COM3"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Reading from {PORT}... Press Ctrl+C to stop.")

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print(line)