This branch contains a repository that holds 3 main files: `calibrate4.py`, `angle4.py`, and `calibration_history.txt`.
`calibrate3.py` and `angle3.py` are previous versions, and `sensor_raw.py` just logs (prints) the raw data of one sensor. This is the data that we are going to process in order to compute the real angle between the Base Station and the sensor.

The purpose of computing this angle is to ultimately determine the exact 2D position (X, Y) of the sensor. By adding a second Base Station, we will obtain a second angle, allowing us to triangulate the exact position using trigonometry.

In order to get this first angle, we need to calibrate the system. The calibration process is handled by `calibrate4.py`. It requires placing the sensor at 3 specific known angles (-30°, 0°, and +30°) at a fixed 1-meter distance from the Base Station. The script independently captures the two laser sweeps (Sweep 0 and Sweep 1) and calculates 4 specific mathematical coefficients (A0, B0, A1, B1). 

These coefficients are automatically saved into `calibration_history.txt`. Finally, `angle4.py` reads this history file to output a highly precise angle in real-time, merging the two sweeps into one perfect center line.
