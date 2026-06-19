import sys
import time
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from utils.angle_lib.angle_decoder import LH2Decoder
from utils.angle_lib.serial_read import LH2SerialReader
from utils.user_interface.saids_implementation.data_processing import (
    LH2_angles_to_pixels,
    solve_3d_scene,
)


# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
LOG_FILE = Path("~/lbees/2indoor_ubuntu/history_calibration.txt").expanduser()
DATA_LOG_FILE = Path("utils/user_interface/tools/flight_recording_said.txt")

# Geometry constants
SENSORS = [0, 1, 2, 3]
EMA_ALPHA = 0.2
MAX_HISTORY_SAMPLES = 24


def main():
    print("\n" + "=" * 95)
    print("   LBEES - MULTITRACKING 3D : SAID SERIAL READER + 3D SOLVER")
    print("=" * 95)

    print(f"\n[OK] Angle decoder et solveur 3D chargés. Force du filtre EMA : {EMA_ALPHA}")
    print("En attente des lasers... (Ctrl+C pour quitter)\n")

    try:
        reader = LH2SerialReader(port=SERIAL_PORT, baud=BAUD_RATE)
        decoder = LH2Decoder()
    except Exception as exc:
        print(f"[ERREUR SÉRIE] {exc}")
        return

    state = {
        sensor_id: {
            4: {
                "ema_az": None,
                "ema_el": None,
                "last_update": 0,
                "pixel": None,
            },
            10: {
                "ema_az": None,
                "ema_el": None,
                "last_update": 0,
                "pixel": None,
            },
            "pos_3d": None,
        }
        for sensor_id in SENSORS
    }

    last_ui_update = time.time()
    print("\n" * 8)

    history_samples = []

    DATA_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)

    with open(DATA_LOG_FILE, "w", encoding="utf-8") as log_handle:
        log_handle.write("timestamp,x,y,z\n")

        try:
            while True:
                try:
                    frames = reader.read_frames()
                    decoder.feed_all(frames)

                    for sensor_id in SENSORS:
                        for bs_id in (4, 10):
                            angles = decoder.get_angles(sensor_id, bs_id)
                            if angles is None:
                                continue

                            slot = state[sensor_id][bs_id]
                            slot["ema_az"] = angles["az"]
                            slot["ema_el"] = angles["el"]
                            slot["last_update"] = angles["last_update"]
                            slot["pixel"] = LH2_angles_to_pixels(
                                np.array([np.radians(angles["az"])]),
                                np.array([np.radians(angles["el"])]),
                            )[0]

                    now = time.time()
                    if now - last_ui_update > 0.1:
                        bloc_affichage = []
                        current_samples = []

                        for sensor_id in SENSORS:
                            fresh_4 = now - state[sensor_id][4]["last_update"] < 0.5
                            fresh_10 = now - state[sensor_id][10]["last_update"] < 0.5

                            if fresh_4 and fresh_10:
                                pixel_4 = state[sensor_id][4]["pixel"]
                                pixel_10 = state[sensor_id][10]["pixel"]
                                if pixel_4 is not None and pixel_10 is not None:
                                    current_samples.append((sensor_id, pixel_4, pixel_10))

                        history_samples.extend(current_samples)
                        if len(history_samples) > MAX_HISTORY_SAMPLES:
                            history_samples = history_samples[-MAX_HISTORY_SAMPLES:]

                        if len(history_samples) >= 8:
                            try:
                                pts_a = np.asarray([sample[1] for sample in history_samples], dtype=np.float32)
                                pts_b = np.asarray([sample[2] for sample in history_samples], dtype=np.float32)
                                sample_sensor_ids = [sample[0] for sample in history_samples]

                                point3d, t_star, r_star = solve_3d_scene(pts_a, pts_b)

                                sensor_points = {sensor_id: [] for sensor_id in SENSORS}
                                for index, sensor_id in enumerate(sample_sensor_ids):
                                    sensor_points[sensor_id].append(point3d[index])

                                for sensor_id in SENSORS:
                                    points_for_sensor = sensor_points[sensor_id]
                                    if points_for_sensor:
                                        state[sensor_id]["pos_3d"] = tuple(np.mean(points_for_sensor, axis=0))
                                    else:
                                        state[sensor_id]["pos_3d"] = None
                            except Exception:
                                pass

                        for sensor_id in SENSORS:
                            pos_3d = state[sensor_id]["pos_3d"]
                            if pos_3d is None:
                                bloc_affichage.append(
                                    f"   [Capteur {sensor_id}] 🔴 |      Hors champ (Attente solve 3D...)    ||  El4:  ...    | El10:  ...   "
                                )
                                continue

                            x_pos, y_pos, z_pos = pos_3d
                            bloc_affichage.append(
                                f"   [Capteur {sensor_id}] 🟢 | X: {x_pos:+06.3f} m | Y: {y_pos:+06.3f} m | Z: {z_pos:+06.3f} m  ||  Solve3D"
                            )

                        bloc_affichage.append("   " + "-" * 90)

                        active_sensors = [sensor_id for sensor_id in SENSORS if state[sensor_id]["pos_3d"] is not None]

                        if len(active_sensors) > 0:
                            cx = sum(state[s]["pos_3d"][0] for s in active_sensors) / len(active_sensors)
                            cy = sum(state[s]["pos_3d"][1] for s in active_sensors) / len(active_sensors)
                            cz = sum(state[s]["pos_3d"][2] for s in active_sensors) / len(active_sensors)

                            timestamp = time.time()
                            log_handle.write(f"{timestamp},{cx:.4f},{cy:.4f},{cz:.4f}\n")
                            log_handle.flush()

                            cel4 = sum(state[s][4]["ema_el"] for s in active_sensors) / len(active_sensors)
                            cel10 = sum(state[s][10]["ema_el"] for s in active_sensors) / len(active_sensors)

                            bloc_affichage.append(
                                f"   🎯 BARYCENTRE  | X: {cx:+06.3f} m | Y: {cy:+06.3f} m | Z: {cz:+06.3f} m  ({len(active_sensors)}/4 capt.)"
                            )
                            bloc_affichage.append(
                                f"   🔍 DIAGNOSTIC  | Moyenne Élévation -> BS 4 : {cel4:+06.2f}°  |  BS 10 : {cel10:+06.2f}°"
                            )
                        else:
                            bloc_affichage.append(
                                "   🎯 BARYCENTRE  |      Attente de données valides...                                  "
                            )
                            bloc_affichage.append(
                                "   🔍 DIAGNOSTIC  |      Attente de données valides...                                  "
                            )

                        lignes_a_monter = len(bloc_affichage)
                        print(f"\033[{lignes_a_monter}A", end="")
                        print("\n".join(ligne + "\033[K" for ligne in bloc_affichage))

                        last_ui_update = now

                    time.sleep(0.005)

                except KeyboardInterrupt:
                    print("\n" * 8 + "\n[OK] Arrêt du tracking.\033[K")
                    break
                except Exception:
                    continue
        finally:
            reader.close()


if __name__ == "__main__":
    main()