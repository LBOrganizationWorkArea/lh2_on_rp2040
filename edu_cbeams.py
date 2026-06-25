import serial
import math
import sys
import re
import serial.tools.list_ports
import numpy as np

# ============================================================
# AUTO DETECT PICO
# ============================================================
def auto_detect_pico():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.hwid is not None and "2E8A" in port.hwid:
            return port.device
    return None


# ===========================================================================
# 1. Constantes, Calibration et Poses 3D
# ===========================================================================

SERIAL_PORT = auto_detect_pico()
BAUD_RATE = 115200
TAN_30 = math.tan(math.radians(30.0))

# Dictionnaire de calibration (extrait de main_real.c)
CALIBRATION = {
    0: {'A0': 0.00315641, 'B0': -121.7511, 'A1': 0.00307607, 'B1': -234.6501},
    1: {'A0': 0.00327992, 'B0': -126.1425, 'A1': 0.00317364, 'B1': -236.6446}
}

# /!\ IMPORTANT /!\
# Remplace ces valeurs par celles de ton fichier "bs_poses_cal.h"
# Sans les vraies positions physiques de tes balises dans la pièce, 
# la triangulation 3D absolue est impossible.
BS_POSES = {
    0: {
        'origin': np.array([0.0, 0.0, 0.0]),  # Coordonnées XYZ de la Base 0
        'R': np.eye(3)                        # Matrice de rotation 3x3 de la Base 0
    },
    1: {
        'origin': np.array([2.0, 2.0, 2.0]),  # Coordonnées XYZ de la Base 1
        'R': np.eye(3)                        # Matrice de rotation 3x3 de la Base 1
    }
}

# Buffer d'état pour accumuler les LFSR : state[sensor][bs][sweep]
state = {
    sensor: {
        0: {0: None, 1: None}, # Base 0
        1: {0: None, 1: None}  # Base 1
    } for sensor in range(4)
}

# Regex pour décoder le format brut de Said
regex_pattern = re.compile(r"\{sensor_id\s*=\s*(\d+)\}\s*\|\|\s*\{base\s*=\s*(\d+)\}\s*\|\|\s*\{poly\s*=\s*(\d+)\}\s*\|\|\s*\{lfsr\s*=\s*(\d+)\}")

# ===========================================================================
# 2. Moteur Mathématique (Angle & Solve 3D)
# ===========================================================================

def decode_angles(bs, lfsr0, lfsr1):
    """Convertit les 2 balayages LFSR en Azimut (h) et Elévation (v) en Radians."""
    cal = CALIBRATION.get(bs)
    
    s0_rad = math.radians((lfsr0 * cal['A0']) + cal['B0'])
    s1_rad = math.radians((lfsr1 * cal['A1']) + cal['B1'])

    horiz_rad = (s0_rad + s1_rad) / 2.0
    dt_rad = (s1_rad - s0_rad) / 2.0
    
    try:
        q = math.sin(dt_rad) / TAN_30
        vert_rad = math.atan(q * math.sqrt(1.0 + math.tan(horiz_rad)**2))
        return horiz_rad, vert_rad
    except ValueError:
        return None, None

def triangulate_crossing_beams(h0, v0, h1, v1):
    """Calcule l'intersection 3D la plus proche entre les deux rayons des Base Stations."""
    
    # --- Rayon 1 (Base 0) ---
    # Vecteur local issu de l'inversion de atan2(y, x) et atan2(z, x)
    d0_loc = np.array([1.0, math.tan(h0), math.tan(v0)])
    d0_loc = d0_loc / np.linalg.norm(d0_loc) # Normalisation
    d0_global = BS_POSES[0]['R'] @ d0_loc    # Rotation dans le repère monde
    o0 = BS_POSES[0]['origin']

    # --- Rayon 2 (Base 1) ---
    d1_loc = np.array([1.0, math.tan(h1), math.tan(v1)])
    d1_loc = d1_loc / np.linalg.norm(d1_loc)
    d1_global = BS_POSES[1]['R'] @ d1_loc
    o1 = BS_POSES[1]['origin']

    # --- Triangulation (Plus courte distance entre 2 droites gauches) ---
    w0 = o0 - o1
    a = np.dot(d0_global, d0_global)
    b = np.dot(d0_global, d1_global)
    c = np.dot(d1_global, d1_global)
    d = np.dot(d0_global, w0)
    e = np.dot(d1_global, w0)

    denom = a * c - b * b
    if denom < 1e-6:
        return None # Les rayons sont parallèles (Singularité)

    # Paramètres de distance t le long de chaque rayon
    t0 = (b * e - c * d) / denom
    t1 = (a * e - b * d) / denom

    # Points les plus proches sur chaque rayon
    p0 = o0 + t0 * d0_global
    p1 = o1 + t1 * d1_global

    # Le centre du segment reliant les deux rayons est la position estimée
    centroid = (p0 + p1) / 2.0
    return centroid

# ===========================================================================
# 3. Boucle d'Écoute
# ===========================================================================

def main():
    print(f"Ouverture du port {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Erreur d'ouverture : {e}")
        sys.exit(1)
        
    print("En attente des LFSR purs de Said pour triangulation 3D...")
    
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Données brutes" in line:
                continue

            match = regex_pattern.search(line)
            if match:
                sensor = int(match.group(1))
                poly = int(match.group(3))
                lfsr = int(match.group(4))

                # Le numéro de polynôme définit la BS et le Sweep
                bs = poly // 2
                sweep = poly % 2

                # On ne stocke que les données des BS 0 et 1 (celles qu'on a calibrées)
                if bs in [0, 1] and sensor in state:
                    state[sensor][bs][sweep] = lfsr

                    # Vérifier si on a les 4 LFSR (BS0_sw0, BS0_sw1, BS1_sw0, BS1_sw1) pour ce capteur
                    if all(state[sensor][0].values()) and all(state[sensor][1].values()):
                        
                        # Décoder les angles pour BS0
                        h0, v0 = decode_angles(0, state[sensor][0][0], state[sensor][0][1])
                        # Décoder les angles pour BS1
                        h1, v1 = decode_angles(1, state[sensor][1][0], state[sensor][1][1])
                        
                        if h0 is not None and h1 is not None:
                            # TRIANGULATION 3D !
                            pos_3d = triangulate_crossing_beams(h0, v0, h1, v1)
                            
                            if pos_3d is not None:
                                print(f"P,{sensor},{pos_3d[0]:.4f},{pos_3d[1]:.4f},{pos_3d[2]:.4f}")
                                
                        # Réinitialiser le buffer de ce capteur pour la prochaine capture
                        state[sensor][0] = {0: None, 1: None}
                        state[sensor][1] = {0: None, 1: None}

        except KeyboardInterrupt:
            print("\nArrêt.")
            ser.close()
            break
        except Exception:
            pass

if __name__ == '__main__':
    main()
