import json
import math
import cv2
import numpy as np

layout = json.load(open("config/sensors_layout.json"))
origin = json.load(open("config/origin_angles.json"))

def angles_to_image_point(theta, phi):
    return [math.tan(theta), math.tan(phi) / math.cos(theta)]

for bs in [4, 10]:
    obj = []
    img = []

    for sid, sinfo in layout["sensors"].items():
        m = origin["measurements"][str(bs)][sid]
        obj.append([sinfo["x"], sinfo["y"], sinfo["z"]])
        img.append(angles_to_image_point(m["theta"], m["phi"]))

    obj = np.array(obj, dtype=np.float64)
    img = np.array(img, dtype=np.float64)
    K = np.eye(3, dtype=np.float64)
    dist = np.zeros((4, 1), dtype=np.float64)

    retval, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
        obj,
        img,
        K,
        dist,
        flags=cv2.SOLVEPNP_IPPE
    )

    print()
    print("BS", bs, "solutions:", len(rvecs))

    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        R, _ = cv2.Rodrigues(rvec)
        pos = (-R.T @ tvec).reshape(3)

        reproj_value = np.array(reproj).reshape(-1)[i]

        print(
            f"sol {i}: "
            f"pos x={pos[0]:+.3f}, "
            f"y={pos[1]:+.3f}, "
            f"z={pos[2]:+.3f}, "
            f"reproj={float(reproj_value):.6f}"
        )
