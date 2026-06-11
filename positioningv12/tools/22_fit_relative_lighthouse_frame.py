#!/usr/bin/env python3

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import math
import os
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from lh2_factory_model import lh2_factory_angle, load_factory_calibration_map


TILT_POS = math.pi / 6.0
TILT_NEG = -math.pi / 6.0


def configure_numeric_threads():
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(name, "1")


def cpu_count_default():
    return max(1, os.cpu_count() or 1)


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def load_layout(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {
        int(item["sensor"]): np.array([
            float(item["x_m"]),
            float(item["y_m"]),
            float(item.get("z_m", 0.0)),
        ], dtype=float)
        for item in data["sensors"]
    }


def load_wave(path, max_family_spread_deg, min_channels, max_frames):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    selected = []
    for frame in data.get("frames", []):
        measurements = []
        for m in frame.get("measurements", []):
            families = [
                family for family in m.get("candidate_families", [])
                if float(family.get("angle_spread_deg", 0.0)) <= max_family_spread_deg
            ]
            if not families:
                continue
            nm = dict(m)
            nm["candidate_families"] = families
            measurements.append(nm)

        channels = len({
            (int(m["sensor"]), int(m["basestation"]), int(m["sweep"]))
            for m in measurements
        })
        if channels >= min_channels:
            nf = dict(frame)
            nf["measurements"] = measurements
            nf["channels"] = channels
            selected.append(nf)

    if max_frames == 0:
        selected = []
    elif max_frames and len(selected) > max_frames:
        idxs = np.linspace(0, len(selected) - 1, int(max_frames)).round().astype(int)
        selected = [selected[int(i)] for i in idxs]

    return data, selected


def load_anchor_poses(path, max_family_spread_deg):
    if not path:
        return {"poses": []}, []
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    anchors = []
    for pose in data.get("poses", []):
        measurements = []
        for m in pose.get("measurements", []):
            families = [
                family for family in m.get("candidate_families", [])
                if float(family.get("angle_spread_deg", 0.0)) <= max_family_spread_deg
            ]
            if not families:
                continue
            nm = dict(m)
            nm["candidate_families"] = families
            measurements.append(nm)

        if not measurements:
            continue
        item = dict(pose)
        item["measurements"] = measurements
        item["channels"] = len({
            (int(m["sensor"]), int(m["basestation"]), int(m["sweep"]))
            for m in measurements
        })
        anchors.append(item)

    return data, anchors


def sensor_world(frame_params, local):
    rvec = frame_params[:3]
    t = frame_params[3:6]
    return Rotation.from_rotvec(rvec).as_matrix() @ local + t


def sensor_anchor_room(anchor, local):
    roll = math.radians(float(anchor.get("roll_deg", 0.0)))
    pitch = math.radians(float(anchor.get("pitch_deg", 0.0)))
    yaw = math.radians(float(anchor.get("yaw_deg", 0.0)))
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    t = np.array([
        float(anchor["x_m"]),
        float(anchor["y_m"]),
        float(anchor.get("z_m", 0.0)),
    ], dtype=float)
    return rot @ local + t


def room_to_bs4(params, p_room):
    rvec = params[:3]
    t = params[3:6]
    return Rotation.from_rotvec(rvec).as_matrix() @ p_room + t


def room_frame_lighthouse_summary(params):
    room_to_bs4_rot = Rotation.from_rotvec(params[:3]).as_matrix()
    bs10_to_bs4_rot = Rotation.from_rotvec(params[6:9]).as_matrix()
    bs10_in_bs4 = params[9:12]
    room_to_bs4_t = params[3:6]

    bs4_to_room_rot = room_to_bs4_rot.T
    bs4_room_position = -(bs4_to_room_rot @ room_to_bs4_t)
    bs10_to_room_rot = bs4_to_room_rot @ bs10_to_bs4_rot.T
    bs10_room_offset = bs4_to_room_rot @ bs10_in_bs4
    bs10_room_position = bs4_room_position + bs10_room_offset

    return {
        "bs4_room_position": bs4_room_position,
        "bs10_room_position": bs10_room_position,
        "bs4_euler_xyz_deg": Rotation.from_matrix(bs4_to_room_rot).as_euler("xyz", degrees=True),
        "bs10_euler_xyz_deg": Rotation.from_matrix(bs10_to_room_rot).as_euler("xyz", degrees=True),
        "bs10_room_offset": bs10_room_offset,
    }


def predict_bs4(p_world, sweep, factory_entry, convention):
    tilt = convention["bs4_tilts"][int(sweep)]
    axis = convention["bs4_axis_map"][int(sweep)]
    axis_cal = factory_entry["axes"].get(axis) if factory_entry else None
    offsets = convention["bs4_offsets"]
    return lh2_factory_angle(p_world, tilt, axis_cal) + offsets[int(sweep)]


def predict_bs10(p_world, sweep, bs10_params, factory_entry, convention):
    rvec = bs10_params[:3]
    t = bs10_params[3:6]
    p_lh = Rotation.from_rotvec(rvec).as_matrix() @ (p_world - t)
    tilt = convention["bs10_tilts"][int(sweep)]
    axis = convention["bs10_axis_map"][int(sweep)]
    axis_cal = factory_entry["axes"].get(axis) if factory_entry else None
    offsets = convention["bs10_offsets"]
    return lh2_factory_angle(p_lh, tilt, axis_cal) + offsets[int(sweep)]


def best_family_residual(pred, families, sign):
    diffs = [angle_diff(pred, sign * float(family["raw_angle_rad"])) for family in families]
    return min(diffs, key=abs)


def convention_from_params(params, hypothesis):
    bs10 = params[6:12]
    return {
        "bs4_tilts": hypothesis["bs4_tilts"],
        "bs4_axis_map": hypothesis["bs4_axis_map"],
        "bs10_tilts": hypothesis["bs10_tilts"],
        "bs10_axis_map": hypothesis["bs10_axis_map"],
        "signs": hypothesis["signs"],
        "bs4_offsets": {0: params[12], 1: params[13]},
        "bs10_offsets": {0: params[14], 1: params[15]},
    }, bs10


def residual_vector(
    params,
    frames,
    anchors,
    layout,
    factory_calibs,
    robust_scale,
    hypothesis,
    bs_distance_prior,
    bs_distance_sigma,
    bs_distance_weight,
    same_height_prior,
    same_height_sigma,
    same_height_weight,
    bs4_height_prior,
    bs4_height_sigma,
    bs4_height_weight,
):
    convention, bs10 = convention_from_params(params, hypothesis)
    pose_base = 16

    out = []
    for anchor in anchors:
        for m in anchor.get("measurements", []):
            sensor = int(m["sensor"])
            if sensor not in layout:
                continue
            p_room = sensor_anchor_room(anchor, layout[sensor])
            p_world = room_to_bs4(params, p_room)
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            if bs == 4:
                pred = predict_bs4(p_world, sweep, factory_calibs.get(4), convention)
            elif bs == 10:
                pred = predict_bs10(p_world, sweep, bs10, factory_calibs.get(10), convention)
            else:
                continue
            sign = convention["signs"][(bs, sweep)]
            out.append(best_family_residual(pred, m.get("candidate_families", []), sign) / robust_scale)

    for frame_index, frame in enumerate(frames):
        pose = params[pose_base + frame_index * 6: pose_base + (frame_index + 1) * 6]
        for m in frame.get("measurements", []):
            sensor = int(m["sensor"])
            if sensor not in layout:
                continue
            p_world = sensor_world(pose, layout[sensor])
            bs = int(m["basestation"])
            sweep = int(m["sweep"])
            if bs == 4:
                pred = predict_bs4(p_world, sweep, factory_calibs.get(4), convention)
            elif bs == 10:
                pred = predict_bs10(p_world, sweep, bs10, factory_calibs.get(10), convention)
            else:
                continue
            sign = convention["signs"][(bs, sweep)]
            out.append(best_family_residual(pred, m.get("candidate_families", []), sign) / robust_scale)

    if bs_distance_prior and bs_distance_prior > 0.0:
        bs10_distance = float(np.linalg.norm(bs10[3:6]))
        sigma = max(float(bs_distance_sigma), 1e-6)
        distance_residual = (bs10_distance - float(bs_distance_prior)) / sigma
        for _ in range(max(1, int(bs_distance_weight))):
            out.append(distance_residual)

    if same_height_prior:
        room_from_bs4 = Rotation.from_rotvec(params[:3]).as_matrix().T
        bs10_room_offset = room_from_bs4 @ bs10[3:6]
        height_residual = float(bs10_room_offset[2]) / max(float(same_height_sigma), 1e-6)
        for _ in range(max(1, int(same_height_weight))):
            out.append(height_residual)

    if bs4_height_prior is not None:
        room_from_bs4 = Rotation.from_rotvec(params[:3]).as_matrix().T
        bs4_room_position = -(room_from_bs4 @ params[3:6])
        bs4_height_residual = (
            float(bs4_room_position[2]) - float(bs4_height_prior)
        ) / max(float(bs4_height_sigma), 1e-6)
        for _ in range(max(1, int(bs4_height_weight))):
            out.append(bs4_height_residual)

    return np.array(out, dtype=float)


def initial_params(frames, bs10_guess, room_to_bs4_guess, pose_seed=0):
    params = []
    params.extend([
        room_to_bs4_guess[0],
        room_to_bs4_guess[1],
        room_to_bs4_guess[2],
        room_to_bs4_guess[3],
        room_to_bs4_guess[4],
        room_to_bs4_guess[5],
    ])
    if len(bs10_guess) == 6:
        params.extend(bs10_guess)
    else:
        params.extend([0.0, 0.0, 0.0, bs10_guess[0], bs10_guess[1], bs10_guess[2]])
    params.extend([0.0, 0.0, 0.0, 0.0])

    n = max(1, len(frames))
    for i, _frame in enumerate(frames):
        phase = ((i / n) * 2.0 * math.pi) + float(pose_seed) * 0.7
        params.extend([
            0.15 * math.sin(phase * 0.7),
            0.0,
            phase * 0.1,
            1.0 + 0.20 * math.sin(phase),
            0.20 * math.cos(phase),
            0.20 + 0.15 * math.sin(phase * 0.5),
        ])
    return np.array(params, dtype=float)


def bounds(num_frames, position_bound):
    lower = [-math.pi, -math.pi, -math.pi, -position_bound, -position_bound, -position_bound]
    upper = [math.pi, math.pi, math.pi, position_bound, position_bound, position_bound]
    lower.extend([-math.pi, -math.pi, -math.pi, -position_bound, -position_bound, -position_bound])
    upper.extend([math.pi, math.pi, math.pi, position_bound, position_bound, position_bound])
    lower.extend([-math.pi, -math.pi, -math.pi, -math.pi])
    upper.extend([math.pi, math.pi, math.pi, math.pi])
    for _ in range(num_frames):
        lower.extend([-math.pi, -math.pi, -math.pi, -position_bound, -position_bound, -position_bound])
        upper.extend([math.pi, math.pi, math.pi, position_bound, position_bound, position_bound])
    return np.array(lower, dtype=float), np.array(upper, dtype=float)


def old_bounds(num_frames, position_bound):
    lower = [-math.pi, -math.pi, -math.pi, -position_bound, -position_bound, -position_bound]
    upper = [math.pi, math.pi, math.pi, position_bound, position_bound, position_bound]
    lower.extend([-math.pi, -math.pi, -math.pi, -math.pi])
    upper.extend([math.pi, math.pi, math.pi, math.pi])
    for _ in range(num_frames):
        lower.extend([-math.pi, -math.pi, -math.pi, -position_bound, -position_bound, -position_bound])
        upper.extend([math.pi, math.pi, math.pi, position_bound, position_bound, position_bound])
    return np.array(lower, dtype=float), np.array(upper, dtype=float)


def rmse_deg_for_params(params, frames, anchors, layout, factory_calibs, hypothesis):
    raw_residuals = residual_vector(
        params,
        frames,
        anchors,
        layout,
        factory_calibs,
        1.0,
        hypothesis,
        0.0,
        1.0,
        1,
        False,
        1.0,
        1,
        None,
        1.0,
        1,
    )
    return math.degrees(float(np.sqrt(np.mean(raw_residuals * raw_residuals))))


def fit_one_start(task):
    (
        start_index,
        x0,
        lower,
        upper,
        frames,
        anchors,
        layout,
        factory_calibs,
        robust_scale,
        max_nfev,
        hypothesis,
        bs_distance_prior,
        bs_distance_sigma,
        bs_distance_weight,
        same_height_prior,
        same_height_sigma,
        same_height_weight,
        bs4_height_prior,
        bs4_height_sigma,
        bs4_height_weight,
    ) = task
    result = least_squares(
        residual_vector,
        x0,
        bounds=(lower, upper),
        args=(
            frames,
            anchors,
            layout,
            factory_calibs,
            robust_scale,
            hypothesis,
            bs_distance_prior,
            bs_distance_sigma,
            bs_distance_weight,
            same_height_prior,
            same_height_sigma,
            same_height_weight,
            bs4_height_prior,
            bs4_height_sigma,
            bs4_height_weight,
        ),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=max_nfev,
        verbose=0,
    )
    rmse_deg = rmse_deg_for_params(result.x, frames, anchors, layout, factory_calibs, hypothesis)
    return {
        "start_index": int(start_index),
        "hypothesis": hypothesis,
        "x": result.x,
        "fun": result.fun,
        "success": bool(result.success),
        "message": str(result.message),
        "cost": float(result.cost),
        "nfev": int(result.nfev),
        "rmse_deg": float(rmse_deg),
    }


def make_start_guesses(base_guess, starts):
    if len(base_guess) == 6:
        rx, ry, rz, bx, by, bz = base_guess
    else:
        rx, ry, rz = 0.0, 0.0, 0.0
        bx, by, bz = base_guess
    guesses = [
        [rx, ry, rz, bx, by, bz],
        [rx, ry, rz, bx, by + 0.4, bz],
        [rx, ry, rz, bx, by - 0.4, bz],
        [rx, ry, rz, bx + 0.4, by, bz],
        [rx, ry, rz, bx - 0.4, by, bz],
        [rx, ry, rz, -bx, by, bz],
        [rx, ry, rz, bx, -by, bz],
        [rx, ry, rz, -bx, -by, bz],
        [rx, ry, rz, bx, by, bz + 0.3],
        [rx, ry, rz, bx, by, bz - 0.3],
    ]
    if len(base_guess) == 3:
        guesses = [guess[3:6] for guess in guesses]
    return guesses[:max(1, int(starts))]


def make_hypotheses(mode):
    if str(mode).lower() == "v10":
        signs = {
            (4, 0): 1.0,
            (4, 1): 1.0,
            (10, 0): 1.0,
            (10, 1): 1.0,
        }
        return [{
            "bs4_tilts": {0: TILT_NEG, 1: TILT_POS},
            "bs4_axis_map": {0: 1, 1: 0},
            "bs10_tilts": {0: TILT_POS, 1: TILT_NEG},
            "bs10_axis_map": {0: 0, 1: 1},
            "signs": signs,
            "label": "v10_like bs4_tilt=1 bs4_axis=10 bs10_tilt=0 bs10_axis=01 signs=1,1,1,1",
        }]

    tilt_maps = [
        {0: TILT_POS, 1: TILT_NEG},
        {0: TILT_NEG, 1: TILT_POS},
    ]
    axis_maps = [
        {0: 0, 1: 1},
        {0: 1, 1: 0},
    ]
    sign_maps = []
    if str(mode).lower() == "simple":
        sign_maps = [{
            (4, 0): 1.0,
            (4, 1): 1.0,
            (10, 0): 1.0,
            (10, 1): 1.0,
        }]
    else:
        for s40 in (1.0, -1.0):
            for s41 in (1.0, -1.0):
                for s100 in (1.0, -1.0):
                    for s101 in (1.0, -1.0):
                        sign_maps.append({
                            (4, 0): s40,
                            (4, 1): s41,
                            (10, 0): s100,
                            (10, 1): s101,
                        })

    hypotheses = []
    for bs4_tilt_map in tilt_maps:
        for bs4_axis_map in axis_maps:
            for bs10_tilt_map in tilt_maps:
                for bs10_axis_map in axis_maps:
                    for signs in sign_maps:
                        hypotheses.append({
                            "bs4_tilts": bs4_tilt_map,
                            "bs4_axis_map": bs4_axis_map,
                            "bs10_tilts": bs10_tilt_map,
                            "bs10_axis_map": bs10_axis_map,
                            "signs": signs,
                            "label": (
                                f"bs4_tilt={0 if bs4_tilt_map[0] == TILT_POS else 1} "
                                f"bs4_axis={bs4_axis_map[0]}{bs4_axis_map[1]} "
                                f"bs10_tilt={0 if bs10_tilt_map[0] == TILT_POS else 1} "
                                f"bs10_axis={bs10_axis_map[0]}{bs10_axis_map[1]} "
                                f"signs={int(signs[(4,0)])},{int(signs[(4,1)])},{int(signs[(10,0)])},{int(signs[(10,1)])}"
                            ),
                        })
    return hypotheses


class ResultView:
    def __init__(self, data):
        self.x = data["x"]
        self.fun = data["fun"]
        self.success = data["success"]
        self.message = data["message"]
        self.hypothesis = data.get("hypothesis")


def save_geometry(path, args, wave_meta, frames, anchors, result, rmse_deg):
    params = result.x
    hypothesis = getattr(result, "hypothesis", None)
    room_summary = room_frame_lighthouse_summary(params)
    bs10_room_offset = room_summary["bs10_room_offset"]
    out = {
        "description": "v12 relative Lighthouse geometry. BS4 is fixed as the reference frame.",
        "input_wave": args.wave,
        "input_anchor_poses": args.anchor_poses,
        "reference_basestation": 4,
        "fit": {
            "frames_used": len(frames),
            "anchor_poses_used": len(anchors),
            "residuals": int(result.fun.size),
            "rmse_deg": float(rmse_deg),
            "success": bool(result.success),
            "message": str(result.message),
            "priors": {
                "bs_distance_m": float(args.bs_distance_prior),
                "bs_distance_sigma_m": float(args.bs_distance_sigma),
                "bs_distance_weight": int(args.bs_distance_weight),
                "same_height_enabled": bool(args.same_height_prior),
                "same_height_sigma_m": float(args.same_height_sigma),
                "same_height_weight": int(args.same_height_weight),
                "bs4_height_m": None if args.bs4_height_prior is None else float(args.bs4_height_prior),
                "bs4_height_sigma_m": float(args.bs4_height_sigma),
                "bs4_height_weight": int(args.bs4_height_weight),
            },
            "bs4_room_frame_position_m": [float(v) for v in room_summary["bs4_room_position"]],
            "bs10_room_frame_position_m": [float(v) for v in room_summary["bs10_room_position"]],
            "bs10_room_frame_offset_from_bs4_m": [float(v) for v in bs10_room_offset],
            "room_frame_orientation_xyz_deg": {
                "bs4": [float(v) for v in room_summary["bs4_euler_xyz_deg"]],
                "bs10": [float(v) for v in room_summary["bs10_euler_xyz_deg"]],
            },
            "hypothesis": None if hypothesis is None else {
                "label": hypothesis["label"],
                "bs4_tilts_deg": {
                    "sweep_0": float(math.degrees(hypothesis["bs4_tilts"][0])),
                    "sweep_1": float(math.degrees(hypothesis["bs4_tilts"][1])),
                },
                "bs4_axis_map": {
                    "sweep_0": int(hypothesis["bs4_axis_map"][0]),
                    "sweep_1": int(hypothesis["bs4_axis_map"][1]),
                },
                "bs10_tilts_deg": {
                    "sweep_0": float(math.degrees(hypothesis["bs10_tilts"][0])),
                    "sweep_1": float(math.degrees(hypothesis["bs10_tilts"][1])),
                },
                "bs10_axis_map": {
                    "sweep_0": int(hypothesis["bs10_axis_map"][0]),
                    "sweep_1": int(hypothesis["bs10_axis_map"][1]),
                },
                "signs": {
                    "bs4_sweep0": float(hypothesis["signs"][(4, 0)]),
                    "bs4_sweep1": float(hypothesis["signs"][(4, 1)]),
                    "bs10_sweep0": float(hypothesis["signs"][(10, 0)]),
                    "bs10_sweep1": float(hypothesis["signs"][(10, 1)]),
                },
            },
        },
        "basestations": [
            {
                "basestation": 4,
                "role": "reference",
                "world_to_lighthouse": {
                    "rotation_vector": [0.0, 0.0, 0.0],
                    "translation_m": [0.0, 0.0, 0.0],
                },
                "angle_offsets_deg": {
                    "sweep_0": float(math.degrees(params[12])),
                    "sweep_1": float(math.degrees(params[13])),
                },
            },
            {
                "basestation": 10,
                "role": "solved_relative_to_bs4",
                "world_to_lighthouse": {
                    "rotation_vector": [float(v) for v in params[6:9]],
                    "translation_m": [float(v) for v in params[9:12]],
                },
                "angle_offsets_deg": {
                    "sweep_0": float(math.degrees(params[14])),
                    "sweep_1": float(math.degrees(params[15])),
                },
            },
        ],
        "room_to_bs4": {
            "rotation_vector": [float(v) for v in params[:3]],
            "translation_m": [float(v) for v in params[3:6]],
        },
        "wave_summary": {
            "total_frames_in_file": len(wave_meta.get("frames", [])),
            "frames_used": len(frames),
            "cluster_deg": wave_meta.get("cluster_deg"),
        },
        "anchor_summary": {
            "poses_used": [anchor.get("name") for anchor in anchors],
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")


def main():
    configure_numeric_threads()
    parser = argparse.ArgumentParser(description="Fit a v12 relative Lighthouse frame from anchored poses and optional LH2A wave.")
    parser.add_argument("--wave", default="config/lh2a_wave_record.json")
    parser.add_argument("--anchor-poses", default="", help="Known room poses with LH2A families, usually a vertical_pose_variants/*.json file.")
    parser.add_argument("--layout", default="config/sensors_layout.json")
    parser.add_argument("--factory-calibs", default="auto")
    parser.add_argument("--output", default="config/lighthouse_relative_geometry.json")
    parser.add_argument("--max-frames", type=int, default=40, help="Use an evenly spaced subset for the first fit.")
    parser.add_argument("--max-family-spread-deg", type=float, default=1.0)
    parser.add_argument("--max-anchor-spread-deg", type=float, default=0.5)
    parser.add_argument("--min-channels", type=int, default=12)
    parser.add_argument("--position-bound", type=float, default=4.0)
    parser.add_argument("--bs10-guess", default="1.4,0,0", help="Initial BS10 translation in the BS4 frame.")
    parser.add_argument("--bs-distance-prior", type=float, default=0.0, help="Optional expected distance between BS4 and BS10 in meters.")
    parser.add_argument("--bs-distance-sigma", type=float, default=0.15, help="Soft prior sigma for --bs-distance-prior, in meters.")
    parser.add_argument("--bs-distance-weight", type=int, default=1, help="Repeat the BS distance residual this many times to make the block constraint stronger.")
    parser.add_argument("--same-height-prior", action="store_true", help="Constrain BS4 and BS10 to the same room-frame height.")
    parser.add_argument("--same-height-sigma", type=float, default=0.04, help="Soft prior sigma for same Lighthouse height, in meters.")
    parser.add_argument("--same-height-weight", type=int, default=80, help="Repeat the same-height residual this many times.")
    parser.add_argument("--bs4-height-prior", type=float, default=None, help="Optional expected BS4 height in the room frame, in meters.")
    parser.add_argument("--bs4-height-sigma", type=float, default=0.05, help="Soft prior sigma for --bs4-height-prior, in meters.")
    parser.add_argument("--bs4-height-weight", type=int, default=80, help="Repeat the BS4 height residual this many times.")
    parser.add_argument("--room-to-bs4-guess", default="0,0,0,0,0,0", help="Initial room->BS4 rvec,t as rx,ry,rz,x,y,z.")
    parser.add_argument("--max-nfev", type=int, default=250)
    parser.add_argument("--starts", type=int, default=8, help="Number of independent initial guesses to try.")
    parser.add_argument("--workers", type=int, default=cpu_count_default(), help="Parallel worker processes. Use 1 to disable.")
    parser.add_argument("--convention-search", choices=["v10", "simple", "all"], default="all", help="Search sweep tilt/axis/sign conventions.")
    args = parser.parse_args()

    wave_meta, frames = load_wave(args.wave, args.max_family_spread_deg, args.min_channels, args.max_frames)
    anchor_meta, anchors = load_anchor_poses(args.anchor_poses, args.max_anchor_spread_deg)
    layout = load_layout(args.layout)
    factory_calibs = load_factory_calibration_map(args.factory_calibs)
    bs10_guess = [float(v) for v in args.bs10_guess.split(",")]
    if len(bs10_guess) not in (3, 6):
        raise SystemExit("--bs10-guess must contain either 3 values (x,y,z) or 6 values (rx,ry,rz,x,y,z).")
    room_to_bs4_guess = [float(v) for v in args.room_to_bs4_guess.split(",")]
    if len(room_to_bs4_guess) != 6:
        raise SystemExit("--room-to-bs4-guess must contain 6 comma-separated numbers.")
    lower, upper = bounds(len(frames), args.position_bound)
    robust_scale = math.radians(1.0)
    start_guesses = make_start_guesses(bs10_guess, args.starts)
    hypotheses = make_hypotheses(args.convention_search)
    tasks = []
    task_index = 0
    for hypothesis in hypotheses:
        for guess_index, guess in enumerate(start_guesses):
            tasks.append((
                task_index,
                np.clip(
                    initial_params(frames, guess, room_to_bs4_guess, pose_seed=guess_index),
                    lower + 1e-6,
                    upper - 1e-6,
                ),
                lower,
                upper,
                frames,
                anchors,
                layout,
                factory_calibs,
                robust_scale,
                args.max_nfev,
                hypothesis,
                args.bs_distance_prior,
                args.bs_distance_sigma,
                args.bs_distance_weight,
                args.same_height_prior,
                args.same_height_sigma,
                args.same_height_weight,
                args.bs4_height_prior,
                args.bs4_height_sigma,
                args.bs4_height_weight,
            ))
            task_index += 1

    print("=" * 88)
    print("v12 relative Lighthouse fit")
    print(f"Wave: {args.wave}")
    print(f"Anchor poses: {args.anchor_poses or '(none)'}")
    print(f"Frames in file: {len(wave_meta.get('frames', []))} | frames used: {len(frames)}")
    print(f"Anchor poses used: {len(anchors)}")
    print(
        f"Variables/start: {tasks[0][1].size if tasks else 0} | "
        f"starts={len(start_guesses)} | hypotheses={len(hypotheses)} | "
        f"tasks={len(tasks)} | workers={args.workers} | max_nfev={args.max_nfev}"
    )
    print("Reference: BS4 fixed at origin")
    if args.bs_distance_prior and args.bs_distance_prior > 0.0:
        print(
            f"BS distance prior: {args.bs_distance_prior:.3f} m +/- {args.bs_distance_sigma:.3f} m "
            f"x weight {args.bs_distance_weight}"
        )
    if args.same_height_prior:
        print(
            f"Same-height prior: room-frame dz=0.000 m +/- {args.same_height_sigma:.3f} m "
            f"x weight {args.same_height_weight}"
        )
    if args.bs4_height_prior is not None:
        print(
            f"BS4 height prior: room-frame z={args.bs4_height_prior:.3f} m "
            f"+/- {args.bs4_height_sigma:.3f} m x weight {args.bs4_height_weight}"
        )
    print("=" * 88)
    if not frames and not anchors:
        raise SystemExit("No usable wave frames or anchor poses after filtering.")

    best = None
    workers = max(1, int(args.workers or 1))
    if workers == 1 or len(tasks) == 1:
        for task in tasks:
            candidate = fit_one_start(task)
            if best is None or candidate["rmse_deg"] < best["rmse_deg"]:
                best = candidate
            print(
                f"  task {candidate['start_index'] + 1}/{len(tasks)} "
                f"rmse={candidate['rmse_deg']:.3f} deg | best={best['rmse_deg']:.3f} deg | "
                f"{candidate['hypothesis']['label']}",
                flush=True,
            )
    else:
        with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as executor:
            futures = [executor.submit(fit_one_start, task) for task in tasks]
            done = 0
            for future in as_completed(futures):
                done += 1
                candidate = future.result()
                if best is None or candidate["rmse_deg"] < best["rmse_deg"]:
                    best = candidate
                print(
                    f"  task {done}/{len(tasks)} "
                    f"rmse={candidate['rmse_deg']:.3f} deg | best={best['rmse_deg']:.3f} deg | "
                    f"{candidate['hypothesis']['label']}",
                    flush=True,
                )

    result = ResultView(best)
    rmse_deg = best["rmse_deg"]
    save_geometry(Path(args.output), args, wave_meta, frames, anchors, result, rmse_deg)

    bs10_t = result.x[9:12]
    room_t = result.x[3:6]
    room_summary = room_frame_lighthouse_summary(result.x)
    bs10_room_offset = room_summary["bs10_room_offset"]
    bs4_room_position = room_summary["bs4_room_position"]
    bs10_room_position = room_summary["bs10_room_position"]
    bs4_euler = room_summary["bs4_euler_xyz_deg"]
    bs10_euler = room_summary["bs10_euler_xyz_deg"]
    print("=" * 88)
    print(f"RMSE: {rmse_deg:.3f} deg")
    print(f"BS10 translation in BS4 frame: x={bs10_t[0]:+.3f}, y={bs10_t[1]:+.3f}, z={bs10_t[2]:+.3f} m")
    print(f"BS4-BS10 distance: {float(np.linalg.norm(bs10_t)):.3f} m")
    print(f"BS4 room position: x={bs4_room_position[0]:+.3f}, y={bs4_room_position[1]:+.3f}, z={bs4_room_position[2]:+.3f} m")
    print(f"BS10 room position: x={bs10_room_position[0]:+.3f}, y={bs10_room_position[1]:+.3f}, z={bs10_room_position[2]:+.3f} m")
    print(f"BS10 room-frame offset from BS4: x={bs10_room_offset[0]:+.3f}, y={bs10_room_offset[1]:+.3f}, z={bs10_room_offset[2]:+.3f} m")
    print(f"BS4 room orientation xyz: roll={bs4_euler[0]:+.1f}, pitch={bs4_euler[1]:+.1f}, yaw={bs4_euler[2]:+.1f} deg")
    print(f"BS10 room orientation xyz: roll={bs10_euler[0]:+.1f}, pitch={bs10_euler[1]:+.1f}, yaw={bs10_euler[2]:+.1f} deg")
    print(f"Room origin in BS4 frame: x={room_t[0]:+.3f}, y={room_t[1]:+.3f}, z={room_t[2]:+.3f} m")
    print(f"Best convention: {best['hypothesis']['label']}")
    print(f"Saved: {args.output}")
    print("=" * 88)


if __name__ == "__main__":
    main()
