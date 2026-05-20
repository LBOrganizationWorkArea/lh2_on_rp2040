#!/usr/bin/env python3

import math
from dataclasses import dataclass
from statistics import median


# Bitcraze LH2 cycle periods. The base station periods are expressed for a
# 48 MHz clock in the public FPGA tools; the RP2040 path uses 24 MHz ticks.
CYCLE_PERIODS = [
    959000 / 2, 957000 / 2, 953000 / 2, 949000 / 2,
    947000 / 2, 943000 / 2, 941000 / 2, 939000 / 2,
    937000 / 2, 929000 / 2, 919000 / 2, 911000 / 2,
    907000 / 2, 901000 / 2, 893000 / 2, 887000 / 2,
]

EXPECTED_POLYS = {
    4: (8, 9),
    10: (20, 21),
}

POLY_TO_AXIS = {
    8: 0,
    20: 0,
    9: 1,
    21: 1,
}


@dataclass(frozen=True)
class Lh2pFrame:
    basestation: int
    sweep0: int
    sweep1: int
    poly0: int
    poly1: int
    block0: int
    block1: int
    delta: int
    offsets: tuple[tuple[int, int], ...]
    raw_line: str


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def circular_median(values):
    values = [float(value) for value in values]
    if not values:
        return 0.0

    best = min(values, key=lambda candidate: sum(abs(angle_diff(value, candidate)) for value in values))
    return float(best)


def circular_spread_deg(values, center):
    if not values:
        return 0.0
    return float(math.degrees(max(abs(angle_diff(float(value), center)) for value in values)))


def parse_lh2p_line(line):
    line = line.strip()
    if not line.startswith("LH2P;"):
        return None

    parts = line.split(";")
    if len(parts) != 17:
        return None

    try:
        values = [int(item) for item in parts[1:]]
    except ValueError:
        return None

    bs, sweep0, sweep1, poly0, poly1, block0, block1, delta = values[:8]
    raw_offsets = values[8:]
    offsets = tuple((raw_offsets[i], raw_offsets[i + 1]) for i in range(0, 8, 2))

    return Lh2pFrame(
        basestation=bs,
        sweep0=sweep0,
        sweep1=sweep1,
        poly0=poly0,
        poly1=poly1,
        block0=block0,
        block1=block1,
        delta=delta,
        offsets=offsets,
        raw_line=line,
    )


def frame_axes(frame):
    axis0 = POLY_TO_AXIS.get(frame.poly0)
    axis1 = POLY_TO_AXIS.get(frame.poly1)
    if axis0 is None or axis1 is None or axis0 == axis1:
        return None

    expected = EXPECTED_POLYS.get(frame.basestation)
    if expected is not None and set((frame.poly0, frame.poly1)) != set(expected):
        return None

    return axis0, axis1


def offset_to_raw_angle(offset, basestation, axis):
    period = CYCLE_PERIODS[int(basestation)]
    angle = float(int(offset) % int(period)) * 2.0 * math.pi / period - math.pi
    if int(axis) == 0:
        return angle + math.pi / 3.0
    return angle - math.pi / 3.0


def _clip1(value):
    return max(-0.999999, min(0.999999, value))


def _measurement_model_lh2(x, y, z, tilt, calibration):
    ax = math.atan2(y, x)
    r = math.sqrt(x * x + y * y)
    if r < 1e-9:
        r = 1e-9

    base = ax + math.asin(_clip1(z * math.tan(tilt - float(calibration["tilt"])) / r))
    comp_gib = -float(calibration["gibmag"]) * math.cos(ax + float(calibration["gibphase"]))
    return base - (float(calibration["phase"]) + comp_gib)


def _ideal_to_distorted_lh2(ideal_axis0, ideal_axis1, axes_calibration):
    tilt_30 = math.pi / 6.0
    tan_30 = 0.5773502691896258

    x = 1.0
    y = math.tan((ideal_axis1 + ideal_axis0) / 2.0)
    denom = tan_30 * (math.cos(ideal_axis1) + math.cos(ideal_axis0))
    if abs(denom) < 1e-9:
        denom = math.copysign(1e-9, denom if denom else 1.0)
    z = math.sin(ideal_axis1 - ideal_axis0) / denom

    return (
        _measurement_model_lh2(x, y, z, -tilt_30, axes_calibration[0]),
        _measurement_model_lh2(x, y, z, tilt_30, axes_calibration[1]),
    )


def apply_factory_calibration_lh2(raw_axis0, raw_axis1, axes_calibration):
    if not axes_calibration:
        return raw_axis0, raw_axis1

    estimated0 = float(raw_axis0)
    estimated1 = float(raw_axis1)

    for _ in range(5):
        distorted0, distorted1 = _ideal_to_distorted_lh2(estimated0, estimated1, axes_calibration)
        delta0 = angle_diff(float(raw_axis0), distorted0)
        delta1 = angle_diff(float(raw_axis1), distorted1)
        estimated0 += delta0
        estimated1 += delta1
        if abs(delta0) < 0.0005 and abs(delta1) < 0.0005:
            break

    return estimated0, estimated1


def frame_to_observations(frame, factory_by_bs=None):
    axes = frame_axes(frame)
    if axes is None:
        return []

    bs = int(frame.basestation)
    first_axis, second_axis = axes
    factory_entry = (factory_by_bs or {}).get(bs)
    axes_calibration = factory_entry.get("axes") if factory_entry else None

    observations = []
    for sensor, pair in enumerate(frame.offsets):
        if pair[0] == 0 and pair[1] == 0:
            continue

        raw_by_axis = {
            first_axis: offset_to_raw_angle(pair[0], bs, first_axis),
            second_axis: offset_to_raw_angle(pair[1], bs, second_axis),
        }
        calibrated0, calibrated1 = apply_factory_calibration_lh2(
            raw_by_axis[0],
            raw_by_axis[1],
            axes_calibration,
        )
        calibrated0 = angle_diff(calibrated0, 0.0)
        calibrated1 = angle_diff(calibrated1, 0.0)
        calibrated_by_axis = {0: calibrated0, 1: calibrated1}
        offset_by_axis = {first_axis: pair[0], second_axis: pair[1]}
        poly_by_axis = {first_axis: frame.poly0, second_axis: frame.poly1}

        for axis in (0, 1):
            observations.append({
                "sensor": int(sensor),
                "basestation": bs,
                "sweep": int(axis),
                "axis": int(axis),
                "polynomial": int(poly_by_axis[axis]),
                "lfsr_location": int(offset_by_axis[axis]),
                "offset_ticks": int(offset_by_axis[axis]),
                "raw_angle_rad": float(raw_by_axis[axis]),
                "calibrated_angle_rad": float(calibrated_by_axis[axis]),
            })

    return observations


def _frame_family_key(frame):
    values = _frame_axis_values(frame)
    if values is None:
        return None

    axis0_values, axis1_values = values
    return median(axis0_values), median(axis1_values)


def _frame_axis_values(frame):
    axes = frame_axes(frame)
    if axes is None:
        return None

    first_axis, second_axis = axes
    period = int(CYCLE_PERIODS[int(frame.basestation)])
    axis0_values = []
    axis1_values = []
    for pair in frame.offsets:
        if pair[0] == 0 and pair[1] == 0:
            continue

        if first_axis == 0:
            axis0_values.append(int(pair[0]) % period)
            axis1_values.append(int(pair[1]) % period)
        else:
            axis0_values.append(int(pair[1]) % period)
            axis1_values.append(int(pair[0]) % period)

    if not axis0_values or not axis1_values:
        return None

    return axis0_values, axis1_values


def _period_distance_ticks(a, b, period):
    diff = abs((int(a) % int(period)) - (int(b) % int(period)))
    return min(diff, int(period) - diff)


def _axis_circular_spread(values, period):
    if not values:
        return float("inf")
    center = median(values)
    return max(_period_distance_ticks(value, center, period) for value in values)


def frame_internal_spread_ticks(frame):
    values = _frame_axis_values(frame)
    if values is None:
        return float("inf")

    period = CYCLE_PERIODS[int(frame.basestation)]
    axis0_values, axis1_values = values
    return max(
        _axis_circular_spread(axis0_values, period),
        _axis_circular_spread(axis1_values, period),
    )


def frame_has_coherent_sensor_offsets(frame, max_sensor_spread_ticks=50000):
    return frame_internal_spread_ticks(frame) <= max_sensor_spread_ticks


def select_clean_lh2p_frames(frames, cluster_ticks=25000, min_frames=1, max_sensor_spread_ticks=None):
    by_bs = {}
    for frame in frames:
        if frame_axes(frame) is None:
            continue
        if max_sensor_spread_ticks is not None and not frame_has_coherent_sensor_offsets(frame, max_sensor_spread_ticks):
            continue
        by_bs.setdefault(frame.basestation, []).append(frame)

    selected = []
    for bs, bs_frames in by_bs.items():
        period = int(CYCLE_PERIODS[int(bs)])
        clusters = []
        for frame in bs_frames:
            key = _frame_family_key(frame)
            if key is None:
                continue

            best_cluster = None
            best_distance = None
            for cluster in clusters:
                center0, center1 = cluster["center"]
                distance = max(
                    _period_distance_ticks(key[0], center0, period),
                    _period_distance_ticks(key[1], center1, period),
                )
                if distance <= cluster_ticks and (best_distance is None or distance < best_distance):
                    best_cluster = cluster
                    best_distance = distance

            if best_cluster is None:
                clusters.append({"center": key, "frames": [frame]})
            else:
                best_cluster["frames"].append(frame)
                keys = [_frame_family_key(item) for item in best_cluster["frames"]]
                best_cluster["center"] = (median(k[0] for k in keys), median(k[1] for k in keys))

        if not clusters:
            continue

        clusters.sort(key=lambda c: (len(c["frames"]), -_cluster_spread(c, period)), reverse=True)
        if len(clusters[0]["frames"]) >= min_frames:
            selected.extend(clusters[0]["frames"])

    return selected


def summarize_observation_group(values, angle_outlier_deg=8.0, min_samples=1):
    if not values:
        return None

    angle_key = None
    if any("calibrated_angle_rad" in item for item in values):
        angle_key = "calibrated_angle_rad"
    elif any("raw_angle_rad" in item for item in values):
        angle_key = "raw_angle_rad"

    kept = list(values)
    rejected_count = 0
    center = None
    spread_deg = None

    if angle_key is not None and angle_outlier_deg is not None and angle_outlier_deg > 0:
        angle_values = [float(item[angle_key]) for item in values if angle_key in item]
        if angle_values:
            center = circular_median(angle_values)
            kept = [
                item
                for item in values
                if angle_key not in item or abs(math.degrees(angle_diff(float(item[angle_key]), center))) <= angle_outlier_deg
            ]
            rejected_count = len(values) - len(kept)

    if len(kept) < int(min_samples):
        return None

    lfsr_values = [item["lfsr_location"] for item in kept if "lfsr_location" in item]
    raw_angle_values = [float(item["raw_angle_rad"]) for item in kept if "raw_angle_rad" in item]
    calibrated_angle_values = [float(item["calibrated_angle_rad"]) for item in kept if "calibrated_angle_rad" in item]

    first = kept[0]
    summary = {
        "sensor": int(first["sensor"]),
        "basestation": int(first["basestation"]),
        "sweep": int(first["sweep"]),
        "sample_count": int(len(kept)),
        "raw_sample_count": int(len(values)),
        "rejected_count": int(rejected_count),
    }

    if lfsr_values:
        summary["lfsr_location"] = float(median(lfsr_values))
        summary["median_lfsr_location"] = float(median(lfsr_values))
    if raw_angle_values:
        raw_center = circular_median(raw_angle_values)
        summary["raw_angle_rad"] = raw_center
        spread_deg = circular_spread_deg(raw_angle_values, raw_center)
    if calibrated_angle_values:
        calibrated_center = circular_median(calibrated_angle_values)
        summary["calibrated_angle_rad"] = calibrated_center
        spread_deg = circular_spread_deg(calibrated_angle_values, calibrated_center)
    if spread_deg is not None:
        summary["angle_spread_deg"] = float(spread_deg)

    return summary


def summarize_observation_buffer(buffer, angle_outlier_deg=8.0, min_samples=1):
    summaries = []
    for key, values in sorted(buffer.items()):
        summary = summarize_observation_group(
            values,
            angle_outlier_deg=angle_outlier_deg,
            min_samples=min_samples,
        )
        if summary is not None:
            summaries.append(summary)
    return summaries


def observation_quality_counts(observations):
    sensors = {int(item["sensor"]) for item in observations}
    basestations = {int(item["basestation"]) for item in observations}
    channels = {
        (int(item["sensor"]), int(item["basestation"]), int(item["sweep"]))
        for item in observations
    }
    samples = sum(int(item.get("sample_count", 1)) for item in observations)
    rejected = sum(int(item.get("rejected_count", 0)) for item in observations)
    return {
        "sensors": len(sensors),
        "basestations": len(basestations),
        "channels": len(channels),
        "samples": samples,
        "rejected": rejected,
    }


def _cluster_spread(cluster, period=None):
    keys = [_frame_family_key(item) for item in cluster["frames"]]
    center0, center1 = cluster["center"]
    if not keys:
        return float("inf")
    if period is None:
        return median(max(abs(k[0] - center0), abs(k[1] - center1)) for k in keys)

    return median(
        max(
            _period_distance_ticks(k[0], center0, period),
            _period_distance_ticks(k[1], center1, period),
        )
        for k in keys
    )
