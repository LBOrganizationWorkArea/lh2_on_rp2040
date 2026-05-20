import argparse

import numpy as np

from wand_common import load_json, save_json


def feature_from_observation(frame, sensor, basestations):
    observations = frame.get("observations", {})
    values = []
    for bs in basestations:
        item = observations.get(str(bs), {}).get(str(sensor))
        if item is None:
            return None
        values.extend([float(item["lfsr0"]), float(item["lfsr1"])])
    return values


def design_matrix(raw_features, mean, scale):
    x = (np.asarray(raw_features, dtype=float) - mean) / scale
    return np.column_stack([np.ones(len(x)), x])


def normalized(raw_features, mean, scale):
    return (np.asarray(raw_features, dtype=float) - mean) / scale


def rbf_kernel(a, b, epsilon):
    diff = a[:, None, :] - b[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    return np.exp(-dist2 / max(epsilon * epsilon, 1e-9))


def fit_affine(raw, target, ridge):
    mean = raw.mean(axis=0)
    scale = raw.std(axis=0)
    scale[scale < 1.0] = 1.0
    A = design_matrix(raw, mean, scale)
    reg = np.eye(A.shape[1]) * ridge
    reg[0, 0] = 0.0
    coeff = np.linalg.solve(A.T @ A + reg, A.T @ target)
    pred = A @ coeff
    return {
        "model": "affine_lfsr_floor2d",
        "mean": mean,
        "scale": scale,
        "coefficients": coeff,
        "pred": pred,
    }


def fit_rbf(raw, target, ridge, epsilon=None):
    mean = raw.mean(axis=0)
    scale = raw.std(axis=0)
    scale[scale < 1.0] = 1.0
    centers = normalized(raw, mean, scale)

    if epsilon is None:
        distances = []
        for i in range(len(centers)):
            d = np.linalg.norm(centers[i] - centers[np.arange(len(centers)) != i], axis=1)
            distances.append(float(np.min(d)))
        epsilon = max(float(np.median(distances)) * 1.75, 0.25)

    K = rbf_kernel(centers, centers, epsilon)
    weights = np.linalg.solve(K + np.eye(len(K)) * ridge, target)
    pred = K @ weights
    return {
        "model": "rbf_lfsr_floor2d",
        "mean": mean,
        "scale": scale,
        "centers": centers,
        "epsilon": float(epsilon),
        "weights": weights,
        "pred": pred,
    }


def predict_fit(fit, raw_features):
    if fit["model"] == "affine_lfsr_floor2d":
        return design_matrix(raw_features, fit["mean"], fit["scale"]) @ fit["coefficients"]
    x = normalized(raw_features, fit["mean"], fit["scale"])
    K = rbf_kernel(x, fit["centers"], fit["epsilon"])
    return K @ fit["weights"]


def serialize_fit(fit):
    out = {
        "model": fit["model"],
        "mean": fit["mean"].tolist(),
        "scale": fit["scale"].tolist(),
    }
    if fit["model"] == "affine_lfsr_floor2d":
        out["coefficients"] = fit["coefficients"].tolist()
    else:
        out["centers"] = fit["centers"].tolist()
        out["epsilon"] = fit["epsilon"]
        out["weights"] = fit["weights"].tolist()
    return out


def main():
    parser = argparse.ArgumentParser(description="Fit a direct floor-only 2D LH2 calibration.")
    parser.add_argument("--input", default="data/angle3d_calibration_floor9.json")
    parser.add_argument("--output", default="config/floor2d_calibration.json")
    parser.add_argument("--sensor", type=int, default=2)
    parser.add_argument("--basestations", default="4,10")
    parser.add_argument("--ridge", type=float, default=1e-6)
    parser.add_argument("--model", choices=("rbf", "affine"), default="rbf")
    parser.add_argument("--epsilon", type=float, default=None)
    args = parser.parse_args()

    record = load_json(args.input)
    basestations = [int(x) for x in args.basestations.split(",")]

    features = []
    targets = []
    names = []
    for frame in record.get("frames", []):
        feat = feature_from_observation(frame, args.sensor, basestations)
        if feat is None:
            continue
        pose = frame["pose"]
        features.append(feat)
        targets.append([float(pose["x_m"]), float(pose["y_m"])])
        names.append(pose.get("name", str(len(names))))

    if len(features) < 4:
        raise ValueError("Need at least 4 usable floor points for a 2D affine fit.")

    raw = np.asarray(features, dtype=float)
    target = np.asarray(targets, dtype=float)
    if args.model == "affine":
        fit = fit_affine(raw, target, args.ridge)
    else:
        fit = fit_rbf(raw, target, args.ridge, args.epsilon)
    pred = fit["pred"]
    err = np.linalg.norm(pred - target, axis=1)

    loo_errors = []
    if len(features) >= 5:
        for i in range(len(features)):
            keep = np.ones(len(features), dtype=bool)
            keep[i] = False
            if args.model == "affine":
                fit_i = fit_affine(raw[keep], target[keep], args.ridge)
            else:
                fit_i = fit_rbf(raw[keep], target[keep], args.ridge, args.epsilon)
            pred_i = predict_fit(fit_i, raw[i:i + 1])
            loo_errors.append(float(np.linalg.norm(pred_i[0] - target[i])))

    fit_output = serialize_fit(fit)
    output = {
        "description": "Direct floor-only 2D LH2 calibration. Valid near the captured floor region.",
        "model": fit_output.pop("model"),
        "input": args.input,
        "sensor": args.sensor,
        "basestations": basestations,
        "feature_order": [f"bs{bs}_lfsr{sweep}" for bs in basestations for sweep in (0, 1)],
        "train_error_m": {
            "median": float(np.median(err)),
            "mean": float(np.mean(err)),
            "max": float(np.max(err)),
        },
        "points": [
            {
                "name": name,
                "ref": target[i].tolist(),
                "est": pred[i].tolist(),
                "err_m": float(err[i]),
            }
            for i, name in enumerate(names)
        ],
    }
    output.update(fit_output)
    if loo_errors:
        output["leave_one_out_error_m"] = {
            "median": float(np.median(loo_errors)),
            "mean": float(np.mean(loo_errors)),
            "max": float(np.max(loo_errors)),
        }

    save_json(args.output, output)

    print("=" * 70)
    print("Fit direct floor 2D calibration")
    print(f"Input: {args.input}")
    print(f"Output: {args.output}")
    print(f"model={output['model']} sensor={args.sensor} points={len(features)} basestations={basestations}")
    if output["model"] == "rbf_lfsr_floor2d":
        print(f"epsilon={output['epsilon']:.3f}")
    print(
        f"train_err median={np.median(err):.3f} m "
        f"mean={np.mean(err):.3f} m max={np.max(err):.3f} m"
    )
    if loo_errors:
        print(
            f"leave_one_out_err median={np.median(loo_errors):.3f} m "
            f"mean={np.mean(loo_errors):.3f} m max={np.max(loo_errors):.3f} m"
        )
    for item in output["points"]:
        print(
            f"  {item['name']}: err={item['err_m']:.3f} m "
            f"est=({item['est'][0]:+.3f},{item['est'][1]:+.3f}) "
            f"ref=({item['ref'][0]:+.3f},{item['ref'][1]:+.3f})"
        )
    print("=" * 70)


if __name__ == "__main__":
    main()
