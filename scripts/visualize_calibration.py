from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

DIM_NAMES = ["x", "theta", "x_dot", "theta_dot"]
CALIBRATION_FILE_RE = re.compile(r"^calibration_(\d+)\.npy$")


def discover_calibration_sizes(calib_dir: Path) -> list[int]:
    """Discover all calibration_<N>.npy files currently present in the directory."""
    sizes: list[int] = []
    for path in calib_dir.glob("calibration_*.npy"):
        match = CALIBRATION_FILE_RE.match(path.name)
        if match:
            sizes.append(int(match.group(1)))
    return sorted(set(sizes))


def load_data(calib_dir: Path):
    candidates_path = calib_dir / "candidates.npy"
    if not candidates_path.is_file():
        raise FileNotFoundError(f"Missing candidates file: {candidates_path}")

    calib_sizes = discover_calibration_sizes(calib_dir)
    if not calib_sizes:
        raise FileNotFoundError(f"No calibration_<N>.npy files found in: {calib_dir}")

    data = {"candidates": np.load(candidates_path)}
    for size in calib_sizes:
        data[f"calib_{size}"] = np.load(calib_dir / f"calibration_{size}.npy")

    meta_path = calib_dir / "metadata.json"
    if meta_path.exists():
        with meta_path.open("r", encoding="utf-8") as f:
            meta = json.load(f)
    else:
        meta = {}

    return data, meta, calib_sizes


def validate_data(data: dict, calib_sizes: list[int]) -> None:
    candidates = data["candidates"]
    if candidates.ndim != 2 or candidates.shape[1] != len(DIM_NAMES):
        raise ValueError(f"Unexpected candidates shape: {candidates.shape}; expected [N, {len(DIM_NAMES)}]")

    for size in calib_sizes:
        arr = data[f"calib_{size}"]
        if arr.ndim != 2 or arr.shape[1] != len(DIM_NAMES):
            raise ValueError(
                f"Unexpected calibration_{size}.npy shape: {arr.shape}; expected [N, {len(DIM_NAMES)}]"
            )
        if len(arr) != size:
            raise ValueError(
                f"calibration_{size}.npy contains {len(arr)} rows; filename declares {size} rows"
            )


def choose_focus_size(calib_sizes: list[int], requested: int | None) -> int:
    """Use the requested size, or default to the largest currently available set."""
    if requested is None:
        return calib_sizes[-1]
    if requested not in calib_sizes:
        available = ", ".join(str(size) for size in calib_sizes)
        raise ValueError(f"--focus-size {requested} is unavailable. Available sizes: {available}")
    return requested


def print_stats(name: str, x: np.ndarray):
    print(f"\n{name}: shape={x.shape}")
    for i, dim_name in enumerate(DIM_NAMES):
        col = x[:, i]
        print(
            f"  {dim_name:10s} "
            f"min={col.min(): .5f}  "
            f"p01={np.percentile(col, 1): .5f}  "
            f"p50={np.percentile(col, 50): .5f}  "
            f"p99={np.percentile(col, 99): .5f}  "
            f"max={col.max(): .5f}"
        )


def robust_selection_space(candidates: np.ndarray, meta: dict):
    """Reconstruct the same 4-D space used by collect_calibration.py for FPS.

    Prefer robust_center / robust_scale / fps_clip saved in metadata.json.
    Fall back to recomputing them exactly as the collector does.
    """
    if all(key in meta for key in ("robust_center", "robust_scale", "fps_clip")):
        center = np.asarray(meta["robust_center"], dtype=np.float32)
        scale = np.asarray(meta["robust_scale"], dtype=np.float32)
        clip = float(meta["fps_clip"])
    else:
        center = np.median(candidates, axis=0).astype(np.float32)
        q25, q75 = np.percentile(candidates, [25.0, 75.0], axis=0)
        robust_scale = ((q75 - q25) / 1.349).astype(np.float32)
        std = candidates.std(axis=0).astype(np.float32)
        scale = np.where(robust_scale > 1.0e-8, robust_scale, std)
        scale = np.where(scale > 1.0e-8, scale, np.float32(1.0))
        clip = 3.0

    if center.shape != (4,) or scale.shape != (4,):
        raise ValueError(f"Unexpected robust center/scale shape: {center.shape}, {scale.shape}")

    return center, scale, clip


def transform_for_coverage(x: np.ndarray, center: np.ndarray, scale: np.ndarray, clip: float):
    z = (x.astype(np.float32, copy=False) - center) / scale
    return np.clip(z, -clip, clip).astype(np.float32, copy=False)


def nearest_distances(
    candidate_features: np.ndarray,
    calibration_features: np.ndarray,
    batch_size: int = 1024,
) -> np.ndarray:
    """Distance from every candidate to its nearest calibration sample."""
    result = np.empty(len(candidate_features), dtype=np.float32)

    for start in range(0, len(candidate_features), batch_size):
        end = min(start + batch_size, len(candidate_features))
        batch = candidate_features[start:end]
        diff = batch[:, None, :] - calibration_features[None, :, :]
        distance_sq = np.einsum("bkd,bkd->bk", diff, diff)
        result[start:end] = np.sqrt(distance_sq.min(axis=1))

    return result


def compute_coverage_metrics(data: dict, meta: dict, calib_sizes: list[int]):
    """Compute geometric coverage for every calibration set currently available."""
    candidates = data["candidates"]
    center, scale, clip = robust_selection_space(candidates, meta)
    candidate_features = transform_for_coverage(candidates, center, scale, clip)

    rows = []
    previous_p95 = None

    for size in calib_sizes:
        calib_features = transform_for_coverage(data[f"calib_{size}"], center, scale, clip)
        d = nearest_distances(candidate_features, calib_features)

        p95 = float(np.percentile(d, 95))
        row = {
            "size": size,
            "mean_nn": float(d.mean()),
            "median_nn": float(np.median(d)),
            "p95_nn": p95,
            "p99_nn": float(np.percentile(d, 99)),
            "max_nn": float(d.max()),
            "exactly_selected_fraction": float(np.mean(d < 1.0e-7)),
            "p95_reduction_vs_previous_pct": None,
        }

        if previous_p95 is not None and previous_p95 > 0:
            row["p95_reduction_vs_previous_pct"] = 100.0 * (previous_p95 - p95) / previous_p95

        previous_p95 = p95
        rows.append(row)

    return rows, {"center": center, "scale": scale, "clip": clip}


def print_coverage_table(rows):
    print("\nQuantitative coverage in robust-normalized 4-D observation space")
    print("Lower nearest-neighbor distance = better coverage")
    print("Primary metric: p95_nn; max_nn is the k-center coverage radius.\n")

    header = (
        f"{'K':>5}  {'mean':>10}  {'median':>10}  {'p95':>10}  "
        f"{'p99':>10}  {'max':>10}  {'p95 improvement':>16}"
    )
    print(header)
    print("-" * len(header))

    for row in rows:
        improvement = row["p95_reduction_vs_previous_pct"]
        improvement_str = "-" if improvement is None else f"{improvement:6.2f}%"
        print(
            f"{row['size']:5d}  "
            f"{row['mean_nn']:10.4f}  "
            f"{row['median_nn']:10.4f}  "
            f"{row['p95_nn']:10.4f}  "
            f"{row['p99_nn']:10.4f}  "
            f"{row['max_nn']:10.4f}  "
            f"{improvement_str:>16}"
        )


def save_coverage_metrics(rows, out_dir: Path):
    json_path = out_dir / "coverage_metrics.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)

    csv_path = out_dir / "coverage_metrics.csv"
    fieldnames = [
        "size",
        "mean_nn",
        "median_nn",
        "p95_nn",
        "p99_nn",
        "max_nn",
        "exactly_selected_fraction",
        "p95_reduction_vs_previous_pct",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_coverage_metrics(rows, out_dir: Path):
    sizes = [r["size"] for r in rows]
    means = [r["mean_nn"] for r in rows]
    p95 = [r["p95_nn"] for r in rows]
    maxima = [r["max_nn"] for r in rows]

    fig = plt.figure(figsize=(8, 6))
    plt.plot(sizes, means, marker="o", label="mean nearest distance")
    plt.plot(sizes, p95, marker="o", label="p95 nearest distance")
    plt.plot(sizes, maxima, marker="o", label="max nearest distance")
    plt.xlabel("calibration samples")
    plt.ylabel("distance in robust-normalized 4-D space")
    plt.title("quantitative calibration coverage")
    plt.xticks(sizes)
    plt.grid(alpha=0.25)
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / "coverage_metrics.png", dpi=150)
    plt.close(fig)


def plot_histograms(data, out_dir: Path, focus_size: int):
    candidates = data["candidates"]
    calib = data[f"calib_{focus_size}"]
    focus_label = f"calibration_{focus_size}"

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.ravel()

    for i, ax in enumerate(axes):
        ax.hist(candidates[:, i], bins=80, density=True, alpha=0.5, label="candidates")
        ax.hist(calib[:, i], bins=40, density=True, alpha=0.6, label=focus_label)
        ax.set_title(DIM_NAMES[i])
        ax.legend()

    fig.suptitle(f"1D distributions: candidates vs {focus_label}")
    fig.tight_layout()
    fig.savefig(out_dir / f"histograms_candidates_vs_calib{focus_size}.png", dpi=150)
    plt.close(fig)


def plot_scatter_overlay(data, out_dir: Path, focus_size: int):
    candidates = data["candidates"]
    calib = data[f"calib_{focus_size}"]
    focus_label = f"calibration_{focus_size}"

    fig = plt.figure(figsize=(7, 6))
    plt.scatter(candidates[:, 1], candidates[:, 3], s=7, alpha=0.22, color="tab:gray", label="candidates")
    plt.scatter(calib[:, 1], calib[:, 3], s=22, alpha=0.9, label=focus_label)
    plt.xlabel("theta [rad]")
    plt.ylabel("theta_dot [rad/s]")
    plt.title(f"theta vs theta_dot: candidates vs {focus_label}")
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / f"scatter_theta_theta_dot_calib{focus_size}.png", dpi=150)
    plt.close(fig)

    fig = plt.figure(figsize=(7, 6))
    plt.scatter(candidates[:, 0], candidates[:, 2], s=7, alpha=0.22, color="tab:gray", label="candidates")
    plt.scatter(calib[:, 0], calib[:, 2], s=22, alpha=0.9, label=focus_label)
    plt.xlabel("x [m]")
    plt.ylabel("x_dot [m/s]")
    plt.title(f"x vs x_dot: candidates vs {focus_label}")
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / f"scatter_x_x_dot_calib{focus_size}.png", dpi=150)
    plt.close(fig)


def plot_hexbin(data, out_dir: Path, focus_size: int):
    candidates = data["candidates"]
    calib = data[f"calib_{focus_size}"]
    focus_label = f"calibration_{focus_size}"

    fig = plt.figure(figsize=(7, 6))
    plt.hexbin(candidates[:, 1], candidates[:, 3], gridsize=45, mincnt=1)
    plt.scatter(calib[:, 1], calib[:, 3], s=12, alpha=0.8, label=focus_label)
    plt.xlabel("theta [rad]")
    plt.ylabel("theta_dot [rad/s]")
    plt.title(f"theta vs theta_dot density + {focus_label}")
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / f"hexbin_theta_theta_dot_calib{focus_size}.png", dpi=150)
    plt.close(fig)


def plot_compare_sizes(data, out_dir: Path, calib_sizes: list[int]):
    candidates = data["candidates"]

    fig = plt.figure(figsize=(8, 7))
    plt.scatter(
        candidates[:, 1],
        candidates[:, 3],
        s=8,
        alpha=0.28,
        color="tab:gray",
        label="candidates",
    )

    for size in calib_sizes:
        arr = data[f"calib_{size}"]
        plt.scatter(arr[:, 1], arr[:, 3], s=24, alpha=0.82, label=f"calibration_{size}")

    plt.xlabel("theta [rad]")
    plt.ylabel("theta_dot [rad/s]")
    plt.title("coverage comparison across available calibration sizes")
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / "compare_sizes_theta_theta_dot.png", dpi=150)
    plt.close(fig)


def plot_each_size(data, out_dir: Path, calib_sizes: list[int]):
    candidates = data["candidates"]

    for size in calib_sizes:
        key = f"calib_{size}"
        calib = data[key]

        fig = plt.figure(figsize=(8, 7))
        plt.scatter(
            candidates[:, 1],
            candidates[:, 3],
            s=8,
            alpha=0.28,
            color="tab:gray",
            label="candidates",
        )
        plt.scatter(calib[:, 1], calib[:, 3], s=26, alpha=0.9, label=f"calibration_{size}")
        plt.xlabel("theta [rad]")
        plt.ylabel("theta_dot [rad/s]")
        plt.title(f"candidates vs calibration_{size}")
        plt.legend()
        plt.tight_layout()
        fig.savefig(out_dir / f"calib_{size}_theta_theta_dot.png", dpi=150)
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize and quantify CartPole PTQ calibration coverage.")
    parser.add_argument("calib_dir", nargs="?", help="Directory containing candidates.npy and calibration_*.npy")
    parser.add_argument(
        "--focus-size",
        type=int,
        default=None,
        help="Calibration size for detailed histogram/scatter/hexbin plots. Default: largest available size.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.calib_dir:
        calib_dir = Path(args.calib_dir)
    else:
        calib_dir = Path(input("Enter calibration directory path: ").strip())

    out_dir = calib_dir / "viz"
    out_dir.mkdir(parents=True, exist_ok=True)

    data, meta, calib_sizes = load_data(calib_dir)
    validate_data(data, calib_sizes)
    focus_size = choose_focus_size(calib_sizes, args.focus_size)

    print(f"Detected calibration sizes: {calib_sizes}")
    print(f"Detailed plot focus: calibration_{focus_size}")
    print("metadata:")
    print(json.dumps(meta, indent=2, ensure_ascii=False))

    print_stats("candidates", data["candidates"])
    for size in calib_sizes:
        print_stats(f"calib_{size}", data[f"calib_{size}"])

    rows, coverage_space = compute_coverage_metrics(data, meta, calib_sizes)
    print_coverage_table(rows)
    print(
        "\nCoverage space: robust center="
        f"{coverage_space['center'].tolist()}, scale={coverage_space['scale'].tolist()}, "
        f"clip=+/-{coverage_space['clip']}"
    )

    save_coverage_metrics(rows, out_dir)
    plot_coverage_metrics(rows, out_dir)
    plot_histograms(data, out_dir, focus_size)
    plot_scatter_overlay(data, out_dir, focus_size)
    plot_hexbin(data, out_dir, focus_size)
    plot_compare_sizes(data, out_dir, calib_sizes)
    plot_each_size(data, out_dir, calib_sizes)

    print(f"\nSaved visualizations and coverage metrics to: {out_dir}")


if __name__ == "__main__":
    main()
