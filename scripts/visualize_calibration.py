from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter

DIM_NAMES = ["x", "theta", "x_dot", "theta_dot"]
CALIBRATION_FILE_RE = re.compile(r"^calibration_(\d+)\.npy$")

CANDIDATE_REGION_NAMES = {
    0: "core",
    1: "controlled_tail",
    2: "extreme",
}
SELECTION_ROLE_NAMES = {
    0: "representative_core",
    1: "core_coverage",
    2: "controlled_tail",
}


def discover_calibration_files(
    calib_dir: Path, meta: dict
) -> tuple[dict[int, Path], str, list[str]]:
    """Resolve the calibration sets belonging to the current run.

    ``metadata.json`` is authoritative when it contains ``calibration_sets``.
    This prevents stale ``calibration_<N>.npy`` files left by an older run from
    silently entering the current comparison. Directory scanning is only a
    fallback for calibration directories without that metadata field.
    """
    metadata_sets = meta.get("calibration_sets")
    if metadata_sets:
        if not isinstance(metadata_sets, dict):
            raise ValueError('metadata["calibration_sets"] must be an object mapping size to filename')

        files: dict[int, Path] = {}
        for raw_size, raw_filename in metadata_sets.items():
            try:
                size = int(raw_size)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid calibration size in metadata: {raw_size!r}") from exc
            if size <= 0:
                raise ValueError(f"Calibration size in metadata must be positive: {size}")
            if size in files:
                raise ValueError(f"Duplicate calibration size in metadata: {size}")
            if not isinstance(raw_filename, str) or not raw_filename:
                raise ValueError(f"Invalid calibration filename for size {size}: {raw_filename!r}")

            path = Path(raw_filename)
            if not path.is_absolute():
                path = calib_dir / path
            if not path.is_file():
                raise FileNotFoundError(
                    f"metadata.json references missing calibration file for size {size}: {path}"
                )
            files[size] = path

        active_paths = {path.resolve() for path in files.values()}
        ignored_stale = sorted(
            path.name
            for path in calib_dir.glob("calibration_*.npy")
            if CALIBRATION_FILE_RE.match(path.name) and path.resolve() not in active_paths
        )
        return dict(sorted(files.items())), 'metadata["calibration_sets"]', ignored_stale

    files = {}
    for path in calib_dir.glob("calibration_*.npy"):
        match = CALIBRATION_FILE_RE.match(path.name)
        if match:
            files[int(match.group(1))] = path

    if not files:
        raise FileNotFoundError(f"No calibration_<N>.npy files found in: {calib_dir}")

    return dict(sorted(files.items())), "directory scan", []


def load_data(calib_dir: Path):
    candidates_path = calib_dir / "candidates.npy"
    if not candidates_path.is_file():
        raise FileNotFoundError(f"Missing candidates file: {candidates_path}")

    meta_path = calib_dir / "metadata.json"
    if meta_path.exists():
        with meta_path.open("r", encoding="utf-8") as f:
            meta = json.load(f)
    else:
        meta = {}

    calibration_files, calibration_source, ignored_stale = discover_calibration_files(calib_dir, meta)
    calib_sizes = list(calibration_files)

    data = {"candidates": np.load(candidates_path)}
    for size, path in calibration_files.items():
        data[f"calib_{size}"] = np.load(path)

    return data, meta, calib_sizes, calibration_source, ignored_stale


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


def compute_robust_radius(candidates: np.ndarray, meta: dict) -> np.ndarray:
    """Recompute the un-clipped 4-D robust radius used for core/tail/extreme classification."""
    center, scale, _ = robust_selection_space(candidates, meta)
    robust_z = (candidates.astype(np.float32, copy=False) - center) / scale
    return np.sqrt(np.einsum("ij,ij->i", robust_z, robust_z)).astype(np.float32, copy=False)


def load_selection_diagnostics(
    calib_dir: Path,
    data: dict,
    meta: dict,
    calib_sizes: list[int],
) -> dict:
    """Load and validate the Hybrid-v2 region/role information for the current run.

    Candidate classification is always available: use ``robust_radius.npy`` when
    present, otherwise recompute the exact un-clipped robust radius. Calibration
    role diagnostics additionally require the current run's
    ``selection_indices_<max>.npy`` and ``selection_roles_<max>.npy`` files.
    """
    candidates = data["candidates"]

    radius_path = calib_dir / "robust_radius.npy"
    if radius_path.is_file():
        radius = np.load(radius_path).astype(np.float32, copy=False)
        radius_source = radius_path.name
        if radius.shape != (len(candidates),):
            raise ValueError(
                f"Unexpected {radius_path.name} shape: {radius.shape}; expected ({len(candidates)},)"
            )
        if not np.isfinite(radius).all():
            raise ValueError(f"{radius_path.name} contains NaN/Inf")
    else:
        radius = compute_robust_radius(candidates, meta)
        radius_source = "recomputed from candidates"

    radius_p95 = float(meta.get("radius_p95", np.percentile(radius, 95.0)))
    radius_p995 = float(meta.get("radius_p995", np.percentile(radius, 99.5)))
    if not np.isfinite(radius_p95) or not np.isfinite(radius_p995) or radius_p995 < radius_p95:
        raise ValueError(
            f"Invalid robust-radius thresholds: P95={radius_p95}, P99.5={radius_p995}"
        )

    candidate_regions = np.zeros(len(candidates), dtype=np.uint8)
    candidate_regions[(radius > radius_p95) & (radius <= radius_p995)] = 1
    candidate_regions[radius > radius_p995] = 2

    region_counts = {
        CANDIDATE_REGION_NAMES[code]: int(np.count_nonzero(candidate_regions == code))
        for code in sorted(CANDIDATE_REGION_NAMES)
    }

    max_size = calib_sizes[-1]
    indices_path = calib_dir / f"selection_indices_{max_size}.npy"
    roles_path = calib_dir / f"selection_roles_{max_size}.npy"

    selection_indices = None
    selection_roles = None
    selection_source = None

    if indices_path.is_file() and roles_path.is_file():
        selection_indices = np.load(indices_path).astype(np.int64, copy=False)
        selection_roles = np.load(roles_path).astype(np.uint8, copy=False)
        selection_source = f"{indices_path.name} + {roles_path.name}"

        if selection_indices.shape != (max_size,):
            raise ValueError(
                f"Unexpected {indices_path.name} shape: {selection_indices.shape}; expected ({max_size},)"
            )
        if selection_roles.shape != (max_size,):
            raise ValueError(
                f"Unexpected {roles_path.name} shape: {selection_roles.shape}; expected ({max_size},)"
            )
        if np.any(selection_indices < 0) or np.any(selection_indices >= len(candidates)):
            raise ValueError(f"{indices_path.name} contains out-of-range candidate indices")
        if len(np.unique(selection_indices)) != max_size:
            raise ValueError(f"{indices_path.name} contains duplicate candidate indices")

        unknown_roles = sorted(set(np.unique(selection_roles).tolist()) - set(SELECTION_ROLE_NAMES))
        if unknown_roles:
            raise ValueError(f"{roles_path.name} contains unknown role codes: {unknown_roles}")

        # Verify that the selection files really belong to the active calibration files,
        # rather than to a stale run left in the same directory.
        for size in calib_sizes:
            expected = candidates[selection_indices[:size]]
            actual = data[f"calib_{size}"]
            if not np.array_equal(expected, actual):
                raise ValueError(
                    f"selection_indices_{max_size}.npy does not reproduce calibration_{size}.npy; "
                    "the selection diagnostics appear to belong to a different run"
                )

        # Re-check the collector invariants from persisted data.
        selected_regions = candidate_regions[selection_indices]
        bad_representative = int(np.count_nonzero((selection_roles == 0) & (selected_regions != 0)))
        bad_core_coverage = int(np.count_nonzero((selection_roles == 1) & (selected_regions != 0)))
        bad_tail = int(np.count_nonzero((selection_roles == 2) & (selected_regions != 1)))
        selected_extreme = int(np.count_nonzero(selected_regions == 2))
        invariant_violations = {
            "representative_outside_core": bad_representative,
            "core_coverage_outside_core": bad_core_coverage,
            "controlled_tail_outside_tail": bad_tail,
            "selected_extreme": selected_extreme,
        }
    elif indices_path.exists() or roles_path.exists():
        raise FileNotFoundError(
            "Incomplete selection diagnostics: expected both "
            f"{indices_path.name} and {roles_path.name}"
        )
    else:
        invariant_violations = None

    return {
        "radius": radius,
        "radius_source": radius_source,
        "radius_p95": radius_p95,
        "radius_p995": radius_p995,
        "candidate_regions": candidate_regions,
        "region_counts": region_counts,
        "selection_indices": selection_indices,
        "selection_roles": selection_roles,
        "selection_source": selection_source,
        "invariant_violations": invariant_violations,
        "max_size": max_size,
    }


def print_selection_diagnostics(diagnostics: dict, focus_size: int) -> None:
    regions = diagnostics["candidate_regions"]
    total = len(regions)
    print("\nHybrid-v2 selection diagnostics")
    print(
        f"Candidate radius source: {diagnostics['radius_source']}; "
        f"P95={diagnostics['radius_p95']:.6g}, P99.5={diagnostics['radius_p995']:.6g}"
    )
    for code in sorted(CANDIDATE_REGION_NAMES):
        name = CANDIDATE_REGION_NAMES[code]
        count = diagnostics["region_counts"][name]
        print(f"  candidate {name:16s}: {count:6d}  ({count / total:6.2%})")

    indices = diagnostics["selection_indices"]
    roles = diagnostics["selection_roles"]
    if indices is None or roles is None:
        print("  calibration roles: unavailable (selection_indices/selection_roles files not found)")
        return

    prefix_roles = roles[:focus_size]
    print(f"Calibration role prefix: first {focus_size} entries from {diagnostics['selection_source']}")
    for code in sorted(SELECTION_ROLE_NAMES):
        count = int(np.count_nonzero(prefix_roles == code))
        print(f"  role {SELECTION_ROLE_NAMES[code]:20s}: {count:6d}  ({count / focus_size:6.2%})")

    violations = diagnostics["invariant_violations"]
    if violations is not None:
        total_violations = sum(violations.values())
        print(f"  selection invariant violations: {total_violations}")
        if total_violations:
            print(f"    {violations}")


def save_selection_diagnostics(diagnostics: dict, focus_size: int, out_dir: Path) -> None:
    regions = diagnostics["candidate_regions"]
    total = len(regions)
    payload = {
        "radius_source": diagnostics["radius_source"],
        "radius_p95": diagnostics["radius_p95"],
        "radius_p995": diagnostics["radius_p995"],
        "candidate_region_counts": diagnostics["region_counts"],
        "candidate_region_fractions": {
            name: count / total for name, count in diagnostics["region_counts"].items()
        },
        "focus_size": focus_size,
        "selection_source": diagnostics["selection_source"],
        "selection_invariant_violations": diagnostics["invariant_violations"],
    }

    roles = diagnostics["selection_roles"]
    if roles is not None:
        prefix = roles[:focus_size]
        payload["focus_role_counts"] = {
            SELECTION_ROLE_NAMES[code]: int(np.count_nonzero(prefix == code))
            for code in sorted(SELECTION_ROLE_NAMES)
        }

    with (out_dir / "selection_diagnostics.json").open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


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


def plot_histograms(
    data,
    out_dir: Path,
    focus_size: int,
    *,
    bins: int = 80,
    tail_percentile: float = 0.5,
):
    """Compare central 1-D distributions using directly comparable histograms.

    Both datasets use the exact same bin edges, derived from the candidate
    P<tail_percentile> .. P<100-tail_percentile> interval. The y-axis is the
    fraction of each *entire* dataset that falls in a bin, so equal bar heights
    have a direct probabilistic interpretation. Samples outside the displayed
    central interval are intentionally omitted from the bars and their retained
    mass is reported below each subplot.
    """
    if bins <= 0:
        raise ValueError("Histogram bin count must be > 0")
    if not 0.0 <= tail_percentile < 50.0:
        raise ValueError("Histogram tail percentile must satisfy 0 <= value < 50")

    candidates = data["candidates"]
    calib = data[f"calib_{focus_size}"]
    focus_label = f"calibration_{focus_size}"
    upper_percentile = 100.0 - tail_percentile

    candidate_weights = np.full(len(candidates), 1.0 / len(candidates), dtype=np.float64)
    calib_weights = np.full(len(calib), 1.0 / len(calib), dtype=np.float64)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes = axes.ravel()

    print(
        "[INFO] Histogram comparison: "
        f"shared bins={bins}, candidate P{tail_percentile:g}-P{upper_percentile:g} range, "
        "y=fraction of all samples per bin"
    )

    for i, ax in enumerate(axes):
        candidate_col = candidates[:, i]
        calib_col = calib[:, i]

        low, high = np.percentile(candidate_col, [tail_percentile, upper_percentile])
        low = float(low)
        high = float(high)

        # Degenerate dimensions are rare for this task, but keep the visualization
        # well-defined if a future observation dimension is effectively constant.
        if not np.isfinite(low) or not np.isfinite(high):
            raise ValueError(f"Non-finite histogram range for {DIM_NAMES[i]}: [{low}, {high}]")
        if high <= low:
            center = low
            pad = max(abs(center) * 1.0e-6, 1.0e-6)
            low, high = center - pad, center + pad

        edges = np.linspace(low, high, bins + 1, dtype=np.float64)
        candidate_mass = float(np.mean((candidate_col >= low) & (candidate_col <= high)))
        calib_mass = float(np.mean((calib_col >= low) & (calib_col <= high)))

        ax.hist(
            candidate_col,
            bins=edges,
            weights=candidate_weights,
            alpha=0.5,
            label="candidates",
        )
        ax.hist(
            calib_col,
            bins=edges,
            weights=calib_weights,
            alpha=0.6,
            label=focus_label,
        )
        ax.set_xlim(low, high)
        ax.set_title(f"{DIM_NAMES[i]}  (candidate P{tail_percentile:g}-P{upper_percentile:g})")
        ax.set_ylabel("fraction of all samples / bin")
        ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0, decimals=1))
        ax.set_xlabel(
            f"displayed mass: candidates {candidate_mass:.1%}, {focus_label} {calib_mass:.1%}\n"
            f"full range: candidates [{candidate_col.min():.4g}, {candidate_col.max():.4g}], "
            f"{focus_label} [{calib_col.min():.4g}, {calib_col.max():.4g}]",
            fontsize=8,
        )
        ax.legend()

    fig.suptitle(
        f"1D central distributions: candidates vs {focus_label} "
        f"(shared {bins} bins; fraction per bin)"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_dir / f"histograms_candidates_vs_calib{focus_size}.png", dpi=150)
    plt.close(fig)


def plot_candidate_region_diagnostics(data: dict, diagnostics: dict, out_dir: Path) -> None:
    """Show which candidate states are core, controlled-tail, or extreme in theta space."""
    candidates = data["candidates"]
    regions = diagnostics["candidate_regions"]
    total = len(candidates)

    fig = plt.figure(figsize=(8, 7))
    styles = {
        0: {"s": 8, "alpha": 0.22, "color": "tab:gray"},
        1: {"s": 18, "alpha": 0.65, "color": "tab:orange"},
        2: {"s": 26, "alpha": 0.85, "color": "tab:red"},
    }
    for code in (0, 1, 2):
        mask = regions == code
        count = int(np.count_nonzero(mask))
        name = CANDIDATE_REGION_NAMES[code]
        plt.scatter(
            candidates[mask, 1],
            candidates[mask, 3],
            label=f"{name} ({count}, {count / total:.1%})",
            **styles[code],
        )

    plt.xlabel("theta [rad]")
    plt.ylabel("theta_dot [rad/s]")
    plt.title(
        "candidate regions by un-clipped 4-D robust radius\n"
        f"core <= P95 ({diagnostics['radius_p95']:.3g}), "
        f"tail <= P99.5 ({diagnostics['radius_p995']:.3g}), extreme > P99.5"
    )
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / "candidate_regions_theta_theta_dot.png", dpi=150)
    plt.close(fig)


def plot_calibration_role_diagnostics(
    data: dict,
    diagnostics: dict,
    out_dir: Path,
    focus_size: int,
) -> None:
    """Overlay Hybrid-v2 selection roles for the requested calibration prefix."""
    indices = diagnostics["selection_indices"]
    roles = diagnostics["selection_roles"]
    if indices is None or roles is None:
        print(
            "[WARN] Skipping calibration-role diagnostic plot because selection index/role files "
            "are unavailable."
        )
        return

    candidates = data["candidates"]
    prefix_indices = indices[:focus_size]
    prefix_roles = roles[:focus_size]

    fig = plt.figure(figsize=(8, 7))
    plt.scatter(
        candidates[:, 1],
        candidates[:, 3],
        s=7,
        alpha=0.14,
        color="tab:gray",
        label="all candidates",
    )

    styles = {
        0: {"s": 15, "alpha": 0.55, "color": "tab:blue", "marker": "o"},
        1: {"s": 34, "alpha": 0.9, "color": "tab:green", "marker": "^"},
        2: {"s": 38, "alpha": 0.9, "color": "tab:orange", "marker": "D"},
    }
    for code in (0, 1, 2):
        role_mask = prefix_roles == code
        selected = prefix_indices[role_mask]
        count = len(selected)
        plt.scatter(
            candidates[selected, 1],
            candidates[selected, 3],
            label=f"{SELECTION_ROLE_NAMES[code]} ({count})",
            **styles[code],
        )

    selected_regions = diagnostics["candidate_regions"][prefix_indices]
    selected_extreme = int(np.count_nonzero(selected_regions == 2))
    plt.xlabel("theta [rad]")
    plt.ylabel("theta_dot [rad/s]")
    plt.title(
        f"calibration_{focus_size} by Hybrid-v2 selection role\n"
        f"selected extreme candidates: {selected_extreme}"
    )
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_dir / f"calibration_roles_theta_theta_dot_calib{focus_size}.png", dpi=150)
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
    parser.add_argument(
        "--hist-bins",
        type=int,
        default=80,
        help="Number of shared bins in each 1-D histogram. Default: 80.",
    )
    parser.add_argument(
        "--hist-tail-percentile",
        type=float,
        default=0.5,
        help=(
            "Candidate percentile excluded from each tail in the detailed histograms. "
            "Default: 0.5, so the displayed range is P0.5-P99.5."
        ),
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

    data, meta, calib_sizes, calibration_source, ignored_stale = load_data(calib_dir)
    validate_data(data, calib_sizes)
    focus_size = choose_focus_size(calib_sizes, args.focus_size)
    selection_diagnostics = load_selection_diagnostics(calib_dir, data, meta, calib_sizes)

    print(f"Detected calibration sizes: {calib_sizes} (source: {calibration_source})")
    if ignored_stale:
        print(f"Ignored stale calibration files: {ignored_stale}")
    print(f"Detailed plot focus: calibration_{focus_size}")
    print("metadata:")
    print(json.dumps(meta, indent=2, ensure_ascii=False))

    print_stats("candidates", data["candidates"])
    for size in calib_sizes:
        print_stats(f"calib_{size}", data[f"calib_{size}"])

    print_selection_diagnostics(selection_diagnostics, focus_size)

    rows, coverage_space = compute_coverage_metrics(data, meta, calib_sizes)
    print_coverage_table(rows)
    print(
        "\nCoverage space: robust center="
        f"{coverage_space['center'].tolist()}, scale={coverage_space['scale'].tolist()}, "
        f"clip=+/-{coverage_space['clip']}"
    )

    save_coverage_metrics(rows, out_dir)
    save_selection_diagnostics(selection_diagnostics, focus_size, out_dir)
    plot_coverage_metrics(rows, out_dir)
    plot_candidate_region_diagnostics(data, selection_diagnostics, out_dir)
    plot_calibration_role_diagnostics(data, selection_diagnostics, out_dir, focus_size)
    plot_histograms(
        data,
        out_dir,
        focus_size,
        bins=args.hist_bins,
        tail_percentile=args.hist_tail_percentile,
    )
    plot_scatter_overlay(data, out_dir, focus_size)
    plot_hexbin(data, out_dir, focus_size)
    plot_compare_sizes(data, out_dir, calib_sizes)
    plot_each_size(data, out_dir, calib_sizes)

    print(f"\nSaved visualizations and coverage metrics to: {out_dir}")


if __name__ == "__main__":
    main()
