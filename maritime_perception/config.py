"""
config.py

Configuration loader with validation.
Loads default.yaml and vessel_profile.yaml, merges them,
and provides typed accessor helpers.

Design: fail fast on startup with clear errors rather than
silently using wrong defaults at sea.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import yaml

log = logging.getLogger(__name__)


def load_config(
    default_path   : str | Path = "configs/default.yaml",
    profile_path   : str | Path = "configs/vessel_profile.yaml",
) -> dict[str, Any]:
    """
    Load and merge both config files.
    Returns a single dict with keys:
        config["lidar"], config["preprocessing"], config["noise_filter"],
        config["segmentation"], config["extraction"], config["tracking"],
        config["fusion"], config["interfaces"], config["health"],
        config["runtime"], config["vessel"], config["lidar_mounting"],
        config["fov_mask"], config["imu_mounting"], config["rtk_mounting"]
    """
    default = _load_yaml(default_path)
    profile = _load_yaml(profile_path)

    # merge: profile keys are top-level additions, no overlap
    merged = {**default, **profile}

    _validate(merged)
    log.info("Configuration loaded from %s and %s", default_path, profile_path)
    return merged


def _load_yaml(path: str | Path) -> dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"Config file not found: {p.resolve()}\n"
            f"Run from the project root directory."
        )
    with open(p, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Config file {p} must contain a YAML mapping.")
    return data


def _validate(cfg: dict) -> None:
    """Fail fast on obviously wrong configuration."""
    errors = []

    pp = cfg.get("preprocessing", {})
    if pp.get("range_min_m", 0) <= 0:
        errors.append("preprocessing.range_min_m must be > 0")
    if pp.get("range_max_m", 0) <= pp.get("range_min_m", 0):
        errors.append("preprocessing.range_max_m must be > range_min_m")

    nf = cfg.get("noise_filter", {})
    if nf.get("min_hits_base", 0) < 1:
        errors.append("noise_filter.min_hits_base must be >= 1")
    if nf.get("min_hits_max", 0) < nf.get("min_hits_base", 0):
        errors.append("noise_filter.min_hits_max must be >= min_hits_base")

    tr = cfg.get("tracking", {})
    if tr.get("min_hits_confirm", 0) < 1:
        errors.append("tracking.min_hits_confirm must be >= 1")

    if errors:
        raise ValueError(
            "Configuration validation failed:\n" +
            "\n".join(f"  - {e}" for e in errors)
        )


def get(cfg: dict, *keys: str, default: Any = None) -> Any:
    """
    Safe nested key access.
    get(cfg, "tracking", "kalman", "process_noise_pos", default=0.1)
    """
    node = cfg
    for key in keys:
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return node