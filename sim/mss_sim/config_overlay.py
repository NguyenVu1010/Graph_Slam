"""Merge a per-scenario pipeline-config overlay onto config/default.yaml (ADR-011 section 5)."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


def merge_overlay(base: dict, overlay: dict, _prefix: str = "") -> dict:
    """Deep-merge `overlay` onto `base` and return a new dict; overlay values win.

    The overlay may not introduce keys absent from `base`, except under a parent
    that is `null` in `base`. An unknown key raises KeyError naming its dotted path.
    """
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        path = f"{_prefix}{key}"
        if key not in merged:
            raise KeyError(f"{path}: unknown key (not in base config)")
        current = merged[key]
        if isinstance(value, dict) and isinstance(current, dict):
            merged[key] = merge_overlay(current, value, path + ".")
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _parameters(doc: Any, source: Path) -> dict:
    try:
        params = doc["mss_node"]["ros__parameters"]
    except (KeyError, TypeError):
        raise KeyError(f"{source}: missing mss_node.ros__parameters") from None
    if not isinstance(params, dict):
        raise KeyError(f"{source}: mss_node.ros__parameters is not a mapping")
    return params


def load_pipeline_config(default: Path, overlay: Path) -> dict:
    """Return `mss_node.ros__parameters` of `default` with `overlay` merged on top."""
    base = _parameters(yaml.safe_load(Path(default).read_text()), Path(default))
    over = _parameters(yaml.safe_load(Path(overlay).read_text()), Path(overlay))
    return merge_overlay(base, over)


def get(cfg: dict, dotted: str) -> Any:
    """Look up a dotted path such as `imu.gyro_noise_density`; KeyError names the path."""
    node: Any = cfg
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(f"{dotted}: not found in config")
        node = node[part]
    return node
