"""Experiment spec: what one explorer run changes, its YAML form and its cache key.

YAML keys: ``scenario``, ``recording`` (paths; relative ones are from the repository root),
``variant`` (null or a key of the scenario's ``pipeline_config_variants``: the overlay the factors
start from),
``factors`` (``gnss``, ``wheel_odom``, ``loop_closure``: true, false or null = the scenario's
overlay), ``noise`` (multipliers, mss_sim.noise_scale), ``faults`` (list of ``sensor``, ``kind``,
``t_start_s``, ``t_end_s``, optional ``magnitude``; mss_sim.faults), ``seed`` (null = the
scenario's) and ``label`` (display only, not part of the key).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml

from mss_sim.faults import Fault, validate_fault
from mss_sim.noise_scale import NoiseScale, validate_scale
from mss_sim.scenario import load_scenario

REPO = Path(__file__).resolve().parents[2]
FACTORS = ("gnss", "wheel_odom", "loop_closure")
_KEYS = {"scenario", "recording", "variant", "factors", "noise", "faults", "seed", "label"}
_FAULT_KEYS = {"sensor", "kind", "t_start_s", "t_end_s", "magnitude"}


class SpecError(ValueError):
    """An experiment spec that cannot run; the message names the field."""


@dataclass(frozen=True)
class ExperimentSpec:
    scenario: Path
    recording: Path
    factors: dict[str, bool | None] = field(default_factory=lambda: dict.fromkeys(FACTORS))
    noise: NoiseScale = NoiseScale()
    faults: tuple[Fault, ...] = ()
    seed: int | None = None
    label: str = ""
    variant: str | None = None  # a key of the scenario's pipeline_config_variants


def _path(value: Any, name: str) -> Path:
    if not isinstance(value, str) or not value:
        raise SpecError(f"{name}: a path is required")
    p = Path(value)
    return p if p.is_absolute() else REPO / p


def _rel(p: Path) -> str:
    try:
        return p.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return p.resolve().as_posix()


def _mapping(value: Any, name: str, allowed: set[str] | tuple[str, ...]) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise SpecError(f"{name}: a mapping is required")
    unknown = sorted(set(value) - set(allowed))
    if unknown:
        raise SpecError(f"{name}.{unknown[0]}: unknown key (allowed: {sorted(allowed)})")
    return value


def spec_from_dict(doc: Any) -> ExperimentSpec:
    """Validated spec (faults get their default magnitudes); ``SpecError`` names the field."""
    doc = _mapping(doc, "spec", _KEYS)
    scenario = _path(doc.get("scenario"), "scenario")
    try:
        sc = load_scenario(scenario)
    except (OSError, ValueError) as e:
        raise SpecError(f"scenario: {e}") from e
    duration_s = sc.duration_s
    variant = doc.get("variant")
    if variant is not None and variant not in sc.pipeline_config_variants:
        known = sorted(sc.pipeline_config_variants)
        raise SpecError(f"variant: '{variant}' is not one of the scenario's {known}")
    factors = dict.fromkeys(FACTORS)
    for k, v in _mapping(doc.get("factors"), "factors", FACTORS).items():
        if v is not None and not isinstance(v, bool):
            raise SpecError(f"factors.{k}: true, false or null")
        factors[k] = v
    noise_doc = _mapping(doc.get("noise"), "noise", [f.name for f in fields(NoiseScale)])
    try:
        noise = NoiseScale(**{k: float(v) for k, v in noise_doc.items()})
        validate_scale(noise)
    except (TypeError, ValueError) as e:
        raise SpecError(str(e)) from e
    faults = []
    for i, f in enumerate(doc.get("faults") or []):
        f = _mapping(f, f"faults[{i}]", _FAULT_KEYS)
        try:
            faults.append(validate_fault(Fault(**f), duration_s))
        except (TypeError, ValueError) as e:
            raise SpecError(f"faults[{i}]: {e}") from e
    seed = doc.get("seed")
    if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int)):
        raise SpecError(f"seed: an integer or null, not {seed!r}")
    return ExperimentSpec(
        scenario=scenario,
        recording=_path(doc.get("recording"), "recording"),
        factors=factors,
        noise=noise,
        faults=tuple(faults),
        seed=seed,
        label=str(doc.get("label") or ""),
        variant=variant,
    )


def spec_to_dict(s: ExperimentSpec, *, with_label: bool = True) -> dict:
    doc = {
        "scenario": _rel(s.scenario),
        "recording": _rel(s.recording),
        "variant": s.variant,
        "factors": dict(s.factors),
        "noise": asdict(s.noise),
        "faults": [asdict(f) for f in s.faults],
        "seed": s.seed,
    }
    if with_label:
        doc["label"] = s.label
    return doc


def load_spec(path: Path) -> ExperimentSpec:
    try:
        doc = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as e:
        raise SpecError(f"{path}: {e}") from e
    return spec_from_dict(doc)


def dump_spec(s: ExperimentSpec, path: Path) -> None:
    Path(path).write_text(yaml.safe_dump(spec_to_dict(s), sort_keys=False))


def spec_key(s: ExperimentSpec, commit: str) -> str:
    """First 16 hex of sha256 over the canonical spec (label excluded) and the git commit."""
    doc = {"spec": spec_to_dict(s, with_label=False), "commit": commit}
    blob = json.dumps(doc, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()[:16]
