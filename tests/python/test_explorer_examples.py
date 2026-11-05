"""The ROOM-01 example specs shipped with the explorer load and validate."""

import pytest
from conftest import _REPO

from mss_explorer.spec import load_spec

_EXAMPLES = sorted((_REPO / "tools/mss_explorer/examples").glob("*.yaml"))


def test_examples_exist() -> None:
    names = {p.stem for p in _EXAMPLES}
    assert {"room01_ref", "room01_no_loop", "room01_gnss_outage", "room01_lidar_blackout"} <= names


@pytest.mark.parametrize("path", _EXAMPLES, ids=lambda p: p.stem)
def test_example_loads(path) -> None:  # noqa: ANN001
    s = load_spec(path)
    assert s.label and s.variant == "loops_on"
    assert s.recording == _REPO / "results/explorer/recordings/ROOM-01"
