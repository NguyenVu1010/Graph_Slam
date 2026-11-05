"""Explorer Streamlit page driven headless with streamlit.testing (needs the ``explorer`` extra)."""

import shutil
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("streamlit")
pytest.importorskip("plotly")

from conftest import _REPO  # noqa: E402
from streamlit.testing.v1 import AppTest  # noqa: E402

from mss_sim.synth import SynthInfo  # noqa: E402

_APP = _REPO / "tools/mss_explorer/app.py"
_FAKE = """
import shutil, sys
from pathlib import Path
args = sys.argv[1:]
out = Path(args[args.index("--out") + 1])
out.mkdir(parents=True, exist_ok=True)
shutil.copyfile(out.parent / "gt.tum", out / "est.tum")
shutil.copyfile(out.parent / "gt.tum", out / "kf_opt.tum")
(out / "bench.json").write_text('{"n_loops_accepted": 0}')
"""


@pytest.fixture
def app(
    raw_bag: tuple[Path, SynthInfo], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> AppTest:
    root = tmp_path / "explorer"
    rec = root / "recordings" / "E-01-6s"
    shutil.copytree(raw_bag[0], rec / "raw_bag")
    text = (_REPO / "sim/scenarios/E-01.yaml").read_text()
    text = text.replace("duration_s: 60", "duration_s: 6").replace("t_end_s: 60", "t_end_s: 6")
    (rec / "scenario.yaml").write_text(text)
    fake = tmp_path / "fake_bench.py"
    fake.write_text(_FAKE)
    monkeypatch.setenv("MSS_EXPLORER_ROOT", str(root))
    monkeypatch.setenv("MSS_BENCH_CMD", f"{sys.executable} {fake}")
    at = AppTest.from_file(str(_APP), default_timeout=60)
    at.run()
    return at


def test_app_renders_without_runs(app: AppTest) -> None:
    assert not app.exception
    assert app.selectbox(key="recording").value == "E-01-6s"
    assert app.button(key="run")


def test_app_bad_fault_shows_error(app: AppTest, tmp_path: Path) -> None:
    app.selectbox(key="f_sensor").set_value("gnss").run()
    app.selectbox(key="f_kind").set_value("outage").run()
    app.number_input(key="f_start").set_value(5.0)
    app.number_input(key="f_end").set_value(4.0)
    app.button(key="f_add").click().run()
    assert not app.exception
    assert any("t_end_s" in e.value for e in app.error)
    assert app.session_state["faults"] == []


def test_app_run_appears_in_table(app: AppTest, tmp_path: Path) -> None:
    app.text_input(key="label").set_value("ref")
    app.button(key="run").click().run()
    assert not app.exception
    runs = tmp_path / "explorer" / "runs"
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and not list(runs.glob("*/result.json")):
        time.sleep(0.5)
    app.run()
    assert not app.exception
    table = app.dataframe[0].value
    assert "ref" in list(table["label"])
    assert list(table["status"]) == ["ok"]
