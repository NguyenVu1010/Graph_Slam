"""World services through the ``gz service`` CLI (no Gazebo Python bindings needed).

Used by ``live_start`` (world control, physics) and ``surface_node`` (wheel slip), as
``tools/record_scenario.sh`` does from the shell.
"""

from __future__ import annotations

import logging
import subprocess

SERVICE_TIMEOUT_MS = 5000  # default wall ms per request (the gz service --timeout)
# Wall seconds the gz process may take beyond its own --timeout (start-up, transport discovery)
# before it is killed. The CLI bounds only the request; a gz that hangs before it (no transport,
# wrong environment) would otherwise block the caller's only callback forever.
PROCESS_MARGIN_S = 2.0

_log = logging.getLogger(__name__)


def world_call(
    world: str, service: str, reqtype: str, req: str, timeout_ms: int = SERVICE_TIMEOUT_MS
) -> bool:
    """Call ``/world/<world>/<service>`` with a text-format request; True on a ``true`` reply.

    False, with a warning, when the ``gz`` binary is missing or the process outlives
    ``timeout_ms / 1000 + PROCESS_MARGIN_S`` wall seconds (it is then killed).
    """
    cmd = [
        "gz", "service", "-s", f"/world/{world}/{service}", "--reqtype", reqtype,
        "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", req,
    ]  # fmt: skip
    limit_s = timeout_ms / 1000.0 + PROCESS_MARGIN_S
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=limit_s)
    except FileNotFoundError:
        _log.warning("gz_cli: no gz binary on PATH, /world/%s/%s not called", world, service)
        return False
    except subprocess.TimeoutExpired:
        _log.warning("gz_cli: gz service /world/%s/%s killed after %.1f s", world, service, limit_s)
        return False
    return proc.returncode == 0 and "true" in proc.stdout
