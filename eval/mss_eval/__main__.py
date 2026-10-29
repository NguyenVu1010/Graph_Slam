"""``python -m mss_eval <command> ...``. Commands: ``map-png`` (``mss_eval.map_png``).

The other tools keep their module form (``python -m mss_eval.ate``, ``.noise_report``).
"""

from __future__ import annotations

import sys

from mss_eval import map_png

_COMMANDS = {"map-png": map_png.main}


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] not in _COMMANDS:
        print(f"usage: python -m mss_eval {{{','.join(_COMMANDS)}}} ...", file=sys.stderr)
        return 2
    return _COMMANDS[argv[0]](argv[1:])


if __name__ == "__main__":
    sys.exit(main())
