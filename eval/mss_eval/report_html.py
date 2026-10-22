"""Render a noise ``Report`` as one self-contained HTML page (light/dark, PNGs inlined)."""

from __future__ import annotations

import base64
import html
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from mss_eval.noise_measure import Report, Row, Section

# Palette tokens copied from docs/architecture.html so both pages look alike.
_CSS = """
:root {
  --bg: #f5f6f8; --fg: #1b1f24; --muted: #5b6470; --panel: #ffffff; --line: #d3d9e0;
  --done: #1f8a4c; --done-bg: #e1f4e8; --plan: #6b7280; --plan-bg: #eceff3;
  --fail: #c2352e; --fail-bg: #fde4e2; --accent: #2457d6;
  --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #0f1216; --fg: #e6e9ee; --muted: #9aa5b1; --panel: #171b21; --line: #2b323b;
  --done: #4cc47f; --done-bg: #10301f; --plan: #9aa5b1; --plan-bg: #22272e;
  --fail: #f07a72; --fail-bg: #3a1714; --accent: #7aa2ff;
} }
:root[data-theme="dark"] {
  --bg: #0f1216; --fg: #e6e9ee; --muted: #9aa5b1; --panel: #171b21; --line: #2b323b;
  --done: #4cc47f; --done-bg: #10301f; --plan: #9aa5b1; --plan-bg: #22272e;
  --fail: #f07a72; --fail-bg: #3a1714; --accent: #7aa2ff;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--fg);
  font: 15px/1.5 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
header, main { max-width: 1240px; margin: 0 auto; padding: 0 16px; }
header { padding-top: 24px; }
h1 { font-size: 22px; margin: 0 0 4px; }
h2 { font-size: 16px; margin: 0 0 4px; display: flex; gap: 10px; align-items: center; }
.sub, .note { color: var(--muted); font-size: 13px; }
.note { margin: 0 0 10px; }
.card { background: var(--panel); border: 1px solid var(--line); border-radius: 12px;
  padding: 14px; margin: 14px 0; overflow: hidden; }
.scroll { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-size: 13.5px; }
th, td { text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--line);
  vertical-align: top; }
th { color: var(--muted); font-weight: 600; font-size: 12px; text-transform: uppercase;
  letter-spacing: .04em; }
td.num { font-family: var(--mono); font-size: 12.5px; white-space: nowrap; }
td .note { display: block; font-size: 12px; margin: 0; }
.badge { display: inline-block; padding: 1px 9px; border-radius: 999px; font-size: 12px;
  font-weight: 600; border: 1px solid; white-space: nowrap; }
.pass { color: var(--done); background: var(--done-bg); border-color: var(--done); }
.fail { color: var(--fail); background: var(--fail-bg); border-color: var(--fail); }
.info { color: var(--plan); background: var(--plan-bg); border-color: var(--line); }
figure { margin: 12px 0 0; }
figure img { max-width: 100%; height: auto; display: block; border-radius: 8px;
  border: 1px solid var(--line); }
"""


def _fmt(v: float | str | None) -> str:
    if v is None:
        return "–"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, int | float):
        return f"{v:.4g}"
    return html.escape(str(v))


def _badge(ok: bool | None) -> str:
    if ok is None:
        return '<span class="badge info">info</span>'
    return '<span class="badge pass">pass</span>' if ok else '<span class="badge fail">fail</span>'


def _row(r: Row, with_pipeline: bool) -> str:
    note = f'<span class="note">{html.escape(r.note)}</span>' if r.note else ""
    cells = [f"<td>{html.escape(r.name)}{note}</td>", f'<td class="num">{_fmt(r.declared)}</td>']
    if with_pipeline:
        cells.append(f'<td class="num">{_fmt(r.pipeline)}</td>')
    cells += [
        f'<td class="num">{_fmt(r.measured)}</td>',
        f"<td>{html.escape(r.tol)}</td>",
        f"<td>{_badge(r.ok)}</td>",
    ]
    return "<tr>" + "".join(cells) + "</tr>"


def _section(s: Section) -> str:
    gated = [r.ok for r in s.rows if r.ok is not None]
    status = _badge(all(gated)) if gated else _badge(None)
    with_pipeline = any(r.pipeline is not None for r in s.rows)
    head = ["Check", "Declared", *(["Pipeline"] if with_pipeline else []), "Measured", "Tol", ""]
    ths = "".join(f"<th>{h}</th>" for h in head)
    rows = "".join(_row(r, with_pipeline) for r in s.rows)
    figs = "".join(
        f'<figure><img alt="{html.escape(s.name)} figure {i + 1}" '
        f'src="data:image/png;base64,{base64.b64encode(png).decode("ascii")}"></figure>'
        for i, png in enumerate(s.figures)
    )
    note = f'<p class="note">{html.escape(s.note)}</p>' if s.note else ""
    return (
        f'<section class="card"><h2>{html.escape(s.name)} {status}</h2>{note}'
        f'<div class="scroll"><table><thead><tr>{ths}</tr></thead><tbody>{rows}</tbody>'
        f"</table></div>{figs}</section>"
    )


def render(report: Report, title: str) -> str:
    """One static page: a card per sensor with a declared-vs-measured table and figures."""
    n_fail = sum(r.ok is False for s in report.sections for r in s.rows)
    verdict = _badge(report.passed)
    summary = "all gated checks pass" if report.passed else f"{n_fail} gated check(s) fail"
    body = "".join(_section(s) for s in report.sections)
    return (
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        f"<title>{html.escape(title)}</title><style>{_CSS}</style></head><body>"
        f"<header><h1>{html.escape(title)}</h1>"
        f'<div class="sub">{verdict} {summary}. Declared = noise profile; measured = bag. '
        "Info rows never gate (ADR-011 §7).</div></header>"
        f"<main>{body}</main></body></html>"
    )
