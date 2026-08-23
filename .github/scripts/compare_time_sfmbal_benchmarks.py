#!/usr/bin/env python3
"""Compare timeSFMBAL benchmark JSON files and render a PR-friendly markdown body."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict


def _load_result(path: Path) -> Dict[str, float]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if not isinstance(payload, list):
        raise ValueError(f"{path} is not a JSON array")

    metrics: Dict[str, float] = {}
    for item in payload:
        if not isinstance(item, dict):
            continue
        name = item.get("name")
        value = item.get("value")
        if not isinstance(name, str):
            continue
        if not isinstance(value, (int, float)):
            continue
        metrics[name] = float(value)
    return metrics


def _collect_results(folder: Path) -> Dict[str, Dict[str, float]]:
    if not folder.exists():
        return {}

    results: Dict[str, Dict[str, float]] = {}
    for path in sorted(folder.glob("*.json")):
        runner_id = path.stem
        results[runner_id] = _load_result(path)
    return results


def _fmt_seconds(value: float | None) -> str:
    if value is None:
        return "N/A"
    return f"{value:.6f}"


def _fmt_delta(head: float | None, base: float | None) -> str:
    if head is None or base is None:
        return "N/A"
    delta = head - base
    return f"{delta:+.6f}"


def _fmt_percent(head: float | None, base: float | None) -> str:
    if head is None or base is None or base == 0.0:
        return "N/A"
    pct = (head - base) / base * 100.0
    return f"{pct:+.2f}%"


def render_markdown(
    head_results: Dict[str, Dict[str, float]],
    base_results: Dict[str, Dict[str, float]],
    head_sha: str,
    base_sha: str,
) -> str:
    lines = ["<!-- time-sfmbal-benchmark -->", "## timeSFMBAL benchmark", ""]
    lines.append(f"- Head: `{head_sha}`")
    if base_sha:
        lines.append(f"- Base: `{base_sha}`")
    lines.append("")

    if not head_results:
        lines.append("No head benchmark results were found.")
        return "\n".join(lines) + "\n"

    lines.append(
        "| Runner | Metric | Base (s) | Head (s) | Delta (s) | Change |"
    )
    lines.append("| --- | --- | ---: | ---: | ---: | ---: |")

    missing_base_runners = []
    for runner_id in sorted(head_results):
        head_metrics = head_results[runner_id]
        base_metrics = base_results.get(runner_id, {})
        if not base_metrics:
            missing_base_runners.append(runner_id)

        metric_names = sorted(set(head_metrics) | set(base_metrics))
        for metric_name in metric_names:
            head_value = head_metrics.get(metric_name)
            base_value = base_metrics.get(metric_name)
            lines.append(
                "| "
                + f"{runner_id} | `{metric_name}` | {_fmt_seconds(base_value)} | "
                + f"{_fmt_seconds(head_value)} | {_fmt_delta(head_value, base_value)} | "
                + f"{_fmt_percent(head_value, base_value)} |"
            )

    if missing_base_runners:
        lines.append("")
        lines.append(
            "Missing base benchmark cache for: "
            + ", ".join(f"`{runner}`" for runner in missing_base_runners)
            + "."
        )

    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare per-runner timeSFMBAL benchmark JSON files."
    )
    parser.add_argument("--head-dir", required=True, help="Directory with head JSON files")
    parser.add_argument("--base-dir", required=True, help="Directory with base JSON files")
    parser.add_argument("--head-sha", required=True, help="Head commit SHA")
    parser.add_argument("--base-sha", default="", help="Base commit SHA")
    parser.add_argument("--output", required=True, help="Output markdown file")
    args = parser.parse_args()

    head_dir = Path(args.head_dir)
    base_dir = Path(args.base_dir)
    output = Path(args.output)

    head_results = _collect_results(head_dir)
    base_results = _collect_results(base_dir)
    markdown = render_markdown(head_results, base_results, args.head_sha, args.base_sha)

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(markdown, encoding="utf-8")
    print(markdown, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
