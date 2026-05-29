from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


Result = Dict[str, Any]
TraceRow = Dict[str, Any]


def _load_results(path: Path) -> List[Result]:
    if path.is_dir():
        all_results = path / "all_results.json"
        if all_results.exists():
            with all_results.open("r", encoding="utf-8") as f:
                payload = json.load(f)
            if isinstance(payload, list):
                return payload
            if isinstance(payload, dict) and isinstance(payload.get("all_results"), list):
                return payload["all_results"]

        rows: List[Result] = []
        for file_path in sorted(path.glob("result_*.json")):
            with file_path.open("r", encoding="utf-8") as f:
                payload = json.load(f)
            rows.extend(payload.get("results", []))
        return rows

    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict) and isinstance(payload.get("all_results"), list):
        return payload["all_results"]
    if isinstance(payload, dict) and isinstance(payload.get("results"), list):
        return payload["results"]
    raise ValueError(f"Cannot find result rows in {path}")


def _float(row: Result, key: str) -> float:
    try:
        return float(row.get(key, 0.0))
    except (TypeError, ValueError):
        return 0.0


def _int(row: Result, key: str) -> int:
    try:
        return int(row.get(key, 0))
    except (TypeError, ValueError):
        return 0


def _method_summary(rows: Iterable[Result]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Result]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("method", "unknown"))].append(row)

    summary: List[Dict[str, Any]] = []
    for method, items in sorted(grouped.items()):
        net_reward = sum(_float(r, "net_reward") for r in items)
        delivered = sum(_int(r, "delivered") for r in items)
        total_orders = sum(_int(r, "total_orders") for r in items)
        on_time = sum(_int(r, "on_time") for r in items)
        late = sum(_int(r, "late") for r in items)
        missed = sum(_int(r, "missed") for r in items)
        wall_sec = sum(_float(r, "wall_sec") for r in items)
        move_cost = sum(_float(r, "total_movecost") for r in items)
        summary.append(
            {
                "method": method,
                "configs": len(items),
                "net_reward": round(net_reward, 4),
                "delivered": delivered,
                "total_orders": total_orders,
                "delivery_rate": round(100.0 * delivered / max(total_orders, 1), 2),
                "on_time": on_time,
                "late": late,
                "missed": missed,
                "on_time_rate": round(100.0 * on_time / max(delivered, 1), 2),
                "reward_per_delivered": round(net_reward / max(delivered, 1), 4),
                "total_movecost": round(move_cost, 4),
                "wall_sec": round(wall_sec, 4),
            }
        )
    return summary


def _config_summary(rows: Iterable[Result]) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for row in sorted(rows, key=lambda r: (str(r.get("config_name", "")), str(r.get("method", "")))):
        delivered = _int(row, "delivered")
        total_orders = _int(row, "total_orders")
        net_reward = _float(row, "net_reward")
        output.append(
            {
                "config_name": row.get("config_name", "unknown"),
                "method": row.get("method", "unknown"),
                "net_reward": round(net_reward, 4),
                "delivered": delivered,
                "total_orders": total_orders,
                "delivery_rate": round(100.0 * delivered / max(total_orders, 1), 2),
                "on_time": _int(row, "on_time"),
                "late": _int(row, "late"),
                "missed": _int(row, "missed"),
                "on_time_rate": round(_float(row, "on_time_rate"), 2),
                "reward_per_delivered": round(net_reward / max(delivered, 1), 4),
                "wall_sec": round(_float(row, "wall_sec"), 4),
            }
        )
    return output


def _write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _load_trace_rows(trace_dir: Path) -> List[TraceRow]:
    rows: List[TraceRow] = []
    for file_path in sorted(trace_dir.glob("solver_trace_*.jsonl")):
        stem = file_path.stem
        parts = stem.split("_")
        method = parts[2] if len(parts) >= 4 else "unknown"
        config = "_".join(parts[3:]) if len(parts) >= 4 else "unknown"
        with file_path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                row = json.loads(line)
                row.setdefault("method", method)
                row.setdefault("config_name", config)
                rows.append(row)
    return rows


def _trace_avg(values: List[float]) -> float:
    return round(sum(values) / max(len(values), 1), 4)


def _trace_summaries(trace_rows: List[TraceRow]) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    grouped: Dict[tuple[str, str], List[TraceRow]] = defaultdict(list)
    for row in trace_rows:
        grouped[(str(row.get("method", "unknown")), str(row.get("config_name", "unknown")))].append(row)

    funnel: List[Dict[str, Any]] = []
    backlog: List[Dict[str, Any]] = []
    movement: List[Dict[str, Any]] = []

    for (method, config), rows in sorted(grouped.items()):
        picked = sum(len(row.get("picked_order_ids", [])) for row in rows)
        delivered = sum(len(row.get("delivered_order_ids", [])) for row in rows)
        active_values = [float(row.get("active_orders", 0)) for row in rows]
        new_values = [float(row.get("new_orders", 0)) for row in rows]
        idle_values = [float(len(row.get("idle_shippers", []))) for row in rows]
        blocked_values = [float(len(row.get("blocked_or_wait_actions", []))) for row in rows]
        bag_values: List[float] = []
        nearest_values: List[float] = []
        for row in rows:
            bag_sizes = row.get("bag_sizes", {})
            if isinstance(bag_sizes, dict):
                bag_values.extend(float(v) for v in bag_sizes.values())
            nearest = row.get("nearest_pickup_distance", {})
            if isinstance(nearest, dict):
                nearest_values.extend(float(v) for v in nearest.values() if float(v) < 10**8)

        funnel.append(
            {
                "method": method,
                "config_name": config,
                "steps": len(rows),
                "picked": picked,
                "delivered": delivered,
                "pickup_to_delivery_rate": round(100.0 * delivered / max(picked, 1), 2),
            }
        )
        backlog.append(
            {
                "method": method,
                "config_name": config,
                "avg_active_orders": _trace_avg(active_values),
                "max_active_orders": max(active_values) if active_values else 0,
                "avg_new_orders": _trace_avg(new_values),
                "avg_idle_shippers": _trace_avg(idle_values),
                "avg_nearest_pickup_distance": _trace_avg(nearest_values),
            }
        )
        movement.append(
            {
                "method": method,
                "config_name": config,
                "avg_blocked_or_wait": _trace_avg(blocked_values),
                "total_blocked_or_wait": int(sum(blocked_values)),
                "avg_bag_size": _trace_avg(bag_values),
                "max_bag_size": max(bag_values) if bag_values else 0,
            }
        )

    return funnel, backlog, movement


def _trace_report(
    funnel: List[Dict[str, Any]],
    backlog: List[Dict[str, Any]],
    movement: List[Dict[str, Any]],
) -> str:
    lines = ["# Trace Bottleneck Report", ""]
    if not funnel:
        return "# Trace Bottleneck Report\n\nNo trace rows found.\n"

    lines.append("## Funnel")
    for row in sorted(funnel, key=lambda r: (r["method"], r["config_name"])):
        lines.append(
            "- {method} {config_name}: picked={picked}, delivered={delivered}, "
            "pickup_to_delivery={pickup_to_delivery_rate:.1f}%".format(**row)
        )

    lines.extend(["", "## Backlog"])
    for row in sorted(backlog, key=lambda r: (r["method"], -r["avg_active_orders"])):
        flags = []
        if row["avg_active_orders"] > 20:
            flags.append("high backlog")
        if row["avg_idle_shippers"] > 1 and row["avg_active_orders"] > 5:
            flags.append("idle while backlog")
        if row["avg_nearest_pickup_distance"] > 6:
            flags.append("far from pickups")
        if flags:
            lines.append(
                "- {method} {config_name}: {flags}; avg_active={avg_active_orders:.2f}, "
                "max_active={max_active_orders:.0f}, avg_idle={avg_idle_shippers:.2f}, "
                "avg_nearest={avg_nearest_pickup_distance:.2f}".format(flags=", ".join(flags), **row)
            )

    lines.extend(["", "## Movement"])
    for row in sorted(movement, key=lambda r: (r["method"], -r["total_blocked_or_wait"])):
        flags = []
        if row["avg_blocked_or_wait"] > 1:
            flags.append("many waits/blocks")
        if row["avg_bag_size"] < 0.7:
            flags.append("low capacity use")
        if flags:
            lines.append(
                "- {method} {config_name}: {flags}; avg_wait={avg_blocked_or_wait:.2f}, "
                "total_wait={total_blocked_or_wait}, avg_bag={avg_bag_size:.2f}, max_bag={max_bag_size:.0f}".format(
                    flags=", ".join(flags),
                    **row,
                )
            )

    return "\n".join(lines) + "\n"


def _bottleneck_report(method_rows: List[Dict[str, Any]], config_rows: List[Dict[str, Any]]) -> str:
    lines = ["# Bottleneck Report", ""]
    lines.append("## Method Summary")
    for row in sorted(method_rows, key=lambda r: r["net_reward"], reverse=True):
        lines.append(
            "- {method}: score={net_reward:.2f}, delivered={delivered}/{total_orders} "
            "({delivery_rate:.1f}%), on_time={on_time_rate:.1f}%, missed={missed}, "
            "reward/order={reward_per_delivered:.2f}, wall={wall_sec:.2f}s".format(**row)
        )

    lines.extend(["", "## Main Bottlenecks"])
    for row in sorted(config_rows, key=lambda r: (r["method"], -r["missed"])):
        flags = []
        if row["missed"] > 0.25 * row["total_orders"]:
            flags.append("high missed")
        if row["delivered"] and row["on_time_rate"] < 75.0:
            flags.append("late risk")
        if row["reward_per_delivered"] < 18.0:
            flags.append("low reward/order")
        if row["wall_sec"] > 30.0:
            flags.append("runtime")
        if flags:
            lines.append(
                "- {method} {config_name}: {flags}; score={net_reward:.2f}, "
                "delivered={delivered}/{total_orders}, on_time={on_time_rate:.1f}%, "
                "reward/order={reward_per_delivered:.2f}".format(
                    flags=", ".join(flags),
                    **row,
                )
            )

    lines.extend(
        [
            "",
            "## Suggested Improvements",
            "- High missed: prioritize throughput, closer pickups, dynamic regional assignment, and idle repositioning.",
            "- Late risk: increase deadline pressure and deliver urgent carried orders before optional batching.",
            "- Low reward/order: raise priority and weight terms for feasible high-value orders.",
            "- Runtime: reduce candidates, horizon, ants, or CBS repair iterations for affected methods.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze MAPD solver result JSON files.")
    parser.add_argument("input", help="Result directory or JSON file, e.g. results/ or results/all_results.json")
    parser.add_argument("--out", default=None, help="Output directory. Defaults to <input>/analysis for directories.")
    parser.add_argument("--trace-dir", default="/tmp", help="Directory containing solver_trace_*.jsonl files.")
    args = parser.parse_args()

    input_path = Path(args.input)
    rows = _load_results(input_path)
    if not rows:
        raise SystemExit(f"No result rows found in {input_path}")

    out_dir = Path(args.out) if args.out else (input_path / "analysis" if input_path.is_dir() else input_path.parent / "analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    method_rows = _method_summary(rows)
    config_rows = _config_summary(rows)

    _write_csv(out_dir / "method_summary.csv", method_rows)
    _write_csv(out_dir / "config_summary.csv", config_rows)
    (out_dir / "bottleneck_report.md").write_text(
        _bottleneck_report(method_rows, config_rows),
        encoding="utf-8",
    )

    trace_rows = _load_trace_rows(Path(args.trace_dir))
    funnel_rows, backlog_rows, movement_rows = _trace_summaries(trace_rows)
    _write_csv(out_dir / "funnel_summary.csv", funnel_rows)
    _write_csv(out_dir / "backlog_summary.csv", backlog_rows)
    _write_csv(out_dir / "movement_summary.csv", movement_rows)
    (out_dir / "trace_bottleneck_report.md").write_text(
        _trace_report(funnel_rows, backlog_rows, movement_rows),
        encoding="utf-8",
    )

    print(f"Wrote {out_dir / 'method_summary.csv'}")
    print(f"Wrote {out_dir / 'config_summary.csv'}")
    print(f"Wrote {out_dir / 'bottleneck_report.md'}")
    print(f"Wrote {out_dir / 'funnel_summary.csv'}")
    print(f"Wrote {out_dir / 'backlog_summary.csv'}")
    print(f"Wrote {out_dir / 'movement_summary.csv'}")
    print(f"Wrote {out_dir / 'trace_bottleneck_report.md'}")


if __name__ == "__main__":
    main()
