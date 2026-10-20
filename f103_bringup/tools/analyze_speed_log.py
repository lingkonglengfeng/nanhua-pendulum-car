"""Normalize serial-assistant chunking, export wheel CSV and a response figure."""
import argparse
import csv
import json
from pathlib import Path
import re
import statistics

parser = argparse.ArgumentParser()
parser.add_argument("log", type=Path)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--from-ms", type=float, default=None, help="Exclude records before a parameter change")
parser.add_argument("--wheel", type=int, choices=range(4), default=0)
parser.add_argument("--target", type=float, default=60.0)
args = parser.parse_args()
raw = args.log.read_text(encoding="utf-8-sig")
flat = re.sub(r"\[\d{2}:\d{2}:\d{2}\s+\d{3}\][收发]：", "", raw)
flat = flat.replace("\r", "").replace("\n", "")
pattern = r"D,((?:-?\d+(?:\.\d+)?,){47}-?\d+(?:\.\d+)?)"
rows = [list(map(float, m.group(1).split(","))) for m in re.finditer(pattern, flat)]
if args.from_ms is not None:
    rows = [r for r in rows if r[0] >= args.from_ms]
if not rows:
    raise SystemExit("No complete 49-field D records")
base = 12 + 9 * args.wheel
rpm_col, u_col, i_col = base + 1, base + 3, base + 4
active = [r for r in rows if r[5] == 1 and r[base] == args.target]
if not active:
    raise SystemExit(f"No active wheel {args.wheel} records at {args.target:g} RPM")
tail = [r for r in active if r[0] >= active[-1][0] - 2000]
summary = {
    "complete_records": len(rows), "active_records": len(active),
    "wheel": args.wheel, "target_rpm": args.target,
    "peak_rpm": max(r[rpm_col] for r in active),
    "last_2s": {
        "samples": len(tail), "mean_rpm": statistics.mean(r[rpm_col] for r in tail),
        "min_rpm": min(r[rpm_col] for r in tail), "max_rpm": max(r[rpm_col] for r in tail),
        "std_rpm": statistics.pstdev(r[rpm_col] for r in tail),
        "mean_u": statistics.mean(r[u_col] for r in tail),
    },
    "dt_ms_distribution": {str(v): sum(r[2] == v for r in rows) for v in sorted({r[2] for r in rows})},
    "exec_ms_distribution": {str(v): sum(r[7] == v for r in rows) for v in sorted({r[7] for r in rows})},
    "missed_periods_end": int(rows[-1][8]),
    "saturated_records": sum(abs(r[u_col]) >= .999 for r in active),
    "approx_settle_5pct_ms_from_first_active_record": next(
        (r[0] - active[0][0] for j, r in enumerate(active)
         if all(abs(k[rpm_col] - args.target) <= abs(args.target)*.05 for k in active[j:])), None),
    "from_ms_filter": args.from_ms,
    "first_active_ms": active[0][0],
    "first_active_rpm": active[0][rpm_col],
    "notes": "Timing estimates are relative to observed records, not exact command application; incomplete D records are excluded.",
}
args.out.mkdir(parents=True, exist_ok=True)
(args.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
headers = ["ms", "cycle", "dt_ms", "mode", "speed_en", "active", "reached", "exec_ms", "missed", "late", "log_drop", "dma_err"]
for w in range(4):
    headers.extend(f"{name}{w}" for name in ("target", "rpm", "error", "u", "i", "count", "delta", "ccr", "dir"))
with (args.out / "normalized.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(headers)
    writer.writerows(rows)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
time = [(r[0] - rows[0][0]) / 1000 for r in rows]
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, gridspec_kw={"height_ratios": [2, 1, 1]}, constrained_layout=True)
axes[0].plot(time, [r[base] for r in rows], "--", color="#ef8a19", label="Target")
axes[0].plot(time, [r[rpm_col] for r in rows], color="#1678a8", label="Measured")
axes[0].fill_between(time, [r[base]*.95 for r in rows], [r[base]*1.05 for r in rows], alpha=.15, color="#31a354", label="Target +/-5%")
period_ms = statistics.median(r[2] for r in rows[1:])
axes[0].set(ylabel=f"Wheel {args.wheel} (RPM)", title=f"{args.target:g} RPM response: measured control period {period_ms:g} ms")
axes[0].legend(loc="lower right")
axes[1].plot(time, [r[u_col] for r in rows], label="PID output", color="#1678a8")
axes[1].plot(time, [r[i_col] for r in rows], "--", label="Integral", color="#ef8a19")
axes[1].set(ylabel="Normalized output")
axes[1].legend(loc="lower right")
axes[2].plot(time[1:], [r[2] for r in rows[1:]], label="Measured interval", color="#c23b36")
axes[2].plot(time, [r[7] for r in rows], label="Execution time", color="#ef8a19")
axes[2].axhline(5, color="#1678a8", linestyle="--", label="Intended 5 ms")
axes[2].set(xlabel="MCU time from first record (s)", ylabel="Time (ms)", ylim=(0, 35))
axes[2].legend(loc="lower right", ncol=3)
for ax in axes:
    ax.grid(alpha=.2)
fig.savefig(args.out / f"wheel{args.wheel}_response.png", dpi=160)
plt.close(fig)
print(json.dumps(summary, indent=2))
