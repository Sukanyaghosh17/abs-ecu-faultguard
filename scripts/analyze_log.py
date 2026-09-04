"""
analyze_log.py — ABS ECU Simulation Log Visualizer
===================================================
Reads logs/abs_log.csv (relative to the repository root, located
automatically from this file's own location) and produces a 2×2
dashboard saved to figures/abs_dashboard.png.

Panels:
  [0,0] Wheel Speeds    — true speed per wheel
  [0,1] Vehicle Speed   — true vehicle speed vs. ECU reference estimate
  [1,0] Slip Ratio      — computed per wheel from CSV data
  [1,1] Brake Pressure  — hydraulic pressure per wheel (%)

Faulty wheels (F0-F3 column > 0 for any row) are drawn with a
dashed line and a shaded background region to make them visually
distinct from healthy wheels.

Usage (works from any directory):
  python scripts/analyze_log.py

Dependencies:
  pip install pandas matplotlib
"""

import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Paths ─────────────────────────────────────────────────────────────────
# Anchor on this script's location and go up one level to the repo root.
# This means the script works regardless of which directory you run it from.
REPO_ROOT  = Path(__file__).resolve().parent.parent
LOG_PATH   = REPO_ROOT / "logs" / "abs_log.csv"
FIG_DIR    = REPO_ROOT / "figures"
OUTPUT_FIG = FIG_DIR / "abs_dashboard.png"

# ── Load data ─────────────────────────────────────────────────────────────
if not LOG_PATH.exists():
    sys.exit(f"[ERROR] Log file not found: {LOG_PATH}\n"
             f"  Run the simulation first: ./abs_ecu_sim")

df = pd.read_csv(LOG_PATH)

time      = df["Time(s)"]
veh_speed = df["Veh_Speed"]
ref_speed = df["Est_Veh_Speed"]
abs_flag  = df["ABS_Active"]

wheel_cols    = ["W0_Speed", "W1_Speed", "W2_Speed", "W3_Speed"]
pressure_cols = ["P0", "P1", "P2", "P3"]
fault_cols    = ["F0", "F1", "F2", "F3"]

# ── Compute slip ratio per wheel ──────────────────────────────────────────
# Slip = (V_ref - V_wheel) / V_ref  (clipped to [0, 1] for display)
# Guard against div-by-zero when the vehicle is nearly stopped.
safe_ref = ref_speed.clip(lower=0.01)
slip_cols = {}
for i, wc in enumerate(wheel_cols):
    slip_cols[f"Slip{i}"] = ((safe_ref - df[wc]) / safe_ref).clip(0.0, 1.0)

# ── Determine faulty wheels ───────────────────────────────────────────────
# A wheel is considered "faulty" for any cycle where its F column is non-zero.
# We draw its lines dashed and shade the subplot background.
fault_labels = {0: "none", 1: "bias", 2: "lockup", 3: "disconnected"}

def is_faulty(wheel_idx: int) -> bool:
    col = fault_cols[wheel_idx]
    return col in df.columns and (df[col] != 0).any()

def fault_name(wheel_idx: int) -> str:
    col = fault_cols[wheel_idx]
    if col not in df.columns:
        return "none"
    mode = int(df[col].max())
    return fault_labels.get(mode, "unknown")

# ── Colour palette (one colour per wheel) ─────────────────────────────────
WHEEL_COLOURS = ["#3a86ff", "#ff006e", "#8338ec", "#fb5607"]

# ── Figure layout ─────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
fig.suptitle("ABS ECU Simulation Dashboard", fontsize=15, fontweight="bold")

# Helper: shade ABS-active regions in grey across an axis.
def shade_abs_active(ax):
    in_region   = False
    region_start = None
    for t_val, flag in zip(time, abs_flag):
        if flag and not in_region:
            region_start = t_val
            in_region    = True
        elif not flag and in_region:
            ax.axvspan(region_start, t_val, color="grey",
                       alpha=0.15, label="_abs_region")
            in_region = False
    if in_region:
        ax.axvspan(region_start, time.iloc[-1], color="grey",
                   alpha=0.15, label="_abs_region")

# Helper: add a faint red background if ANY fault is active for a wheel.
def shade_fault_region(ax, wheel_idx):
    col = fault_cols[wheel_idx]
    if col not in df.columns:
        return
    in_region    = False
    region_start = None
    for t_val, fv in zip(time, df[col]):
        if fv != 0 and not in_region:
            region_start = t_val
            in_region    = True
        elif fv == 0 and in_region:
            ax.axvspan(region_start, t_val, color="#ff006e",
                       alpha=0.08, label="_fault_region")
            in_region = False
    if in_region:
        ax.axvspan(region_start, time.iloc[-1], color="#ff006e",
                   alpha=0.08, label="_fault_region")

# ── Panel [0,0]: Wheel Speeds ─────────────────────────────────────────────
ax = axes[0, 0]
shade_abs_active(ax)

for i, (col, colour) in enumerate(zip(wheel_cols, WHEEL_COLOURS)):
    faulty  = is_faulty(i)
    ls      = "--" if faulty else "-"
    label   = f"Wheel {i}"
    if faulty:
        label += f" [{fault_name(i)}]"
        shade_fault_region(ax, i)

    ax.plot(time, df[col], color=colour, linestyle=ls,
            linewidth=1.8, label=label)

ax.set_ylabel("Speed (m/s)")
ax.set_title("Wheel Speeds")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.4)

# ── Panel [0,1]: Vehicle Speed ────────────────────────────────────────────
ax = axes[0, 1]
shade_abs_active(ax)

ax.plot(time, veh_speed, color="#2ec4b6", linewidth=2.2,
        label="True vehicle speed")
ax.plot(time, ref_speed, color="#e9c46a", linewidth=1.8,
        linestyle="--", label="ECU reference (peak-hold)")

ax.set_ylabel("Speed (m/s)")
ax.set_title("Vehicle Speed")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.4)

# ── Panel [1,0]: Slip Ratio ───────────────────────────────────────────────
ax = axes[1, 0]
shade_abs_active(ax)

for i, colour in enumerate(WHEEL_COLOURS):
    faulty  = is_faulty(i)
    ls      = "--" if faulty else "-"
    label   = f"Wheel {i}"
    if faulty:
        label += f" [{fault_name(i)}]"
        shade_fault_region(ax, i)

    ax.plot(time, slip_cols[f"Slip{i}"], color=colour,
            linestyle=ls, linewidth=1.8, label=label)

# Mark the ABS trigger threshold.
ax.axhline(0.20, color="red",    linestyle=":", linewidth=1.2,
           label="Release threshold (0.20)")
ax.axhline(0.05, color="orange", linestyle=":", linewidth=1.2,
           label="Apply threshold (0.05)")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Slip ratio")
ax.set_title("Wheel Slip Ratio")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.4)

# ── Panel [1,1]: Brake Pressure ───────────────────────────────────────────
ax = axes[1, 1]
shade_abs_active(ax)

for i, (col, colour) in enumerate(zip(pressure_cols, WHEEL_COLOURS)):
    faulty  = is_faulty(i)
    ls      = "--" if faulty else "-"
    label   = f"Wheel {i}"
    if faulty:
        label += f" [{fault_name(i)}]"
        shade_fault_region(ax, i)

    ax.plot(time, df[col], color=colour, linestyle=ls,
            linewidth=1.8, label=label)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Brake pressure (%)")
ax.set_title("Brake Pressure")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.4)

# ── Shared legend for ABS and fault regions ───────────────────────────────
abs_patch   = mpatches.Patch(color="grey",    alpha=0.3, label="ABS active (shaded)")
fault_patch = mpatches.Patch(color="#ff006e", alpha=0.2, label="Fault active (shaded)")
fig.legend(handles=[abs_patch, fault_patch], loc="lower center",
           ncol=2, fontsize=9, framealpha=0.8)

# ── Save and show ─────────────────────────────────────────────────────────
plt.tight_layout(rect=[0, 0.04, 1, 1])   # leave room for shared legend

FIG_DIR.mkdir(parents=True, exist_ok=True)
fig.savefig(OUTPUT_FIG, dpi=150, bbox_inches="tight")
print(f"Dashboard saved → {OUTPUT_FIG}")

# ── Console statistics (unchanged from original) ──────────────────────────
print("\n=== Vehicle Speed Statistics ===")
print(df["Veh_Speed"].describe().to_string())
print("\n=== ABS Active Summary ===")
total  = len(abs_flag)
active = int(abs_flag.sum())
print(f"ABS activated  : {active} cycles out of {total}")
print(f"ABS activation : {100 * active / total:.2f}% of runtime")
if any(is_faulty(w) for w in range(4)):
    print("\nFaulted wheels:")
    for w in range(4):
        if is_faulty(w):
            print(f"  Wheel {w} : {fault_name(w)}")

plt.show()