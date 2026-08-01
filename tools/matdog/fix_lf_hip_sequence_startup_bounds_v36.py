#!/usr/bin/env python3
"""Make the V36 LF HIP restart envelope self-contained.

V34 removed the former symmetric bounds helper. The combined LF HIP sequence
must span both reviewed HIP guards, so compute the bounded union directly:
MAX guard 1472 - 10 = 1462, MIN guard 2624 + 10 = 2634.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog.rs"

old = """        return expanded_startup_bounds(
            minimum.guard_tick,
            maximum.guard_tick,
            STATIC_TOLERANCE_TICKS,
        );"""
new = """        let low = minimum
            .guard_tick
            .min(maximum.guard_tick)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = minimum
            .guard_tick
            .max(maximum.guard_tick)
            .saturating_add(STATIC_TOLERANCE_TICKS)
            .min(protocol::MAX_ANGLE_STEP);
        return (low, high);"""

source = SOURCE.read_text(encoding="utf-8")
count = source.count(old)
if count != 1:
    raise SystemExit(f"V36 startup bounds fix expected one match, found {count}")
SOURCE.write_text(source.replace(old, new, 1), encoding="utf-8")
