#!/usr/bin/env python3
"""Update source-shape oracles for the V41 persistent LF session."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"

text = TESTS.read_text(encoding="utf-8")
old = '''    // V38 adds one explicitly bounded use for final model-zero placement.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 14);
'''
new = '''    // V41 adds bounded startup, persistent-session transition and final affine-home uses.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 18);
'''
count = text.count(old)
if count != 1:
    raise SystemExit(f"V41 probe-home oracle: expected one match, found {count}")
TESTS.write_text(text.replace(old, new, 1), encoding="utf-8")
