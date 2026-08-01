#!/usr/bin/env python3
"""Correct V36 test oracles after the combined LF HIP implementation.

These changes do not alter runtime code:
- M12=3015 is a valid intermediate point on the bounded 2048->3072 path;
- unsigned GoalPosition is verified by u16 gate boundaries, not prose search;
- the V36 recovery adds five scoped PROBE_HOME_TOLERANCE_TICKS uses.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"

replacements = [
    (
        '    assert!(!armed_goal_target_allowed(&profile, 12, 3015));',
        '    assert!(armed_goal_target_allowed(&profile, 12, 3015));',
        'allow reviewed M12 intermediate path',
    ),
    (
        '    assert!(!source.contains("signed GoalPosition"));',
        '    assert!(!source.contains("i16::from_le_bytes"));',
        'verify no signed GoalPosition conversion',
    ),
    (
        '    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 8);',
        '    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 13);',
        'account for V36 scoped probe-home recovery checks',
    ),
]

text = TESTS.read_text(encoding="utf-8")
for old, new, label in replacements:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected one match, found {count}")
    text = text.replace(old, new, 1)
TESTS.write_text(text, encoding="utf-8")
