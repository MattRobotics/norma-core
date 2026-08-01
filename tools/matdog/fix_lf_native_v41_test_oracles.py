#!/usr/bin/env python3
"""Finalize V41 source shape and update persistent-session test oracles."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"

source = SOURCE.read_text(encoding="utf-8")
start_marker = "async fn execute_contact_stage("
end_marker = "async fn run_lf_full_calibration("
if source.count(start_marker) != 1 or source.count(end_marker) != 1:
    raise SystemExit("V41 legacy-stage cleanup markers are not unique")
start = source.index(start_marker)
end = source.index(end_marker, start)
source = source[:start] + source[end:]
SOURCE.write_text(source, encoding="utf-8")

text = TESTS.read_text(encoding="utf-8")
old = '''    // V38 adds one explicitly bounded use for final model-zero placement.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 14);
'''
new = '''    // V41 persistent-session startup, transitions and final affine-home use this bounded tolerance.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 17);
'''
count = text.count(old)
if count != 1:
    raise SystemExit(f"V41 probe-home oracle: expected one match, found {count}")
TESTS.write_text(text.replace(old, new, 1), encoding="utf-8")
