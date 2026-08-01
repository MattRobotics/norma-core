#!/usr/bin/env python3
"""Update the source-scope oracle for the LH final q=0 placement.

V39 adds exactly one additional bounded use of PROBE_HOME_TOLERANCE_TICKS in
`place_lh_at_model_zero`. The tolerance value, static-hold gate and every
existing call site remain unchanged.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"


def main() -> None:
    text = TESTS.read_text(encoding="utf-8")
    old = '''// V38 adds one explicitly bounded use for final model-zero placement.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 14);'''
    new = '''// V38 and V39 add one explicitly bounded use each for final
    // model-zero placement of LF and LH.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 15);'''
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"V39 HOME scope oracle: expected one match, found {count}")
    TESTS.write_text(text.replace(old, new, 1), encoding="utf-8")


if __name__ == "__main__":
    main()
