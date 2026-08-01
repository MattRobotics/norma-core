#!/usr/bin/env python3
"""Update the source-scope oracle for final 12-joint q=0 placement.

V40 adds one explicitly bounded `PROBE_HOME_TOLERANCE_TICKS` call site in the
all-joint final placement loop. The tolerance value and every earlier call site
remain unchanged.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"


def main() -> None:
    text = TESTS.read_text(encoding="utf-8")
    old = '''// V38 and V39 add one explicitly bounded use each for final
    // model-zero placement of LF and LH.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 15);'''
    new = '''// V38, V39 and V40 add one explicitly bounded use each for final
    // model-zero placement of LF, LH and the complete 12-joint HOME.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 16);'''
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"V40 HOME scope oracle: expected one match, found {count}")
    TESTS.write_text(text.replace(old, new, 1), encoding="utf-8")


if __name__ == "__main__":
    main()
