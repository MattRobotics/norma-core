#!/usr/bin/env python3
"""Update historical test oracles for the V38 faster motion envelope.

V38 doubles commanded GoalSpeed from 80 to 160 while deliberately retaining
the conservative observed-motion floor of 80 ticks/s for deadline sizing. The
old test asserted that a 1279-tick return could not fit in 12 seconds at the
*commanded* speed and required at least 36 seconds; neither statement remains
true after the intentional speed change. The corrected test proves both the
new command capacity and the unchanged conservative 20.988-second deadline.

The full LF sequence also adds exactly one probe-HOME tolerance call site, so
the source-scope count moves from 13 to 14 without changing its value or any
static-hold tolerance.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"


def replace_exact(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    text = TESTS.read_text(encoding="utf-8")

    old_timeout = '''#[test]
fn motion_timeout_covers_observed_m12_max_return() {
    let distance = circular_distance(3327, HOME_TICK);
    assert_eq!(distance, 1279);

    // Hardware V12R measured 958 ticks in the old 12-second window at
    // GOAL_SPEED=80, leaving 321 ticks. The fixed deadline is therefore
    // mathematically insufficient for this valid MAX return.
    assert!(u64::from(distance) > u64::from(GOAL_SPEED) * MOTION_TIMEOUT.as_secs());

    let timeout = motion_timeout_for_distance(distance);
    assert!(timeout > MOTION_TIMEOUT);
    assert!(timeout >= Duration::from_secs(36));
}
'''
    new_timeout = '''#[test]
fn motion_timeout_covers_observed_m12_max_return() {
    let distance = circular_distance(3327, HOME_TICK);
    assert_eq!(distance, 1279);
    assert_eq!(GOAL_SPEED, 160);
    assert_eq!(MIN_EXPECTED_MOTION_TICKS_PER_SECOND, 80);

    // V38 can command this return inside the original 12-second capacity,
    // while deadline sizing intentionally retains the slower hardware-derived
    // 80 tick/s floor plus the unchanged five-second settling margin.
    assert!(u64::from(distance) <= u64::from(GOAL_SPEED) * MOTION_TIMEOUT.as_secs());

    let timeout = motion_timeout_for_distance(distance);
    assert!(timeout > MOTION_TIMEOUT);
    assert_eq!(timeout, Duration::from_millis(20_988));
}
'''
    text = replace_exact(text, old_timeout, new_timeout, "motion timeout oracle")

    old_count = 'assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 13);'
    new_count = '''// V38 adds one explicitly bounded use for final model-zero placement.
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 14);'''
    text = replace_exact(text, old_count, new_count, "probe HOME scope count")

    TESTS.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
