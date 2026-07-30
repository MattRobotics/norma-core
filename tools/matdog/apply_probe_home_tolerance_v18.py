#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: apply_probe_home_tolerance_v18.py MATDOG_RS MATDOG_TEST_RS"
        )

    source_path = Path(sys.argv[1])
    tests_path = Path(sys.argv[2])
    source = source_path.read_text(encoding="utf-8")
    tests = tests_path.read_text(encoding="utf-8")

    constants_anchor = (
        "const STATIC_TOLERANCE_TICKS: u16 = 10;\n"
        "const STARTUP_HOME_RECOVERY_LIMIT_TICKS: u16 = 64;\n"
    )
    constants_replacement = (
        "const STATIC_TOLERANCE_TICKS: u16 = 10;\n"
        "// The active probe can settle a few ticks farther from digital home under\n"
        "// geometry-prerequisite load and gearbox backlash. Keep this tolerance\n"
        "// separate so prerequisite drift and contact tracking remain at 10 ticks.\n"
        "const PROBE_HOME_TOLERANCE_TICKS: u16 = 16;\n"
        "const STARTUP_HOME_RECOVERY_LIMIT_TICKS: u16 = 64;\n"
    )
    source = replace_once(
        source,
        constants_anchor,
        constants_replacement,
        "probe-home tolerance constant",
    )

    probe_home_call = re.compile(
        r"self\.move_motor_to\(\s*"
        r"self\.profile\.motor_id,\s*"
        r"HOME_TICK,\s*"
        r"STATIC_TOLERANCE_TICKS,?\s*"
        r"\)"
    )
    matches = list(probe_home_call.finditer(source))
    if len(matches) != 3:
        raise SystemExit(
            f"probe-home move call count: expected 3, found {len(matches)}"
        )
    source = probe_home_call.sub(
        "self.move_motor_to(\n"
        "            self.profile.motor_id,\n"
        "            HOME_TICK,\n"
        "            PROBE_HOME_TOLERANCE_TICKS,\n"
        "        )",
        source,
    )

    test_anchor = """#[test]
fn robust_current_baseline_uses_median_and_mad() {
"""
    new_tests = r'''#[test]
fn probe_home_tolerance_covers_observed_m13_settle_without_weakening_static_gate() {
    let observed_error = circular_distance(2059, HOME_TICK);
    assert_eq!(observed_error, 11);
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(PROBE_HOME_TOLERANCE_TICKS, 16);
    assert!(observed_error > STATIC_TOLERANCE_TICKS);
    assert!(observed_error <= PROBE_HOME_TOLERANCE_TICKS);
}

#[test]
fn probe_home_tolerance_is_scoped_to_exactly_three_active_probe_returns() {
    let source = include_str!("matdog.rs");
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 4);
    assert_eq!(
        source
            .matches("HOME_TICK,\n            PROBE_HOME_TOLERANCE_TICKS")
            .count(),
        3
    );
    assert!(source.contains("if circular_distance(observation.position, target.target_tick)\n                    > STATIC_TOLERANCE_TICKS"));
    assert!(source.contains("if circular_distance(observation.position, target) <= STATIC_TOLERANCE_TICKS"));
}

'''
    tests = replace_once(
        tests,
        test_anchor,
        new_tests + test_anchor,
        "probe-home tolerance tests",
    )

    source_path.write_text(source, encoding="utf-8")
    tests_path.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
