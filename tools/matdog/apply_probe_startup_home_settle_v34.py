#!/usr/bin/env python3
"""Apply the V34 MATDOG probe-startup home-endpoint correction.

The transformation is deliberately narrow:
- the model/guard endpoint keeps STATIC_TOLERANCE_TICKS;
- only the digital-home endpoint of the active probe uses the existing
  PROBE_HOME_TOLERANCE_TICKS;
- prerequisite holds, contact corridors and all motion limits are unchanged.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"


def replace_exact(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    tests = TESTS.read_text(encoding="utf-8")

    helper_anchor = """fn home_hold_tolerance(
    profile: &ContactProfile,
    motor_id: u8,
    probe_home_handoff_active: bool,
) -> u16 {
"""
    helper = """// Preserve the strict model/guard endpoint while allowing the active probe
// to settle within the already validated digital-home tolerance under
// geometry-prerequisite load and gearbox backlash.
fn startup_probe_bounds(profile: &ContactProfile) -> (u16, u16) {
    if profile.probe_sign < 0 {
        (
            profile.guard_tick.saturating_sub(STATIC_TOLERANCE_TICKS),
            HOME_TICK
                .saturating_add(PROBE_HOME_TOLERANCE_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        )
    } else {
        (
            HOME_TICK.saturating_sub(PROBE_HOME_TOLERANCE_TICKS),
            profile
                .guard_tick
                .saturating_add(STATIC_TOLERANCE_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        )
    }
}

fn home_hold_tolerance(
    profile: &ContactProfile,
    motor_id: u8,
    probe_home_handoff_active: bool,
) -> u16 {
"""
    source = replace_exact(source, helper_anchor, helper, "insert startup_probe_bounds")

    old_probe_arm = """        StartupRole::Probe => {
            expanded_linear_bounds(HOME_TICK, profile.guard_tick, STATIC_TOLERANCE_TICKS)
        }
"""
    new_probe_arm = """        StartupRole::Probe => startup_probe_bounds(profile),
"""
    source = replace_exact(source, old_probe_arm, new_probe_arm, "use asymmetric probe bounds")

    test_anchor = """#[test]
fn startup_wrong_profile_residue_is_rejected() {
"""
    regression_test = """#[test]
fn startup_probe_home_endpoint_accepts_observed_m11_2059_without_weakening_guard_side() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    assert_eq!(profile.probe_sign, -1);
    assert_eq!(startup_probe_bounds(&profile), (1547, 2064));
    assert_eq!(startup_envelope(&profile, 11), (1547, 2064));
    assert!(startup_position_allowed(&profile, 11, 2059));
    assert!(startup_position_allowed(&profile, 11, 2064));
    assert!(!startup_position_allowed(&profile, 11, 2065));
    assert!(startup_position_allowed(&profile, 11, 1547));
    assert!(!startup_position_allowed(&profile, 11, 1546));

    let mut probe = observation(2059, 0, 0, HOME_TICK);
    probe.torque_enabled = false;
    let home_ready = BTreeSet::new();
    let established = BTreeSet::new();
    assert!(validate_profile_entry_hold(
        &profile,
        11,
        0,
        &home_ready,
        &established,
        probe,
    )
    .is_ok());
}

#[test]
fn startup_wrong_profile_residue_is_rejected() {
"""
    tests = replace_exact(tests, test_anchor, regression_test, "add M11 2059 regression")

    old_oracle = """                StartupRole::Probe => {
                    expanded_linear_bounds(HOME_TICK, profile.guard_tick, STATIC_TOLERANCE_TICKS)
                }
"""
    new_oracle = """                StartupRole::Probe => startup_probe_bounds(&profile),
"""
    tests = replace_exact(tests, old_oracle, new_oracle, "update exhaustive oracle")

    old_scope = """fn probe_home_tolerance_is_scoped_to_exactly_three_active_probe_returns() {
    let source = include_str!("matdog.rs");
    let normalized = source.split_whitespace().collect::<Vec<_>>().join(" ");
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 5);
"""
    new_scope = """fn probe_home_tolerance_is_scoped_to_startup_home_endpoint_and_active_probe_returns() {
    let source = include_str!("matdog.rs");
    let normalized = source.split_whitespace().collect::<Vec<_>>().join(" ");
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 6);
    assert!(normalized.contains("StartupRole::Probe => startup_probe_bounds(profile)"));
"""
    tests = replace_exact(tests, old_scope, new_scope, "update tolerance scope test")

    SOURCE.write_text(source, encoding="utf-8")
    TESTS.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
