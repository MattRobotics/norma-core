#!/usr/bin/env python3
"""Apply the V37 bounded pre-corridor settle correction.

Hardware V36 reached LF HIP MAX but classified the observed target=1968,
present=1981 state as an early stall. The 13-tick residual is larger than the
10-tick strict tracking gate but still inside the already validated 16-tick
servo settle envelope.

V37 changes only detector classification outside a model contact corridor:
- residual error <= 16 ticks is treated as target settlement and the next
  bounded coarse target may be issued;
- a persistent obstruction is still rejected on the next target because its
  accumulated error exceeds 16 ticks;
- inside the contact corridor the original strict 10-tick gate is preserved,
  so contact confirmation and all guard/corridor limits remain unchanged.
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

    constant_anchor = "const STATIC_TOLERANCE_TICKS: u16 = 10;\n"
    constant_block = """const STATIC_TOLERANCE_TICKS: u16 = 10;
// V36 hardware evidence on LF HIP MAX showed a normal 13-tick directional
// settle at target=1968, present=1981 before the model contact corridor.
// Permit one bounded coarse-step continuation outside the corridor only.
// The strict 10-tick detector gate remains active inside every corridor.
const OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS: u16 = 16;
"""
    source = replace_exact(
        source,
        constant_anchor,
        constant_block,
        "insert bounded outside-corridor settle constant",
    )

    detector_anchor = """        let goal_error = circular_distance(observation.position, commanded_target);
        let target_ahead = i32::from(signed_tick_delta(commanded_target, observation.position))
            * i32::from(self.probe_sign)
            > 0;
        let _current_supports_contact = observation.current >= self.baseline.contact_threshold();

        if goal_error <= self.config.target_reached_tolerance_ticks {
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }
"""
    detector_replacement = """        let goal_error = circular_distance(observation.position, commanded_target);
        let inside_acceptance =
            (self.acceptance_low..=self.acceptance_high).contains(&observation.position);
        let target_settle_tolerance = if inside_acceptance {
            self.config.target_reached_tolerance_ticks
        } else {
            OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS
        };
        let target_ahead = i32::from(signed_tick_delta(commanded_target, observation.position))
            * i32::from(self.probe_sign)
            > 0;
        let _current_supports_contact = observation.current >= self.baseline.contact_threshold();

        if goal_error <= target_settle_tolerance {
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }
"""
    source = replace_exact(
        source,
        detector_anchor,
        detector_replacement,
        "scope settle tolerance by contact corridor",
    )

    old_acceptance = """                if (self.acceptance_low..=self.acceptance_high).contains(&observation.position) {
                    ContactState::ContactConfirmed
                } else {
                    ContactState::EarlyStall
                }
"""
    new_acceptance = """                if inside_acceptance {
                    ContactState::ContactConfirmed
                } else {
                    ContactState::EarlyStall
                }
"""
    source = replace_exact(
        source,
        old_acceptance,
        new_acceptance,
        "reuse exact corridor classification",
    )

    test_anchor = """#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
"""
    regression = """#[test]
fn lf_hip_max_v36_1981_settle_continues_once_but_real_stall_and_contact_remain_bounded() {
    let profile = profile_for_arm_value("LF_HIP_M13_MAX").unwrap();
    assert_eq!(contact_acceptance_bounds(&profile), (1472, 1600));
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS, 16);

    let baseline = BaselineStats {
        median_current: 2,
        mad_current: 0,
    };

    // Exact V36 hardware state: outside the MAX corridor, 13 ticks behind the
    // command and below the current threshold. This is bounded target settle,
    // not contact and not yet an early stall.
    let mut detector = HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
    let settle_target = 1968;
    let settle = observation(1981, 0, 2, settle_target);
    assert_eq!(circular_distance(settle.position, settle_target), 13);
    assert!(!position_inside_contact_acceptance(&profile, settle.position));
    for _ in 0..(usize::from(TARGET_STARTUP_SAMPLES) + 8) {
        assert_eq!(
            detector.observe(settle, settle_target),
            ContactState::FreeMotion
        );
    }

    // The grace is bounded. If M13 does not follow the next 32-tick coarse
    // command, accumulated error becomes 45 ticks and the original early-stall
    // protection fires after the unchanged persistence window.
    let next_target = 1936;
    let stuck = observation(1981, 0, 2, next_target);
    assert_eq!(circular_distance(stuck.position, next_target), 45);
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            detector.observe(stuck, next_target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::EarlyStall
    );

    // Inside the reviewed MAX corridor, the strict 10-tick gate remains in
    // force: a 13-tick persistent kinematic stall is still confirmed contact.
    let mut contact_detector =
        HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
    let contact_target = 1517;
    let contact = observation(1530, 0, 2, contact_target);
    assert_eq!(circular_distance(contact.position, contact_target), 13);
    assert!(position_inside_contact_acceptance(&profile, contact.position));
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            contact_detector.observe(contact, contact_target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::ContactConfirmed
    );
}

#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
"""
    tests = replace_exact(
        tests,
        test_anchor,
        regression,
        "add V36 LF HIP MAX hardware regression",
    )

    SOURCE.write_text(source, encoding="utf-8")
    TESTS.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
