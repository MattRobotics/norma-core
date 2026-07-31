#!/usr/bin/env python3
"""Apply the ordered UPPER -> LOWER -> HIP MATDOG redesign.

This transformer is intentionally applied after the validated V11, V13 and V18
patches. It does not enable HIP hardware. It prepares safe LOWER calibration,
adds audited side-specific compact HIP prerequisites, rejects early stalls
outside the model corridor and blocks every isolated HIP hardware arm until a
verified UPPER+LOWER phase proof is implemented.
"""

from __future__ import annotations

from pathlib import Path
import sys


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: count={count}, expected=1")
    return text.replace(old, new, 1)


def replace_between(
    text: str,
    start_marker: str,
    end_marker: str,
    replacement: str,
    label: str,
) -> str:
    start_count = text.count(start_marker)
    if start_count != 1:
        raise SystemExit(f"{label}: start count={start_count}, expected=1")
    start = text.index(start_marker)
    end = text.find(end_marker, start + len(start_marker))
    if end < 0:
        raise SystemExit(f"{label}: end marker not found")
    return text[:start] + replacement + text[end:]


def main() -> int:
    if len(sys.argv) != 3:
        raise SystemExit("usage: apply_ordered_sequence_v20.py MATDOG_RS MATDOG_TEST_RS")
    source_path = Path(sys.argv[1])
    test_path = Path(sys.argv[2])
    source = source_path.read_text(encoding="utf-8")
    tests = test_path.read_text(encoding="utf-8")

    source = replace_once(
        source,
        "const UPPER_90_DELTA: i16 = 1024;\n",
        "const UPPER_90_DELTA: i16 = 1024;\n"
        "const UPPER_85_DELTA: i16 = 967;\n"
        "const LOWER_FOLDED_DELTA: i16 = -990;\n"
        "const CONTACT_ACCEPTANCE_INNER_TICKS: u16 = 64;\n"
        "const HIP_HARDWARE_BLOCK_REASON: &str = \\\"HIP hardware is blocked until ordered UPPER MIN/MAX and LOWER MIN/MAX phase proof is verified\\\";\n",
        "ordered constants",
    )

    prerequisites = '''fn hip_upper_clearance_delta(leg: Leg, side: ContactSide) -> i16 {
    match (leg, side) {
        (Leg::Lf, ContactSide::Min)
        | (Leg::Rf, ContactSide::Max)
        | (Leg::Rh, _)
        | (Leg::Lh, _) => UPPER_90_DELTA,
        (Leg::Lf, ContactSide::Max) | (Leg::Rf, ContactSide::Min) => UPPER_85_DELTA,
    }
}

fn prerequisites_for(
    leg: Leg,
    kind: JointKind,
    side: ContactSide,
) -> Result<Vec<StaticTarget>, String> {
    let mut targets = Vec::new();
    match leg {
        Leg::Lf => targets.push(static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)?),
        Leg::Rf => targets.push(static_target(Leg::Rh, JointKind::Upper, UPPER_30_DELTA)?),
        Leg::Rh | Leg::Lh => {}
    }

    match kind {
        JointKind::Upper => {
            targets.push(static_target(leg, JointKind::Hip, 0)?);
            targets.push(static_target(leg, JointKind::Lower, 0)?);
        }
        JointKind::Lower => {
            targets.push(static_target(leg, JointKind::Hip, 0)?);
            targets.push(static_target(leg, JointKind::Upper, UPPER_90_DELTA)?);
        }
        JointKind::Hip => {
            targets.push(static_target(
                leg,
                JointKind::Upper,
                hip_upper_clearance_delta(leg, side),
            )?);
            targets.push(static_target(
                leg,
                JointKind::Lower,
                LOWER_FOLDED_DELTA,
            )?);
        }
    }
    Ok(targets)
}

'''
    source = replace_between(
        source,
        "fn prerequisites_for(leg: Leg, kind: JointKind) -> Result<Vec<StaticTarget>, String> {\n",
        "#[cfg(test)]\nfn prerequisite_restore_order",
        prerequisites,
        "ordered prerequisites",
    )
    source = replace_once(
        source,
        "prerequisites: prerequisites_for(leg, joint)?,",
        "prerequisites: prerequisites_for(leg, joint, side)?,",
        "profile prerequisite call",
    )
    source = replace_once(
        source,
        "for joint in [JointKind::Upper, JointKind::Hip, JointKind::Lower] {",
        "for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {",
        "profile order",
    )

    active_profile = '''fn hardware_profile_allowed(profile: &ContactProfile) -> Result<(), String> {
    if profile.joint == JointKind::Hip {
        return Err(format!("{}: {}", profile.label, HIP_HARDWARE_BLOCK_REASON));
    }
    Ok(())
}

pub(crate) fn active_profile() -> Result<ContactProfile, String> {
    let value = std::env::var(MATDOG_ARM_ENV).map_err(|_| {
        format!("MATDOG calibrator is not armed: set {MATDOG_ARM_ENV} explicitly")
    })?;
    let profile = profile_for_arm_value(&value)?;
    hardware_profile_allowed(&profile)?;
    Ok(profile)
}

'''
    source = replace_between(
        source,
        "pub(crate) fn active_profile() -> Result<ContactProfile, String> {\n",
        "pub(crate) fn armed_ram_write_allowed",
        active_profile,
        "isolated HIP hardware block",
    )

    acceptance_helpers = '''fn contact_acceptance_bounds(profile: &ContactProfile) -> (u16, u16) {
    let inner = i32::from(profile.urdf_limit_tick)
        - i32::from(profile.probe_sign) * i32::from(CONTACT_ACCEPTANCE_INNER_TICKS);
    let inner = u16::try_from(inner)
        .unwrap_or(if profile.probe_sign > 0 { 0 } else { protocol::MAX_ANGLE_STEP });
    (
        inner.min(profile.guard_tick),
        inner.max(profile.guard_tick).min(protocol::MAX_ANGLE_STEP),
    )
}

fn position_inside_contact_acceptance(
    profile: &ContactProfile,
    position: u16,
) -> bool {
    let (low, high) = contact_acceptance_bounds(profile);
    (low..=high).contains(&position)
}

'''
    source = replace_once(
        source,
        "#[derive(Debug, Clone, Copy, PartialEq, Eq)]\nenum ContactState {\n",
        acceptance_helpers
        + "#[derive(Debug, Clone, Copy, PartialEq, Eq)]\nenum ContactState {\n",
        "contact acceptance helpers",
    )
    source = replace_once(
        source,
        "    ContactConfirmed,\n    HardAbort,",
        "    ContactConfirmed,\n    EarlyStall,\n    HardAbort,",
        "early stall state",
    )
    source = replace_once(
        source,
        "    target_samples_seen: u8,\n}",
        "    target_samples_seen: u8,\n    acceptance_low: u16,\n    acceptance_high: u16,\n}",
        "detector acceptance fields",
    )

    detector_constructor = '''    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {
        Self::with_acceptance(
            start_position,
            baseline,
            probe_sign,
            0,
            protocol::MAX_ANGLE_STEP,
        )
    }

    fn new_for_profile(
        start_position: u16,
        baseline: BaselineStats,
        profile: &ContactProfile,
    ) -> Self {
        let (acceptance_low, acceptance_high) = contact_acceptance_bounds(profile);
        Self::with_acceptance(
            start_position,
            baseline,
            profile.probe_sign,
            acceptance_low,
            acceptance_high,
        )
    }

    fn with_acceptance(
        start_position: u16,
        baseline: BaselineStats,
        probe_sign: i8,
        acceptance_low: u16,
        acceptance_high: u16,
    ) -> Self {
        Self {
            start_position,
            previous_position: start_position,
            baseline,
            config: HybridContactConfig::default(),
            probe_sign,
            confirming_samples: 0,
            active_target: None,
            target_samples_seen: 0,
            acceptance_low,
            acceptance_high,
        }
    }

'''
    source = replace_between(
        source,
        "    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {\n",
        "    fn observe(&mut self, observation: MotorObservation, commanded_target: u16) -> ContactState {\n",
        detector_constructor,
        "profile-aware detector constructor",
    )
    source = replace_once(
        source,
        "            if self.confirming_samples >= self.config.persistence_samples {\n                ContactState::ContactConfirmed\n            } else {\n                ContactState::ContactSuspected\n            }",
        "            if self.confirming_samples >= self.config.persistence_samples {\n"
        "                if (self.acceptance_low..=self.acceptance_high)\n"
        "                    .contains(&observation.position)\n"
        "                {\n"
        "                    ContactState::ContactConfirmed\n"
        "                } else {\n"
        "                    ContactState::EarlyStall\n"
        "                }\n"
        "            } else {\n"
        "                ContactState::ContactSuspected\n"
        "            }",
        "early stall detector decision",
    )
    source = replace_once(
        source,
        "        let mut detector =\n            HybridContactDetector::new(start.position, baseline, self.profile.probe_sign);",
        "        let mut detector =\n            HybridContactDetector::new_for_profile(start.position, baseline, &self.profile);",
        "approach profile-aware detector",
    )

    early_stall_arm = '''                    ContactState::EarlyStall => {
                        self.stop_pressure(motor_id, observation.position).await?;
                        let (acceptance_low, acceptance_high) =
                            contact_acceptance_bounds(&self.profile);
                        return Err(format!(
                            "{} early stall outside model contact corridor: target={}, present={}, acceptance={}..={}, URDF={}, guard={}, current={}, threshold={}, velocity={}",
                            self.profile.label,
                            target,
                            observation.position,
                            acceptance_low,
                            acceptance_high,
                            self.profile.urdf_limit_tick,
                            self.profile.guard_tick,
                            observation.current,
                            baseline.contact_threshold(),
                            speed_magnitude(observation.velocity)
                        )
                        .into());
                    }
'''
    source = replace_once(
        source,
        "                    ContactState::HardAbort => {\n",
        early_stall_arm + "                    ContactState::HardAbort => {\n",
        "early stall approach handling",
    )

    replacement_tests = '''#[test]
fn ordered_profile_table_lists_upper_then_lower_then_hip() {
    let profiles = all_profiles().unwrap();
    let lf: Vec<_> = profiles
        .iter()
        .filter(|profile| profile.leg == Leg::Lf)
        .map(|profile| (profile.joint, profile.side))
        .collect();
    assert_eq!(
        lf,
        vec![
            (JointKind::Upper, ContactSide::Min),
            (JointKind::Upper, ContactSide::Max),
            (JointKind::Lower, ContactSide::Min),
            (JointKind::Lower, ContactSide::Max),
            (JointKind::Hip, ContactSide::Min),
            (JointKind::Hip, ContactSide::Max),
        ]
    );
}

#[test]
fn lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers() {
    let minimum = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert_eq!(minimum.motor_id, 11);
    assert_eq!(minimum.probe_sign, 1);
    assert_eq!(minimum.urdf_limit_tick, 3095);
    assert_eq!(minimum.guard_tick, 3159);
    assert_eq!(minimum.baseline_target_tick, 2112);
    assert_eq!(minimum.allowed_motor_ids, &LF_ALLOWED);
    assert!(minimum
        .prerequisites
        .contains(&StaticTarget { motor_id: 42, target_tick: 2389 }));
    assert!(minimum
        .prerequisites
        .contains(&StaticTarget { motor_id: 13, target_tick: 2048 }));
    assert!(minimum
        .prerequisites
        .contains(&StaticTarget { motor_id: 12, target_tick: 3072 }));

    let maximum = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    assert_eq!(maximum.probe_sign, -1);
    assert_eq!(maximum.urdf_limit_tick, 1621);
    assert_eq!(maximum.guard_tick, 1557);
    assert_eq!(maximum.baseline_target_tick, 1984);
}

#[test]
fn hip_prerequisites_are_compact_and_side_specific() {
    let cases = [
        ("LF_HIP_M13_MIN", 12, 3072, 11, 3038),
        ("LF_HIP_M13_MAX", 12, 3015, 11, 3038),
        ("RF_HIP_M23_MIN", 22, 1081, 21, 1058),
        ("RF_HIP_M23_MAX", 22, 1024, 21, 1058),
        ("RH_HIP_M33_MIN", 32, 1024, 31, 1058),
        ("RH_HIP_M33_MAX", 32, 1024, 31, 1058),
        ("LH_HIP_M43_MIN", 42, 3072, 41, 3038),
        ("LH_HIP_M43_MAX", 42, 3072, 41, 3038),
    ];
    for (token, upper_id, upper_tick, lower_id, lower_tick) in cases {
        let profile = profile_for_arm_value(token).unwrap();
        assert!(profile.prerequisites.contains(&StaticTarget {
            motor_id: upper_id,
            target_tick: upper_tick,
        }));
        assert!(profile.prerequisites.contains(&StaticTarget {
            motor_id: lower_id,
            target_tick: lower_tick,
        }));
    }
}

#[test]
fn isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed() {
    let hip = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let error = hardware_profile_allowed(&hip).unwrap_err();
    assert!(error.contains(HIP_HARDWARE_BLOCK_REASON));

    let lower = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert!(hardware_profile_allowed(&lower).is_ok());
}

#[test]
fn contact_acceptance_corridors_match_model_inner_boundary_and_guard() {
    let m12_min = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    assert_eq!(contact_acceptance_bounds(&m12_min), (1387, 1515));
    assert!(position_inside_contact_acceptance(&m12_min, 1443));

    let m12_max = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(contact_acceptance_bounds(&m12_max), (3378, 3506));
    assert!(position_inside_contact_acceptance(&m12_max, 3442));

    let m13_min = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    assert_eq!(contact_acceptance_bounds(&m13_min), (2496, 2624));
    assert!(!position_inside_contact_acceptance(&m13_min, 2405));

    let m11_min = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert_eq!(contact_acceptance_bounds(&m11_min), (3031, 3159));
}

'''
    tests = replace_between(
        tests,
        "#[test]\nfn hip_and_lower_prerequisites_match_geometry_checkpoint() {\n",
        "#[test]\nfn armed_motor_allowlists_are_leg_scoped_and_include_front_parking_joint() {\n",
        replacement_tests,
        "replace obsolete geometry prerequisite test",
    )

    detector_tests = '''
#[test]
fn v19_m13_2405_is_early_stall_not_contact() {
    let profile = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let mut detector = HybridContactDetector::new_for_profile(
        HOME_TICK,
        baseline,
        &profile,
    );
    let target = 2464;
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            detector.observe(observation(2405, 0, 1, target), target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::EarlyStall
    );
}

#[test]
fn detector_confirms_only_persistent_stall_inside_profile_corridor() {
    let profile = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let mut detector = HybridContactDetector::new_for_profile(
        HOME_TICK,
        baseline,
        &profile,
    );
    let target = 2568;
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            detector.observe(observation(2520, 0, 1, target), target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::ContactConfirmed
    );
}
'''
    if "fn v19_m13_2405_is_early_stall_not_contact" in tests:
        raise SystemExit("ordered detector tests already present")
    tests = tests.rstrip() + "\n" + detector_tests

    source_path.write_text(source, encoding="utf-8")
    test_path.write_text(tests, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
