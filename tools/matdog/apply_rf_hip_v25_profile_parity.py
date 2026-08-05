#!/usr/bin/env python3
from pathlib import Path

SOURCE = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TESTS = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")

source = SOURCE.read_text()
tests = TESTS.read_text()

constant_anchor = '''const LF_HIP_SEQUENCE_ARM_VALUE: &str = "LF_HIP_M13_MIN_MAX";\nconst LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_STATE_MACHINE";'''
constant_replacement = '''const LF_HIP_SEQUENCE_ARM_VALUE: &str = "LF_HIP_M13_MIN_MAX";\nconst RF_HIP_SEQUENCE_ARM_VALUE: &str = "RF_HIP_M23_MIN_MAX";\nconst LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_STATE_MACHINE";'''
if constant_anchor not in source:
    raise SystemExit("constant anchor not found")
source = source.replace(constant_anchor, constant_replacement, 1)

function_anchor = '''fn is_lf_hip_sequence(profile: &ContactProfile) -> bool {\n    profile.arm_value == LF_HIP_SEQUENCE_ARM_VALUE\n        && profile.leg == Leg::Lf\n        && profile.joint == JointKind::Hip\n        && profile.motor_id == 13\n}\n'''
function_insert = '''fn full_sequence_hip_profile(\n    leg: Leg,\n    side: ContactSide,\n) -> Result<ContactProfile, String> {\n    if leg == Leg::Lf {\n        // Preserve the hardware-validated LF V25 profile byte-for-byte.\n        return lf_hip_sequence_profile(side);\n    }\n    if leg != Leg::Rf {\n        return Err(format!(\n            "{} full-sequence HIP profile is not enabled",\n            leg.label()\n        ));\n    }\n\n    let parking_leg = leg\n        .parking_leg()\n        .ok_or_else(|| format!("{} has no reviewed parking leg", leg.label()))?;\n    let mut profile = build_profile(leg, JointKind::Hip, side)?;\n    profile.arm_value = RF_HIP_SEQUENCE_ARM_VALUE.to_string();\n    profile.label = RF_HIP_SEQUENCE_ARM_VALUE.to_string();\n    profile.prerequisites = vec![\n        static_target(parking_leg, JointKind::Upper, UPPER_30_DELTA)?,\n        static_target(leg, JointKind::Upper, UPPER_90_DELTA)?,\n        static_target(leg, JointKind::Lower, LOWER_FOLDED_DELTA)?,\n    ];\n    Ok(profile)\n}\n\nfn full_sequence_hip_profile_pair(\n    leg: Leg,\n) -> Result<(ContactProfile, ContactProfile), String> {\n    let minimum = full_sequence_hip_profile(leg, ContactSide::Min)?;\n    let maximum = full_sequence_hip_profile(leg, ContactSide::Max)?;\n    if minimum.side != ContactSide::Min || maximum.side != ContactSide::Max {\n        return Err(format!("{} HIP sequence order is not MIN then MAX", leg.label()));\n    }\n    if minimum.prerequisites != maximum.prerequisites {\n        return Err(format!(\n            "{} HIP sequence changed its V25 parallel prerequisite pose between MIN and MAX",\n            leg.label()\n        ));\n    }\n    Ok((minimum, maximum))\n}\n\nfn is_lf_hip_sequence(profile: &ContactProfile) -> bool {\n    profile.arm_value == LF_HIP_SEQUENCE_ARM_VALUE\n        && profile.leg == Leg::Lf\n        && profile.joint == JointKind::Hip\n        && profile.motor_id == 13\n}\n'''
if function_anchor not in source:
    raise SystemExit("HIP function anchor not found")
source = source.replace(function_anchor, function_insert, 1)

run_anchor = '''        let hip_contacts = self\n            .measure_lf_joint_pair_efficient(\n                build_profile(leg, JointKind::Hip, ContactSide::Min)\n                    .map_err(|message| -> DynError { message.into() })?,\n                build_profile(leg, JointKind::Hip, ContactSide::Max)\n                    .map_err(|message| -> DynError { message.into() })?,\n            )\n            .await?;'''
run_replacement = '''        let (hip_minimum_profile, hip_maximum_profile) =\n            full_sequence_hip_profile_pair(leg)\n                .map_err(|message| -> DynError { message.into() })?;\n        let hip_contacts = self\n            .measure_lf_joint_pair_efficient(hip_minimum_profile, hip_maximum_profile)\n            .await?;'''
if run_anchor not in source:
    raise SystemExit("full-sequence HIP call anchor not found")
source = source.replace(run_anchor, run_replacement, 1)

new_tests = r'''

#[test]
fn lf_full_sequence_hip_pair_remains_exact_v25() {
    let (minimum, maximum) = full_sequence_hip_profile_pair(Leg::Lf).unwrap();
    assert_eq!(minimum, lf_hip_sequence_profile(ContactSide::Min).unwrap());
    assert_eq!(maximum, lf_hip_sequence_profile(ContactSide::Max).unwrap());
}

#[test]
fn rf_full_sequence_hip_pair_is_v25_min_then_max_with_one_parallel_pose() {
    let (minimum, maximum) = full_sequence_hip_profile_pair(Leg::Rf).unwrap();
    let expected = vec![
        StaticTarget {
            motor_id: 32,
            target_tick: 1707,
        },
        StaticTarget {
            motor_id: 22,
            target_tick: 1024,
        },
        StaticTarget {
            motor_id: 21,
            target_tick: 1058,
        },
    ];

    assert_eq!(minimum.side, ContactSide::Min);
    assert_eq!(maximum.side, ContactSide::Max);
    assert_eq!(minimum.arm_value, RF_HIP_SEQUENCE_ARM_VALUE);
    assert_eq!(maximum.arm_value, RF_HIP_SEQUENCE_ARM_VALUE);
    assert_eq!(minimum.prerequisites, expected);
    assert_eq!(maximum.prerequisites, expected);

    assert_eq!(minimum.motor_id, 23);
    assert_eq!(minimum.probe_sign, 1);
    assert_eq!(minimum.urdf_limit_tick, 2560);
    assert_eq!(minimum.guard_tick, 2624);
    assert_eq!(contact_acceptance_bounds(&minimum), (2496, 2624));

    assert_eq!(maximum.motor_id, 23);
    assert_eq!(maximum.probe_sign, -1);
    assert_eq!(maximum.urdf_limit_tick, 1536);
    assert_eq!(maximum.guard_tick, 1472);
    assert_eq!(contact_acceptance_bounds(&maximum), (1472, 1600));
}
'''
if "rf_full_sequence_hip_pair_is_v25_min_then_max_with_one_parallel_pose" in tests:
    raise SystemExit("tests already patched")
tests += new_tests

SOURCE.write_text(source)
TESTS.write_text(tests)
print("RF HIP full-sequence V25 profile parity patch applied")
