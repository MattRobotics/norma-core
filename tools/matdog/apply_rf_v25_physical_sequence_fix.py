#!/usr/bin/env python3
from pathlib import Path
import re

SOURCE = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TESTS = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def regex_once(text: str, pattern: str, replacement: str, label: str) -> str:
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one regex match, found {count}")
    return updated


source = SOURCE.read_text(encoding="utf-8")
tests = TESTS.read_text(encoding="utf-8")

# The digital q=0 EEPROM calibration is the canonical kinematic zero. Contact-derived
# affine diagnostics must not redefine the already validated V25 prerequisite pose.
source = regex_once(
    source,
    r"\nfn round_ratio_nearest\(.*?\nfn replace_prerequisite_target\(",
    "\nfn replace_prerequisite_target(",
    "remove incorrect affine prerequisite conversion",
)

old_pair = r"fn full_sequence_hip_profile_pair\(leg: Leg\) -> Result<\(ContactProfile, ContactProfile\), String> \{.*?\n\}\n\nfn is_lf_hip_sequence"
new_pair = '''fn full_sequence_hip_execution_sides(
    leg: Leg,
) -> Result<(ContactSide, ContactSide), String> {
    match leg {
        // LF V25 first moves M13 toward increasing ticks, which is the
        // physically downward contact. RF is mirrored: historical direction
        // validation proves that decreasing M23 ticks moves the hip downward.
        // Therefore RF executes URDF MAX first and URDF MIN second while the
        // state-machine order remains the logical V25 MIN -> MAX sequence.
        Leg::Lf => Ok((ContactSide::Min, ContactSide::Max)),
        Leg::Rf => Ok((ContactSide::Max, ContactSide::Min)),
        Leg::Rh | Leg::Lh => Err(format!(
            "{} full-sequence HIP execution is not enabled",
            leg.label()
        )),
    }
}

fn full_sequence_hip_profile_pair(leg: Leg) -> Result<(ContactProfile, ContactProfile), String> {
    let (first_side, second_side) = full_sequence_hip_execution_sides(leg)?;
    let first = full_sequence_hip_profile(leg, first_side)?;
    let second = full_sequence_hip_profile(leg, second_side)?;
    if first.prerequisites != second.prerequisites {
        return Err(format!(
            "{} HIP sequence changed its V25 parallel prerequisite pose between contacts",
            leg.label()
        ));
    }
    Ok((first, second))
}

fn v25_hip_contacts_from_execution(
    leg: Leg,
    first: ContactResult,
    second: ContactResult,
) -> Result<DualContactResult, String> {
    match leg {
        Leg::Lf => Ok(DualContactResult {
            minimum: first,
            maximum: second,
        }),
        // RF physically mirrors the LF sequence: the first/downward contact is
        // URDF MAX and the second/upward contact is URDF MIN. Reorder only the
        // evidence container so URDF min/max diagnostics remain mathematically
        // correct; execution order is not changed.
        Leg::Rf => Ok(DualContactResult {
            minimum: second,
            maximum: first,
        }),
        Leg::Rh | Leg::Lh => Err(format!(
            "{} V25 HIP contact ordering is not enabled",
            leg.label()
        )),
    }
}

fn is_lf_hip_sequence'''
source = regex_once(source, old_pair, new_pair, "replace HIP execution-pair mapping")

source = replace_once(
    source,
    '''        let upper_horizontal =
            full_sequence_prerequisite_target(leg, JointKind::Upper, UPPER_90_DELTA, upper_affine)
                .map_err(|message| -> DynError { message.into() })?;
''',
    '''        let upper_horizontal = static_target(leg, JointKind::Upper, UPPER_90_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
''',
    "restore raw V25 upper prerequisite",
)

source = replace_once(
    source,
    '''        let folded = full_sequence_prerequisite_target(
            leg,
            JointKind::Lower,
            LOWER_FOLDED_DELTA,
            lower_affine,
        )
        .map_err(|message| -> DynError { message.into() })?;
''',
    '''        let folded = static_target(leg, JointKind::Lower, LOWER_FOLDED_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
''',
    "restore raw V25 lower prerequisite",
)

old_hip_block = '''        let (mut hip_minimum_profile, mut hip_maximum_profile) =
            full_sequence_hip_profile_pair(leg)
                .map_err(|message| -> DynError { message.into() })?;
        for profile in [&mut hip_minimum_profile, &mut hip_maximum_profile] {
            replace_prerequisite_target(profile, upper_horizontal)
                .map_err(|message| -> DynError { message.into() })?;
            replace_prerequisite_target(profile, folded)
                .map_err(|message| -> DynError { message.into() })?;
        }
        if hip_minimum_profile.side != ContactSide::Min
            || hip_maximum_profile.side != ContactSide::Max
            || hip_minimum_profile.prerequisites != hip_maximum_profile.prerequisites
        {
            return Err(format!(
                "{} measured HIP prerequisite pair lost V25 MIN->MAX parity",
                leg.label()
            )
            .into());
        }
        let hip_contacts = self
            .measure_lf_joint_pair_efficient(hip_minimum_profile, hip_maximum_profile)
            .await?;
'''
new_hip_block = '''        let (mut hip_first_profile, mut hip_second_profile) =
            full_sequence_hip_profile_pair(leg)
                .map_err(|message| -> DynError { message.into() })?;
        for profile in [&mut hip_first_profile, &mut hip_second_profile] {
            replace_prerequisite_target(profile, upper_horizontal)
                .map_err(|message| -> DynError { message.into() })?;
            replace_prerequisite_target(profile, folded)
                .map_err(|message| -> DynError { message.into() })?;
        }
        let (expected_first_side, expected_second_side) =
            full_sequence_hip_execution_sides(leg)
                .map_err(|message| -> DynError { message.into() })?;
        if hip_first_profile.side != expected_first_side
            || hip_second_profile.side != expected_second_side
            || hip_first_profile.prerequisites != hip_second_profile.prerequisites
        {
            return Err(format!(
                "{} HIP execution pair lost its reviewed V25 physical order or parallel pose",
                leg.label()
            )
            .into());
        }
        let hip_contacts = self
            .measure_v25_hip_pair_efficient(leg, hip_first_profile, hip_second_profile)
            .await?;
'''
source = replace_once(source, old_hip_block, new_hip_block, "replace HIP state-machine execution")

source = replace_once(
    source,
    '"Move {} HIP M{} from MAX contact to URDF-derived staged q=0",',
    '"Move {} HIP M{} from second V25 contact to URDF-derived staged q=0",',
    "correct final HIP phase description",
)

method_marker = '''    async fn measure_lf_joint_pair_efficient(
'''
new_method = '''    async fn measure_v25_hip_pair_efficient(
        &mut self,
        leg: Leg,
        first_profile: ContactProfile,
        second_profile: ContactProfile,
    ) -> Result<DualContactResult, DynError> {
        let (expected_first_side, expected_second_side) =
            full_sequence_hip_execution_sides(leg)
                .map_err(|message| -> DynError { message.into() })?;
        if first_profile.motor_id != second_profile.motor_id
            || first_profile.joint != JointKind::Hip
            || second_profile.joint != JointKind::Hip
            || first_profile.side != expected_first_side
            || second_profile.side != expected_second_side
            || first_profile.prerequisites != second_profile.prerequisites
        {
            return Err(format!(
                "invalid {} V25 HIP physical execution pair",
                leg.label()
            )
            .into());
        }

        self.remove_held_target(first_profile.motor_id);
        self.transition_lf_state(LfSessionState::HipMin)?;
        self.profile = first_profile;
        info!(
            "MATDOG {} HIP V25 LOGICAL MIN: execute URDF {} first with probe_sign={}",
            leg.label(),
            self.profile.side.label(),
            self.profile.probe_sign
        );
        self.set_lf_active(
            self.profile.motor_id,
            self.latest_observation(self.profile.motor_id)?
                .goal_position,
            LfActiveKind::ContactProbe,
        )?;
        let first = self.measure_lf_contact_side_efficient(None).await?;

        self.stop_pressure(self.profile.motor_id, first.second_tick)
            .await?;
        self.transition_lf_state(LfSessionState::HipMax)?;
        self.profile = second_profile;
        info!(
            "MATDOG {} HIP V25 LOGICAL MAX: execute URDF {} second with probe_sign={}",
            leg.label(),
            self.profile.side.label(),
            self.profile.probe_sign
        );
        self.set_lf_active(
            self.profile.motor_id,
            self.latest_observation(self.profile.motor_id)?
                .goal_position,
            LfActiveKind::ContactProbe,
        )?;
        let second = self
            .measure_lf_contact_side_efficient(Some(first.second_tick))
            .await?;

        v25_hip_contacts_from_execution(leg, first, second)
            .map_err(|message| -> DynError { message.into() })
    }

    async fn measure_lf_joint_pair_efficient(
'''
source = replace_once(source, method_marker, new_method, "insert mirrored V25 HIP executor")

# Replace the three tests introduced by the incorrect affine/ordering patch.
tests = regex_once(
    tests,
    r"#\[test\]\nfn rf_full_sequence_hip_pair_is_v25_min_then_max_with_one_parallel_pose\(\) \{.*\Z",
    '''#[test]
fn rf_full_sequence_hip_pair_executes_downward_encoder_decrease_first() {
    let (first, second) = full_sequence_hip_profile_pair(Leg::Rf).unwrap();
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

    assert_eq!(first.arm_value, RF_HIP_SEQUENCE_ARM_VALUE);
    assert_eq!(second.arm_value, RF_HIP_SEQUENCE_ARM_VALUE);
    assert_eq!(first.prerequisites, expected);
    assert_eq!(second.prerequisites, expected);

    // Historical hardware direction evidence: decreasing M23 moves RF HIP
    // downward. That is the physical equivalent of LF V25 logical MIN and
    // therefore must execute first, even though it is RF URDF MAX.
    assert_eq!(first.side, ContactSide::Max);
    assert_eq!(first.motor_id, 23);
    assert_eq!(first.probe_sign, -1);
    assert_eq!(first.baseline_target_tick, 1984);
    assert_eq!(first.urdf_limit_tick, 1536);
    assert_eq!(first.guard_tick, 1472);
    assert_eq!(contact_acceptance_bounds(&first), (1472, 1600));

    assert_eq!(second.side, ContactSide::Min);
    assert_eq!(second.motor_id, 23);
    assert_eq!(second.probe_sign, 1);
    assert_eq!(second.baseline_target_tick, 2112);
    assert_eq!(second.urdf_limit_tick, 2560);
    assert_eq!(second.guard_tick, 2624);
    assert_eq!(contact_acceptance_bounds(&second), (2496, 2624));
}

#[test]
fn rf_v25_prerequisites_use_validated_raw_digital_zero_geometry() {
    assert_eq!(
        static_target(Leg::Rh, JointKind::Upper, UPPER_30_DELTA).unwrap(),
        StaticTarget {
            motor_id: 32,
            target_tick: 1707,
        }
    );
    assert_eq!(
        static_target(Leg::Rf, JointKind::Upper, UPPER_90_DELTA).unwrap(),
        StaticTarget {
            motor_id: 22,
            target_tick: 1024,
        }
    );
    assert_eq!(
        static_target(Leg::Rf, JointKind::Lower, LOWER_FOLDED_DELTA).unwrap(),
        StaticTarget {
            motor_id: 21,
            target_tick: 1058,
        }
    );
}

#[test]
fn rf_v25_execution_reorders_results_only_for_urdf_evidence() {
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let downward_first_urdf_max = ContactResult {
        coarse_scout_tick: 1538,
        first_tick: 1536,
        second_tick: 1536,
        spread_ticks: 0,
        baseline,
    };
    let upward_second_urdf_min = ContactResult {
        coarse_scout_tick: 2558,
        first_tick: 2560,
        second_tick: 2560,
        spread_ticks: 0,
        baseline,
    };
    let rf = v25_hip_contacts_from_execution(
        Leg::Rf,
        downward_first_urdf_max,
        upward_second_urdf_min,
    )
    .unwrap();
    assert_eq!(rf.minimum.first_tick, 2560);
    assert_eq!(rf.maximum.first_tick, 1536);

    let lf = v25_hip_contacts_from_execution(
        Leg::Lf,
        upward_second_urdf_min,
        downward_first_urdf_max,
    )
    .unwrap();
    assert_eq!(lf.minimum.first_tick, 2560);
    assert_eq!(lf.maximum.first_tick, 1536);
}
''',
    "replace incorrect RF affine/order tests",
)

for forbidden in (
    "fn round_ratio_nearest(",
    "fn affine_static_target(",
    "full_sequence_prerequisite_target(",
    "target_tick: 1044",
    "target_tick: 1032",
):
    if forbidden in source or forbidden in tests:
        raise SystemExit(f"forbidden stale implementation remains: {forbidden}")

required_source = (
    "Leg::Rf => Ok((ContactSide::Max, ContactSide::Min))",
    "measure_v25_hip_pair_efficient",
    "v25_hip_contacts_from_execution",
    "static_target(leg, JointKind::Upper, UPPER_90_DELTA)",
    "static_target(leg, JointKind::Lower, LOWER_FOLDED_DELTA)",
)
for token in required_source:
    if token not in source:
        raise SystemExit(f"missing corrected source token: {token}")

SOURCE.write_text(source, encoding="utf-8")
TESTS.write_text(tests, encoding="utf-8")
print("RF V25 physical-sequence correction applied")
