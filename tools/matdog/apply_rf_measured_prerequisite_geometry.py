#!/usr/bin/env python3
"""Apply the RF prerequisite geometry parity correction, then remove this file."""

from pathlib import Path

SOURCE = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TESTS = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")

source = SOURCE.read_text(encoding="utf-8")
tests = TESTS.read_text(encoding="utf-8")

marker = "fn full_sequence_prerequisite_target("
if marker in source:
    print("measured prerequisite geometry correction already present")
    raise SystemExit(0)

old_static = '''fn static_target(leg: Leg, kind: JointKind, q_delta: i16) -> Result<StaticTarget, String> {
    let spec = spec_for(leg, kind);
    Ok(StaticTarget {
        motor_id: spec.motor_id,
        target_tick: spec.tick_for_delta(q_delta)?,
    })
}
'''
new_static = old_static + '''
fn round_ratio_nearest(numerator: i32, denominator: i32) -> Result<i32, String> {
    if denominator <= 0 {
        return Err(format!("invalid affine denominator: {denominator}"));
    }
    if numerator >= 0 {
        Ok((numerator + denominator / 2) / denominator)
    } else {
        Ok(-((-numerator + denominator / 2) / denominator))
    }
}

fn affine_static_target(
    spec: JointSpec,
    calibration: AffineJointCalibration,
    q_delta: i16,
) -> Result<StaticTarget, String> {
    if calibration.motor_id != spec.motor_id || calibration.joint_name != spec.name {
        return Err(format!(
            "affine prerequisite calibration does not match {} M{}",
            spec.name, spec.motor_id
        ));
    }
    if !calibration.accepted {
        return Err(format!(
            "{} M{} affine prerequisite calibration is not accepted",
            spec.name, spec.motor_id
        ));
    }
    let scaled_delta = round_ratio_nearest(
        i32::from(q_delta) * i32::from(calibration.measured_span_ticks),
        i32::from(calibration.expected_span_ticks),
    )?;
    let tick = i32::from(calibration.estimated_zero_tick)
        + i32::from(spec.direction) * scaled_delta;
    let target_tick = u16::try_from(tick)
        .ok()
        .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| {
            format!(
                "{} affine prerequisite target is outside unsigned ST3215 range: {tick}",
                spec.name
            )
        })?;
    Ok(StaticTarget {
        motor_id: spec.motor_id,
        target_tick,
    })
}

fn full_sequence_prerequisite_target(
    leg: Leg,
    kind: JointKind,
    q_delta: i16,
    calibration: AffineJointCalibration,
) -> Result<StaticTarget, String> {
    match leg {
        // LF V25 is frozen and its validated raw prerequisite targets are
        // immutable. RF is still RAM-only, so reproduce the same V25 joint
        // geometry through the affine map measured earlier in this session.
        Leg::Lf => static_target(leg, kind, q_delta),
        Leg::Rf => affine_static_target(*spec_for(leg, kind), calibration, q_delta),
        Leg::Rh | Leg::Lh => Err(format!(
            "{} full-sequence prerequisite geometry is not enabled",
            leg.label()
        )),
    }
}

fn replace_prerequisite_target(
    profile: &mut ContactProfile,
    target: StaticTarget,
) -> Result<(), String> {
    let prerequisite = profile
        .prerequisites
        .iter_mut()
        .find(|candidate| candidate.motor_id == target.motor_id)
        .ok_or_else(|| {
            format!(
                "{} has no prerequisite slot for M{}",
                profile.label, target.motor_id
            )
        })?;
    *prerequisite = target;
    Ok(())
}
'''
if source.count(old_static) != 1:
    raise SystemExit(f"static_target anchor count={source.count(old_static)}")
source = source.replace(old_static, new_static, 1)

old_upper = '''        self.record_lf_contacts(JointKind::Upper, upper_contacts)?;

        self.transition_lf_state(LfSessionState::UpperHorizontal)?;
        self.next_phase(&format!(
            "Transition M{} directly from MAX contact to horizontal hold",
            upper_id
        ))?;
        self.profile = build_profile(leg, JointKind::Lower, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?;
        let upper_horizontal = static_target(leg, JointKind::Upper, UPPER_90_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
        self.move_motor_to(
            upper_id,
            upper_horizontal.target_tick,
            STATIC_TOLERANCE_TICKS,
        )
        .await?;
        self.upsert_held_target(upper_horizontal)?;
'''
new_upper = '''        self.record_lf_contacts(JointKind::Upper, upper_contacts)?;
        let upper_affine = derive_affine_joint_calibration(
            *spec_for(leg, JointKind::Upper),
            upper_contacts,
        );
        if !upper_affine.accepted {
            return Err(format!(
                "{} UPPER affine prerequisite gate rejected before LOWER/HIP geometry",
                leg.label()
            )
            .into());
        }
        let upper_horizontal = full_sequence_prerequisite_target(
            leg,
            JointKind::Upper,
            UPPER_90_DELTA,
            upper_affine,
        )
        .map_err(|message| -> DynError { message.into() })?;
        let mut lower_minimum_profile =
            build_profile(leg, JointKind::Lower, ContactSide::Min)
                .map_err(|message| -> DynError { message.into() })?;
        let mut lower_maximum_profile =
            build_profile(leg, JointKind::Lower, ContactSide::Max)
                .map_err(|message| -> DynError { message.into() })?;
        replace_prerequisite_target(&mut lower_minimum_profile, upper_horizontal)
            .map_err(|message| -> DynError { message.into() })?;
        replace_prerequisite_target(&mut lower_maximum_profile, upper_horizontal)
            .map_err(|message| -> DynError { message.into() })?;

        self.transition_lf_state(LfSessionState::UpperHorizontal)?;
        self.next_phase(&format!(
            "Transition M{} directly from MAX contact to horizontal hold",
            upper_id
        ))?;
        self.profile = lower_minimum_profile.clone();
        self.move_motor_to(
            upper_id,
            upper_horizontal.target_tick,
            STATIC_TOLERANCE_TICKS,
        )
        .await?;
        self.upsert_held_target(upper_horizontal)?;
'''
if source.count(old_upper) != 1:
    raise SystemExit(f"upper transition anchor count={source.count(old_upper)}")
source = source.replace(old_upper, new_upper, 1)

old_lower_pair = '''        let lower_contacts = self
            .measure_lf_joint_pair_efficient(
                build_profile(leg, JointKind::Lower, ContactSide::Min)
                    .map_err(|message| -> DynError { message.into() })?,
                build_profile(leg, JointKind::Lower, ContactSide::Max)
                    .map_err(|message| -> DynError { message.into() })?,
            )
            .await?;
        self.record_lf_contacts(JointKind::Lower, lower_contacts)?;
'''
new_lower_pair = '''        let lower_contacts = self
            .measure_lf_joint_pair_efficient(lower_minimum_profile, lower_maximum_profile)
            .await?;
        self.record_lf_contacts(JointKind::Lower, lower_contacts)?;
        let lower_affine = derive_affine_joint_calibration(
            *spec_for(leg, JointKind::Lower),
            lower_contacts,
        );
        if !lower_affine.accepted {
            return Err(format!(
                "{} LOWER affine prerequisite gate rejected before HIP geometry",
                leg.label()
            )
            .into());
        }
'''
if source.count(old_lower_pair) != 1:
    raise SystemExit(f"lower pair anchor count={source.count(old_lower_pair)}")
source = source.replace(old_lower_pair, new_lower_pair, 1)

old_folded = '''        let folded = static_target(leg, JointKind::Lower, LOWER_FOLDED_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
'''
new_folded = '''        let folded = full_sequence_prerequisite_target(
            leg,
            JointKind::Lower,
            LOWER_FOLDED_DELTA,
            lower_affine,
        )
        .map_err(|message| -> DynError { message.into() })?;
'''
if source.count(old_folded) != 1:
    raise SystemExit(f"folded target anchor count={source.count(old_folded)}")
source = source.replace(old_folded, new_folded, 1)

old_hip_pair = '''        let (hip_minimum_profile, hip_maximum_profile) = full_sequence_hip_profile_pair(leg)
            .map_err(|message| -> DynError { message.into() })?;
        let hip_contacts = self
            .measure_lf_joint_pair_efficient(hip_minimum_profile, hip_maximum_profile)
            .await?;
'''
new_hip_pair = '''        let (mut hip_minimum_profile, mut hip_maximum_profile) =
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
if source.count(old_hip_pair) != 1:
    raise SystemExit(f"hip pair anchor count={source.count(old_hip_pair)}")
source = source.replace(old_hip_pair, new_hip_pair, 1)

unique_test = "rf_measured_affine_prerequisites_reproduce_the_lf_v25_geometry_without_changing_lf"
if unique_test in tests:
    raise SystemExit("measured prerequisite regression tests already present")

tests += r'''

#[test]
fn rf_measured_affine_prerequisites_reproduce_the_lf_v25_geometry_without_changing_lf() {
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let rf_upper_contacts = DualContactResult {
        minimum: ContactResult {
            coarse_scout_tick: 2691,
            first_tick: 2685,
            second_tick: 2686,
            spread_ticks: 1,
            baseline,
        },
        maximum: ContactResult {
            coarse_scout_tick: 667,
            first_tick: 670,
            second_tick: 668,
            spread_ticks: 2,
            baseline,
        },
    };
    let rf_lower_contacts = DualContactResult {
        minimum: ContactResult {
            coarse_scout_tick: 973,
            first_tick: 977,
            second_tick: 977,
            spread_ticks: 0,
            baseline,
        },
        maximum: ContactResult {
            coarse_scout_tick: 2399,
            first_tick: 2392,
            second_tick: 2394,
            spread_ticks: 2,
            baseline,
        },
    };
    let rf_upper_affine = derive_affine_joint_calibration(
        *spec_for(Leg::Rf, JointKind::Upper),
        rf_upper_contacts,
    );
    let rf_lower_affine = derive_affine_joint_calibration(
        *spec_for(Leg::Rf, JointKind::Lower),
        rf_lower_contacts,
    );
    assert!(rf_upper_affine.accepted);
    assert!(rf_lower_affine.accepted);
    assert_eq!(rf_upper_affine.estimated_zero_tick, 2081);
    assert_eq!(rf_lower_affine.estimated_zero_tick, 1983);
    assert_eq!(
        full_sequence_prerequisite_target(
            Leg::Rf,
            JointKind::Upper,
            UPPER_90_DELTA,
            rf_upper_affine,
        )
        .unwrap(),
        StaticTarget {
            motor_id: 22,
            target_tick: 1044,
        }
    );
    assert_eq!(
        full_sequence_prerequisite_target(
            Leg::Rf,
            JointKind::Lower,
            LOWER_FOLDED_DELTA,
            rf_lower_affine,
        )
        .unwrap(),
        StaticTarget {
            motor_id: 21,
            target_tick: 1032,
        }
    );

    let lf_upper_contacts = DualContactResult {
        minimum: ContactResult {
            coarse_scout_tick: 1439,
            first_tick: 1439,
            second_tick: 1439,
            spread_ticks: 0,
            baseline,
        },
        maximum: ContactResult {
            coarse_scout_tick: 3443,
            first_tick: 3443,
            second_tick: 3443,
            spread_ticks: 0,
            baseline,
        },
    };
    let lf_lower_contacts = DualContactResult {
        minimum: ContactResult {
            coarse_scout_tick: 3093,
            first_tick: 3093,
            second_tick: 3093,
            spread_ticks: 0,
            baseline,
        },
        maximum: ContactResult {
            coarse_scout_tick: 1658,
            first_tick: 1658,
            second_tick: 1658,
            spread_ticks: 0,
            baseline,
        },
    };
    let lf_upper_affine = derive_affine_joint_calibration(
        *spec_for(Leg::Lf, JointKind::Upper),
        lf_upper_contacts,
    );
    let lf_lower_affine = derive_affine_joint_calibration(
        *spec_for(Leg::Lf, JointKind::Lower),
        lf_lower_contacts,
    );
    assert_eq!(
        full_sequence_prerequisite_target(
            Leg::Lf,
            JointKind::Upper,
            UPPER_90_DELTA,
            lf_upper_affine,
        )
        .unwrap(),
        static_target(Leg::Lf, JointKind::Upper, UPPER_90_DELTA).unwrap()
    );
    assert_eq!(
        full_sequence_prerequisite_target(
            Leg::Lf,
            JointKind::Lower,
            LOWER_FOLDED_DELTA,
            lf_lower_affine,
        )
        .unwrap(),
        static_target(Leg::Lf, JointKind::Lower, LOWER_FOLDED_DELTA).unwrap()
    );
}

#[test]
fn rf_measured_hip_profiles_remain_one_identical_min_then_max_v25_pose() {
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let upper_contacts = DualContactResult {
        minimum: ContactResult {
            coarse_scout_tick: 2691,
            first_tick: 2685,
            second_tick: 2686,
            spread_ticks: 1,
            baseline,
        },
        maximum: ContactResult {
            coarse_scout_tick: 667,
            first_tick: 670,
            second_tick: 668,
            spread_ticks: 2,
            baseline,
        },
    };
    let lower_contacts = DualContactResult {
        minimum: ContactResult {
            coarse_scout_tick: 973,
            first_tick: 977,
            second_tick: 977,
            spread_ticks: 0,
            baseline,
        },
        maximum: ContactResult {
            coarse_scout_tick: 2399,
            first_tick: 2392,
            second_tick: 2394,
            spread_ticks: 2,
            baseline,
        },
    };
    let upper = full_sequence_prerequisite_target(
        Leg::Rf,
        JointKind::Upper,
        UPPER_90_DELTA,
        derive_affine_joint_calibration(*spec_for(Leg::Rf, JointKind::Upper), upper_contacts),
    )
    .unwrap();
    let lower = full_sequence_prerequisite_target(
        Leg::Rf,
        JointKind::Lower,
        LOWER_FOLDED_DELTA,
        derive_affine_joint_calibration(*spec_for(Leg::Rf, JointKind::Lower), lower_contacts),
    )
    .unwrap();
    let (mut minimum, mut maximum) = full_sequence_hip_profile_pair(Leg::Rf).unwrap();
    for profile in [&mut minimum, &mut maximum] {
        replace_prerequisite_target(profile, upper).unwrap();
        replace_prerequisite_target(profile, lower).unwrap();
    }
    assert_eq!(minimum.side, ContactSide::Min);
    assert_eq!(maximum.side, ContactSide::Max);
    assert_eq!(minimum.prerequisites, maximum.prerequisites);
    assert!(minimum.prerequisites.contains(&upper));
    assert!(minimum.prerequisites.contains(&lower));
}
'''

SOURCE.write_text(source, encoding="utf-8")
TESTS.write_text(tests, encoding="utf-8")
print("measured prerequisite geometry correction applied")
