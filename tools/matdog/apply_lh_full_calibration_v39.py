#!/usr/bin/env python3
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

    source = replace_exact(
        source,
        'const LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_FULL_V38";\n',
        'const LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_FULL_V38";\n'
        'const LH_FULL_SEQUENCE_ARM_VALUE: &str = "LH_LEG_FULL_V39";\n',
        "LH arm token",
    )

    lf_profile_block = '''fn is_lf_full_sequence(profile: &ContactProfile) -> bool {
    profile.arm_value == LF_FULL_SEQUENCE_ARM_VALUE
        && profile.leg == Leg::Lf
        && profile.allowed_motor_ids == &LF_ALLOWED
}
'''
    profile_block = lf_profile_block + '''
fn lh_full_sequence_profile() -> Result<ContactProfile, String> {
    let mut profile = build_profile(Leg::Lh, JointKind::Upper, ContactSide::Min)?;
    profile.arm_value = LH_FULL_SEQUENCE_ARM_VALUE.to_string();
    profile.label = LH_FULL_SEQUENCE_ARM_VALUE.to_string();
    profile.allowed_motor_ids = &LH_ALLOWED;
    // The canonical 2026-07-20 exact-mesh audit proved that rear-leg paths do
    // not require front-leg parking. All non-active legs remain near HOME.
    profile.prerequisites.clear();
    Ok(profile)
}

fn is_lh_full_sequence(profile: &ContactProfile) -> bool {
    profile.arm_value == LH_FULL_SEQUENCE_ARM_VALUE
        && profile.leg == Leg::Lh
        && profile.allowed_motor_ids == &LH_ALLOWED
}
'''
    source = replace_exact(source, lf_profile_block, profile_block, "LH sentinel profile")

    source = replace_exact(
        source,
        '''pub(crate) fn profile_for_arm_value(value: &str) -> Result<ContactProfile, String> {
    if value == LF_FULL_SEQUENCE_ARM_VALUE {
        return lf_full_sequence_profile();
    }
''',
        '''pub(crate) fn profile_for_arm_value(value: &str) -> Result<ContactProfile, String> {
    if value == LH_FULL_SEQUENCE_ARM_VALUE {
        return lh_full_sequence_profile();
    }
    if value == LF_FULL_SEQUENCE_ARM_VALUE {
        return lf_full_sequence_profile();
    }
''',
        "LH profile lookup",
    )
    source = replace_exact(
        source,
        '''            supported.push(LF_HIP_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LF_FULL_SEQUENCE_ARM_VALUE.to_string());
''',
        '''            supported.push(LF_HIP_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LF_FULL_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LH_FULL_SEQUENCE_ARM_VALUE.to_string());
''',
        "LH supported token",
    )
    source = replace_exact(
        source,
        '''    if profile.joint == JointKind::Hip
        && !is_lf_hip_sequence(profile)
        && !is_lf_full_sequence(profile)
''',
        '''    if profile.joint == JointKind::Hip
        && !is_lf_hip_sequence(profile)
        && !is_lf_full_sequence(profile)
        && !is_lh_full_sequence(profile)
''',
        "LH hardware sentinel allowance",
    )

    lf_goal_block = '''fn lf_full_sequence_goal_allowed(motor_id: u8, target: u16) -> bool {
    if full_sequence_joint_goal_allowed(Leg::Lf, JointKind::Hip, motor_id, target)
        || full_sequence_joint_goal_allowed(Leg::Lf, JointKind::Upper, motor_id, target)
        || full_sequence_joint_goal_allowed(Leg::Lf, JointKind::Lower, motor_id, target)
    {
        return true;
    }

    if motor_id == spec_for(Leg::Lh, JointKind::Upper).motor_id {
        let Ok(parking) = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA) else {
            return false;
        };
        let low = HOME_TICK
            .min(parking.target_tick)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = HOME_TICK
            .max(parking.target_tick)
            .saturating_add(STATIC_TOLERANCE_TICKS)
            .min(protocol::MAX_ANGLE_STEP);
        return (low..=high).contains(&target);
    }

    if MATDOG_MOTOR_IDS.contains(&motor_id) {
        let low = HOME_TICK.saturating_sub(STARTUP_HOME_RECOVERY_LIMIT_TICKS);
        let high = HOME_TICK
            .saturating_add(STARTUP_HOME_RECOVERY_LIMIT_TICKS)
            .min(protocol::MAX_ANGLE_STEP);
        return (low..=high).contains(&target);
    }
    false
}
'''
    goal_block = lf_goal_block + '''
fn lh_full_sequence_goal_allowed(motor_id: u8, target: u16) -> bool {
    if full_sequence_joint_goal_allowed(Leg::Lh, JointKind::Hip, motor_id, target)
        || full_sequence_joint_goal_allowed(Leg::Lh, JointKind::Upper, motor_id, target)
        || full_sequence_joint_goal_allowed(Leg::Lh, JointKind::Lower, motor_id, target)
    {
        return true;
    }

    // Exact-mesh audit: an active rear leg needs no additional cross-leg
    // parking. Every non-LH joint is therefore restricted to bounded HOME.
    if MATDOG_MOTOR_IDS.contains(&motor_id) {
        let low = HOME_TICK.saturating_sub(STARTUP_HOME_RECOVERY_LIMIT_TICKS);
        let high = HOME_TICK
            .saturating_add(STARTUP_HOME_RECOVERY_LIMIT_TICKS)
            .min(protocol::MAX_ANGLE_STEP);
        return (low..=high).contains(&target);
    }
    false
}
'''
    source = replace_exact(source, lf_goal_block, goal_block, "LH union goal gate")
    source = replace_exact(
        source,
        '''fn armed_goal_target_allowed(profile: &ContactProfile, motor_id: u8, target: u16) -> bool {
    if is_lf_full_sequence(profile) {
''',
        '''fn armed_goal_target_allowed(profile: &ContactProfile, motor_id: u8, target: u16) -> bool {
    if is_lh_full_sequence(profile) {
        return lh_full_sequence_goal_allowed(motor_id, target);
    }
    if is_lf_full_sequence(profile) {
''',
        "LH active goal dispatch",
    )

    source = replace_exact(
        source,
        '''    tokio::spawn(async move {
        let result = if is_lf_full_sequence(&profile) {
''',
        '''    tokio::spawn(async move {
        let result = if is_lh_full_sequence(&profile) {
            run_lh_full_calibration(
                profile,
                serial_for_task,
                found_motors,
                comm,
                inference_rx,
                stop_requested,
            )
            .await
        } else if is_lf_full_sequence(&profile) {
''',
        "LH runtime dispatch",
    )

    insertion_anchor = '''struct MatdogRamOnlyCalibrator {
'''
    lh_runtime = r'''async fn place_lh_at_model_zero(
    sentinel: ContactProfile,
    estimates: [ModelZeroEstimate; 3],
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    let mut calibrator = MatdogRamOnlyCalibrator::new(
        sentinel,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    );
    calibrator.total_steps = 7;
    calibrator.publish_progress(
        0,
        "derive and place URDF-consistent LH q=0",
        CalibrationStatus::InProgress,
        None,
    );

    calibrator.next_phase("Verify exact MATDOG ID set before LH model-zero placement")?;
    calibrator.wait_for_exact_motor_set().await?;
    calibrator.next_phase("Verified global torque OFF before LH model-zero placement")?;
    calibrator.global_torque_off_verified().await?;

    for estimate in estimates {
        calibrator.next_phase(&format!(
            "Place M{} {} at calibrated software q=0 tick {}",
            estimate.motor_id, estimate.joint_name, estimate.estimated_zero_tick
        ))?;
        calibrator.prepare_motor(estimate.motor_id).await?;
        calibrator
            .move_motor_to(
                estimate.motor_id,
                estimate.estimated_zero_tick,
                PROBE_HOME_TOLERANCE_TICKS,
            )
            .await?;
        calibrator.held_targets.push(StaticTarget {
            motor_id: estimate.motor_id,
            target_tick: estimate.estimated_zero_tick,
        });
    }

    calibrator.next_phase("Verify all three LH calibrated q=0 holds")?;
    calibrator.verify_static_holds_except(0).await?;
    calibrator.next_phase("Final verified global torque OFF at calibrated LH q=0")?;
    calibrator.global_torque_off_verified().await?;
    calibrator.mark_done();
    Ok(())
}

async fn run_lh_full_calibration(
    sentinel: ContactProfile,
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before LH full calibration".into());
    }
    if !is_lh_full_sequence(&sentinel) {
        return Err("LH full calibration did not receive its exact arm sentinel".into());
    }

    let upper_minimum = execute_contact_stage(
        build_profile(Leg::Lh, JointKind::Upper, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    let upper_maximum = execute_contact_stage(
        build_profile(Leg::Lh, JointKind::Upper, ContactSide::Max)
            .map_err(|message| -> DynError { message.into() })?,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    let lower_minimum = execute_contact_stage(
        build_profile(Leg::Lh, JointKind::Lower, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    let lower_maximum = execute_contact_stage(
        build_profile(Leg::Lh, JointKind::Lower, ContactSide::Max)
            .map_err(|message| -> DynError { message.into() })?,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    let hip_minimum = execute_contact_stage(
        build_profile(Leg::Lh, JointKind::Hip, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    let hip_maximum = execute_contact_stage(
        build_profile(Leg::Lh, JointKind::Hip, ContactSide::Max)
            .map_err(|message| -> DynError { message.into() })?,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;

    let upper_zero = derive_model_zero(
        *spec_for(Leg::Lh, JointKind::Upper),
        DualContactResult {
            minimum: upper_minimum,
            maximum: upper_maximum,
        },
    );
    let lower_zero = derive_model_zero(
        *spec_for(Leg::Lh, JointKind::Lower),
        DualContactResult {
            minimum: lower_minimum,
            maximum: lower_maximum,
        },
    );
    let hip_zero = derive_model_zero(
        *spec_for(Leg::Lh, JointKind::Hip),
        DualContactResult {
            minimum: hip_minimum,
            maximum: hip_maximum,
        },
    );
    let estimates = [hip_zero, upper_zero, lower_zero];

    for estimate in estimates {
        info!(
            "MATDOG V39 model-zero {} M{}: min_contact={}, max_contact={}, zero_from_min={}, zero_from_max={}, disagreement={}, estimated_zero={}, shift_from_2048={}, accepted={}",
            estimate.joint_name,
            estimate.motor_id,
            estimate.minimum_contact_tick,
            estimate.maximum_contact_tick,
            estimate.zero_from_minimum_tick,
            estimate.zero_from_maximum_tick,
            estimate.endpoint_disagreement_ticks,
            estimate.estimated_zero_tick,
            estimate.shift_from_digital_home_ticks,
            estimate.accepted
        );
    }

    let rejected = estimates
        .iter()
        .filter(|estimate| !estimate.accepted)
        .map(|estimate| {
            format!(
                "{} M{} min_zero={} max_zero={} disagreement={} shift={}",
                estimate.joint_name,
                estimate.motor_id,
                estimate.zero_from_minimum_tick,
                estimate.zero_from_maximum_tick,
                estimate.endpoint_disagreement_ticks,
                estimate.shift_from_digital_home_ticks
            )
        })
        .collect::<Vec<_>>();
    if !rejected.is_empty() {
        let message = format!(
            "MODEL_ZERO_INCONSISTENT: {}; limits: endpoint_disagreement<={} ticks, shift_from_2048<={} ticks; LH contacts recorded but calibrated HOME was not applied",
            rejected.join("; "),
            MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS,
            MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS
        );
        let mut failure = MatdogRamOnlyCalibrator::new(
            sentinel,
            target_bus_serial,
            comm,
            inference_rx,
            stop_requested,
        );
        failure.total_steps = 1;
        let cleanup = failure.global_torque_off_verified().await;
        if let Err(cleanup_err) = cleanup {
            let combined = format!("{message}; global torque-OFF also failed: {cleanup_err}");
            failure.mark_failed(&combined);
            return Err(combined.into());
        }
        failure.mark_failed(&message);
        return Err(message.into());
    }

    place_lh_at_model_zero(
        sentinel,
        estimates,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    )
    .await?;
    info!(
        "MATDOG {} complete: M43_q0={}, M42_q0={}, M41_q0={}, RAM-only=true, EEPROM_written=false",
        LH_FULL_SEQUENCE_ARM_VALUE,
        hip_zero.estimated_zero_tick,
        upper_zero.estimated_zero_tick,
        lower_zero.estimated_zero_tick
    );
    Ok(())
}

'''
    source = replace_exact(source, insertion_anchor, lh_runtime + insertion_anchor, "LH runtime")

    test_anchor = '''#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
'''
    new_tests = r'''#[test]
fn lh_full_sequence_is_one_explicit_rear_leg_arm_without_front_parking() {
    let profile = profile_for_arm_value(LH_FULL_SEQUENCE_ARM_VALUE).unwrap();
    assert!(is_lh_full_sequence(&profile));
    assert!(hardware_profile_allowed(&profile).is_ok());
    assert_eq!(profile.allowed_motor_ids, &LH_ALLOWED);
    assert!(profile.prerequisites.is_empty());

    for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {
        for side in [ContactSide::Min, ContactSide::Max] {
            let stage = build_profile(Leg::Lh, joint, side).unwrap();
            assert!(lh_full_sequence_goal_allowed(
                stage.motor_id,
                stage.guard_tick
            ));
            assert!(lh_full_sequence_goal_allowed(stage.motor_id, HOME_TICK));
        }
    }

    // Rear-leg exact-mesh checkpoint: no front-leg parking is required.
    assert!(lh_full_sequence_goal_allowed(12, HOME_TICK));
    assert!(!lh_full_sequence_goal_allowed(12, 2389));
    assert!(!lh_full_sequence_goal_allowed(12, HOME_TICK + 65));
    assert!(!lh_full_sequence_goal_allowed(99, HOME_TICK));
}

#[test]
fn lh_profiles_preserve_horizontal_upper_and_lower_geometry_for_hip() {
    for side in [ContactSide::Min, ContactSide::Max] {
        let profile = build_profile(Leg::Lh, JointKind::Hip, side).unwrap();
        assert!(profile.prerequisites.contains(&StaticTarget {
            motor_id: 42,
            target_tick: 3072,
        }));
        assert!(profile.prerequisites.contains(&StaticTarget {
            motor_id: 41,
            target_tick: 3038,
        }));
        assert!(!profile.prerequisites.iter().any(|target| target.motor_id == 12));
    }
}

#[test]
fn lh_model_zero_solver_recovers_exact_urdf_home_without_scale_fitting() {
    for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {
        let spec = *spec_for(Leg::Lh, joint);
        let minimum = build_profile(Leg::Lh, joint, ContactSide::Min).unwrap();
        let maximum = build_profile(Leg::Lh, joint, ContactSide::Max).unwrap();
        let estimate = derive_model_zero(
            spec,
            DualContactResult {
                minimum: contact_result(minimum.urdf_limit_tick, minimum.urdf_limit_tick),
                maximum: contact_result(maximum.urdf_limit_tick, maximum.urdf_limit_tick),
            },
        );
        assert_eq!(estimate.estimated_zero_tick, HOME_TICK);
        assert_eq!(estimate.endpoint_disagreement_ticks, 0);
        assert!(estimate.accepted);
    }
}

#[test]
fn source_contains_one_click_lh_order_and_model_zero_fail_closed() {
    let source = include_str!("matdog.rs");
    let start = source.find("async fn run_lh_full_calibration(").unwrap();
    let end = source[start..]
        .find("struct MatdogRamOnlyCalibrator")
        .map(|offset| start + offset)
        .unwrap();
    let body = &source[start..end];
    let upper_min = body.find("JointKind::Upper, ContactSide::Min").unwrap();
    let upper_max = body.find("JointKind::Upper, ContactSide::Max").unwrap();
    let lower_min = body.find("JointKind::Lower, ContactSide::Min").unwrap();
    let lower_max = body.find("JointKind::Lower, ContactSide::Max").unwrap();
    let hip_min = body.find("JointKind::Hip, ContactSide::Min").unwrap();
    let hip_max = body.find("JointKind::Hip, ContactSide::Max").unwrap();
    let zero = body.find("derive_model_zero").unwrap();
    let place = body.find("place_lh_at_model_zero").unwrap();
    assert!(upper_min < upper_max);
    assert!(upper_max < lower_min);
    assert!(lower_min < lower_max);
    assert!(lower_max < hip_min);
    assert!(hip_min < hip_max);
    assert!(hip_max < zero && zero < place);
    assert!(body.contains("MODEL_ZERO_INCONSISTENT"));
    assert!(body.contains("EEPROM_written=false"));
}

#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
'''
    tests = replace_exact(tests, test_anchor, new_tests, "LH tests")

    SOURCE.write_text(source, encoding="utf-8")
    TESTS.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
