#!/usr/bin/env python3
"""Add the complete LF -> RF -> RH -> LH one-click calibrator.

V40 is deliberately layered on the independently verified V38 LF and V39 LH
implementations. Every leg acquires six contacts while all joints are restored
to the historical digital HOME between stages. A leg must pass both-endpoint
URDF consistency before the next leg starts. Only after all four legs pass are
all 12 joints moved to their accepted software q=0 targets, followed by a
verified global torque-OFF.
"""

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
        '''const LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_FULL_V38";
const LH_FULL_SEQUENCE_ARM_VALUE: &str = "LH_LEG_FULL_V39";
''',
        '''const LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_FULL_V38";
const LH_FULL_SEQUENCE_ARM_VALUE: &str = "LH_LEG_FULL_V39";
const ALL_LEGS_FULL_SEQUENCE_ARM_VALUE: &str = "MATDOG_ALL_LEGS_FULL_V40";
''',
        "V40 arm token",
    )

    lh_profile_block = '''fn is_lh_full_sequence(profile: &ContactProfile) -> bool {
    profile.arm_value == LH_FULL_SEQUENCE_ARM_VALUE
        && profile.leg == Leg::Lh
        && profile.allowed_motor_ids == &LH_ALLOWED
}
'''
    all_profile_block = lh_profile_block + '''
fn all_legs_full_sequence_profile() -> Result<ContactProfile, String> {
    let mut profile = build_profile(Leg::Lf, JointKind::Upper, ContactSide::Min)?;
    profile.arm_value = ALL_LEGS_FULL_SEQUENCE_ARM_VALUE.to_string();
    profile.label = ALL_LEGS_FULL_SEQUENCE_ARM_VALUE.to_string();
    profile.allowed_motor_ids = &MATDOG_MOTOR_IDS;
    profile.prerequisites.clear();
    Ok(profile)
}

fn is_all_legs_full_sequence(profile: &ContactProfile) -> bool {
    profile.arm_value == ALL_LEGS_FULL_SEQUENCE_ARM_VALUE
        && profile.allowed_motor_ids == &MATDOG_MOTOR_IDS
}
'''
    source = replace_exact(source, lh_profile_block, all_profile_block, "V40 sentinel profile")

    source = replace_exact(
        source,
        '''pub(crate) fn profile_for_arm_value(value: &str) -> Result<ContactProfile, String> {
    if value == LH_FULL_SEQUENCE_ARM_VALUE {
''',
        '''pub(crate) fn profile_for_arm_value(value: &str) -> Result<ContactProfile, String> {
    if value == ALL_LEGS_FULL_SEQUENCE_ARM_VALUE {
        return all_legs_full_sequence_profile();
    }
    if value == LH_FULL_SEQUENCE_ARM_VALUE {
''',
        "V40 profile lookup",
    )
    source = replace_exact(
        source,
        '''            supported.push(LF_HIP_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LF_FULL_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LH_FULL_SEQUENCE_ARM_VALUE.to_string());
''',
        '''            supported.push(LF_HIP_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LF_FULL_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LH_FULL_SEQUENCE_ARM_VALUE.to_string());
            supported.push(ALL_LEGS_FULL_SEQUENCE_ARM_VALUE.to_string());
''',
        "V40 supported token",
    )
    source = replace_exact(
        source,
        '''    if profile.joint == JointKind::Hip
        && !is_lf_hip_sequence(profile)
        && !is_lf_full_sequence(profile)
        && !is_lh_full_sequence(profile)
''',
        '''    if profile.joint == JointKind::Hip
        && !is_lf_hip_sequence(profile)
        && !is_lf_full_sequence(profile)
        && !is_lh_full_sequence(profile)
        && !is_all_legs_full_sequence(profile)
''',
        "V40 hardware sentinel allowance",
    )

    lh_goal_block = '''fn lh_full_sequence_goal_allowed(motor_id: u8, target: u16) -> bool {
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
    all_goal_block = lh_goal_block + '''
fn all_legs_full_sequence_goal_allowed(motor_id: u8, target: u16) -> bool {
    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Hip, JointKind::Upper, JointKind::Lower] {
            if full_sequence_joint_goal_allowed(leg, joint, motor_id, target) {
                return true;
            }
        }
    }
    false
}
'''
    source = replace_exact(source, lh_goal_block, all_goal_block, "V40 union goal gate")
    source = replace_exact(
        source,
        '''fn armed_goal_target_allowed(profile: &ContactProfile, motor_id: u8, target: u16) -> bool {
    if is_lh_full_sequence(profile) {
''',
        '''fn armed_goal_target_allowed(profile: &ContactProfile, motor_id: u8, target: u16) -> bool {
    if is_all_legs_full_sequence(profile) {
        return all_legs_full_sequence_goal_allowed(motor_id, target);
    }
    if is_lh_full_sequence(profile) {
''',
        "V40 active goal dispatch",
    )

    source = replace_exact(
        source,
        '''    tokio::spawn(async move {
        let result = if is_lh_full_sequence(&profile) {
''',
        '''    tokio::spawn(async move {
        let result = if is_all_legs_full_sequence(&profile) {
            run_all_legs_full_calibration(
                profile,
                serial_for_task,
                found_motors,
                comm,
                inference_rx,
                stop_requested,
            )
            .await
        } else if is_lh_full_sequence(&profile) {
''',
        "V40 runtime dispatch",
    )

    insertion_anchor = '''struct MatdogRamOnlyCalibrator {
'''
    runtime = r'''fn leg_code(leg: Leg) -> &'static str {
    match leg {
        Leg::Lf => "LF",
        Leg::Rf => "RF",
        Leg::Rh => "RH",
        Leg::Lh => "LH",
    }
}

async fn execute_leg_full_contacts_v40(
    leg: Leg,
    target_bus_serial: &str,
    comm: &Arc<ST3215BusCommunicator>,
    inference_rx: &watch::Receiver<InferenceState>,
    stop_requested: &Arc<AtomicBool>,
) -> Result<[ModelZeroEstimate; 3], DynError> {
    info!(
        "MATDOG {} begin {} six-contact acquisition",
        ALL_LEGS_FULL_SEQUENCE_ARM_VALUE,
        leg_code(leg)
    );
    let upper_minimum = execute_contact_stage(
        build_profile(leg, JointKind::Upper, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    )
    .await?;
    let upper_maximum = execute_contact_stage(
        build_profile(leg, JointKind::Upper, ContactSide::Max)
            .map_err(|message| -> DynError { message.into() })?,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    )
    .await?;
    let lower_minimum = execute_contact_stage(
        build_profile(leg, JointKind::Lower, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    )
    .await?;
    let lower_maximum = execute_contact_stage(
        build_profile(leg, JointKind::Lower, ContactSide::Max)
            .map_err(|message| -> DynError { message.into() })?,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    )
    .await?;

    let hip_contacts = if leg == Leg::Lf {
        // Preserve the hardware-validated shared LF HIP geometry and one-cycle
        // MIN -> HOME -> MAX implementation.
        execute_lf_hip_stage(target_bus_serial, comm, inference_rx, stop_requested).await?
    } else {
        let minimum = execute_contact_stage(
            build_profile(leg, JointKind::Hip, ContactSide::Min)
                .map_err(|message| -> DynError { message.into() })?,
            target_bus_serial,
            comm,
            inference_rx,
            stop_requested,
        )
        .await?;
        let maximum = execute_contact_stage(
            build_profile(leg, JointKind::Hip, ContactSide::Max)
                .map_err(|message| -> DynError { message.into() })?,
            target_bus_serial,
            comm,
            inference_rx,
            stop_requested,
        )
        .await?;
        DualContactResult { minimum, maximum }
    };

    let estimates = [
        derive_model_zero(*spec_for(leg, JointKind::Hip), hip_contacts),
        derive_model_zero(
            *spec_for(leg, JointKind::Upper),
            DualContactResult {
                minimum: upper_minimum,
                maximum: upper_maximum,
            },
        ),
        derive_model_zero(
            *spec_for(leg, JointKind::Lower),
            DualContactResult {
                minimum: lower_minimum,
                maximum: lower_maximum,
            },
        ),
    ];
    for estimate in estimates {
        info!(
            "MATDOG V40 {} model-zero {} M{}: min_contact={}, max_contact={}, zero_from_min={}, zero_from_max={}, disagreement={}, estimated_zero={}, shift_from_2048={}, accepted={}",
            leg_code(leg),
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
    Ok(estimates)
}

async fn require_leg_model_zero_v40(
    leg: Leg,
    estimates: &[ModelZeroEstimate; 3],
    target_bus_serial: &str,
    comm: &Arc<ST3215BusCommunicator>,
    inference_rx: &watch::Receiver<InferenceState>,
    stop_requested: &Arc<AtomicBool>,
) -> Result<(), DynError> {
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
    if rejected.is_empty() {
        return Ok(());
    }

    let message = format!(
        "MODEL_ZERO_INCONSISTENT {}: {}; the next leg was not started and no calibrated HOME was applied",
        leg_code(leg),
        rejected.join("; ")
    );
    let sentinel = all_legs_full_sequence_profile()
        .map_err(|profile_err| -> DynError { profile_err.into() })?;
    let mut failure = MatdogRamOnlyCalibrator::new(
        sentinel,
        target_bus_serial.to_string(),
        comm.clone(),
        inference_rx.clone(),
        stop_requested.clone(),
    );
    failure.total_steps = 1;
    if let Err(cleanup_err) = failure.global_torque_off_verified().await {
        let combined = format!("{message}; global torque-OFF also failed: {cleanup_err}");
        failure.mark_failed(&combined);
        return Err(combined.into());
    }
    failure.mark_failed(&message);
    Err(message.into())
}

async fn place_all_legs_at_model_zero_v40(
    sentinel: ContactProfile,
    estimates: [ModelZeroEstimate; 12],
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
    calibrator.total_steps = 16;
    calibrator.publish_progress(
        0,
        "place all 12 joints at accepted URDF-consistent software q=0",
        CalibrationStatus::InProgress,
        None,
    );
    calibrator.next_phase("Verify exact MATDOG ID set before 12-joint q=0 placement")?;
    calibrator.wait_for_exact_motor_set().await?;
    calibrator.next_phase("Verified global torque OFF before 12-joint q=0 placement")?;
    calibrator.global_torque_off_verified().await?;

    // The estimate array is [hip, upper, lower] for LF, RF, RH and LH.
    // Move all hips first, then uppers, then lowers. Every target differs from
    // digital HOME by at most 96 ticks and previous targets remain held.
    for index in [0_usize, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11] {
        let estimate = estimates[index];
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

    calibrator.next_phase("Verify all 12 calibrated q=0 holds")?;
    calibrator.verify_static_holds_except(0).await?;
    calibrator.next_phase("Final verified global torque OFF at 12-joint calibrated HOME")?;
    calibrator.global_torque_off_verified().await?;
    calibrator.mark_done();
    Ok(())
}

async fn run_all_legs_full_calibration(
    sentinel: ContactProfile,
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before V40 full calibration".into());
    }
    if !is_all_legs_full_sequence(&sentinel) {
        return Err("V40 full calibration did not receive its exact arm sentinel".into());
    }

    let lf = execute_leg_full_contacts_v40(
        Leg::Lf,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    require_leg_model_zero_v40(
        Leg::Lf,
        &lf,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;

    let rf = execute_leg_full_contacts_v40(
        Leg::Rf,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    require_leg_model_zero_v40(
        Leg::Rf,
        &rf,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;

    let rh = execute_leg_full_contacts_v40(
        Leg::Rh,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    require_leg_model_zero_v40(
        Leg::Rh,
        &rh,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;

    let lh = execute_leg_full_contacts_v40(
        Leg::Lh,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;
    require_leg_model_zero_v40(
        Leg::Lh,
        &lh,
        &target_bus_serial,
        &comm,
        &inference_rx,
        &stop_requested,
    )
    .await?;

    let estimates = [
        lf[0], lf[1], lf[2], rf[0], rf[1], rf[2], rh[0], rh[1], rh[2], lh[0], lh[1], lh[2],
    ];
    place_all_legs_at_model_zero_v40(
        sentinel,
        estimates,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    )
    .await?;
    info!(
        "MATDOG {} complete: 24 contacts, 12 accepted software q=0 targets, RAM-only=true, EEPROM_written=false",
        ALL_LEGS_FULL_SEQUENCE_ARM_VALUE
    );
    Ok(())
}

'''
    source = replace_exact(source, insertion_anchor, runtime + insertion_anchor, "V40 runtime")

    test_anchor = '''#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
'''
    new_tests = r'''#[test]
fn all_legs_v40_is_one_explicit_arm_with_per_joint_guard_union() {
    let profile = profile_for_arm_value(ALL_LEGS_FULL_SEQUENCE_ARM_VALUE).unwrap();
    assert!(is_all_legs_full_sequence(&profile));
    assert!(hardware_profile_allowed(&profile).is_ok());
    assert_eq!(profile.allowed_motor_ids, &MATDOG_MOTOR_IDS);
    assert!(profile.prerequisites.is_empty());

    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Hip, JointKind::Upper, JointKind::Lower] {
            for side in [ContactSide::Min, ContactSide::Max] {
                let stage = build_profile(leg, joint, side).unwrap();
                assert!(all_legs_full_sequence_goal_allowed(
                    stage.motor_id,
                    stage.guard_tick
                ));
                assert!(all_legs_full_sequence_goal_allowed(
                    stage.motor_id,
                    HOME_TICK
                ));
            }
        }
    }
    assert!(!all_legs_full_sequence_goal_allowed(99, HOME_TICK));
}

#[test]
fn all_12_exact_urdf_endpoints_recover_digital_home_without_scale_fitting() {
    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Hip, JointKind::Upper, JointKind::Lower] {
            let spec = *spec_for(leg, joint);
            let minimum = build_profile(leg, joint, ContactSide::Min).unwrap();
            let maximum = build_profile(leg, joint, ContactSide::Max).unwrap();
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
}

#[test]
fn source_orders_v40_lf_rf_rh_lh_and_fails_each_leg_closed() {
    let source = include_str!("matdog.rs");
    let start = source.find("async fn run_all_legs_full_calibration(").unwrap();
    let end = source[start..]
        .find("struct MatdogRamOnlyCalibrator")
        .map(|offset| start + offset)
        .unwrap();
    let body = &source[start..end];
    let lf = body.find("Leg::Lf").unwrap();
    let rf = body[lf + 1..].find("Leg::Rf").map(|i| lf + 1 + i).unwrap();
    let rh = body[rf + 1..].find("Leg::Rh").map(|i| rf + 1 + i).unwrap();
    let lh = body[rh + 1..].find("Leg::Lh").map(|i| rh + 1 + i).unwrap();
    let place = body.find("place_all_legs_at_model_zero_v40").unwrap();
    assert!(lf < rf && rf < rh && rh < lh && lh < place);
    assert_eq!(body.matches("require_leg_model_zero_v40(").count(), 4);
    assert!(body.contains("EEPROM_written=false"));
}

#[test]
fn final_v40_home_moves_hips_then_uppers_then_lowers_and_only_after_24_contacts() {
    let source = include_str!("matdog.rs");
    let place_start = source
        .find("async fn place_all_legs_at_model_zero_v40(")
        .unwrap();
    let run_start = source
        .find("async fn run_all_legs_full_calibration(")
        .unwrap();
    let place = &source[place_start..run_start];
    assert!(place.contains("[0_usize, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]"));
    assert!(place.contains("Verify all 12 calibrated q=0 holds"));
    assert!(place.contains("Final verified global torque OFF"));

    let run_end = source[run_start..]
        .find("struct MatdogRamOnlyCalibrator")
        .map(|offset| run_start + offset)
        .unwrap();
    let run = &source[run_start..run_end];
    assert_eq!(run.matches("execute_leg_full_contacts_v40(").count(), 4);
    assert!(run.find("place_all_legs_at_model_zero_v40").unwrap()
        > run.rfind("execute_leg_full_contacts_v40(").unwrap());
}

#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
'''
    tests = replace_exact(tests, test_anchor, new_tests, "V40 tests")

    SOURCE.write_text(source, encoding="utf-8")
    TESTS.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
