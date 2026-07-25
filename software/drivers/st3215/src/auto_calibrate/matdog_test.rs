use super::*;

fn motor_state(
    motor_id: u32,
    register_bytes: Vec<u8>,
) -> crate::st3215_proto::inference_state::MotorState {
    crate::st3215_proto::inference_state::MotorState {
        id: motor_id,
        state: register_bytes.into(),
        ..Default::default()
    }
}

fn inference_state(
    bus_serial: &str,
    motors: Vec<crate::st3215_proto::inference_state::MotorState>,
) -> InferenceState {
    InferenceState {
        buses: vec![crate::st3215_proto::inference_state::BusState {
            bus: Some(crate::st3215_proto::St3215Bus {
                serial_number: bus_serial.to_string(),
                ..Default::default()
            }),
            motors,
            ..Default::default()
        }],
        ..Default::default()
    }
}

fn set_register(bytes: &mut [u8], register: RamRegister, value: &[u8]) {
    let address = register.address() as usize;
    bytes[address..address + value.len()].copy_from_slice(value);
}

fn observation(position: u16, velocity: u16, current: u16, goal: u16) -> MotorObservation {
    MotorObservation {
        monotonic_stamp_ns: u64::from(position) + 1,
        position,
        velocity,
        current,
        goal_position: goal,
        torque_limit: TORQUE_LIMIT,
        torque_enabled: true,
        status: 0,
        has_driver_error: false,
    }
}

#[test]
fn exact_matdog_id_set_is_required() {
    assert!(is_exact_matdog_motor_set(&MATDOG_MOTOR_IDS));
    let mut reversed = MATDOG_MOTOR_IDS;
    reversed.reverse();
    assert!(is_exact_matdog_motor_set(&reversed));
    assert!(!is_exact_matdog_motor_set(&MATDOG_MOTOR_IDS[..11]));
    let mut unexpected = MATDOG_MOTOR_IDS;
    unexpected[11] = 44;
    assert!(!is_exact_matdog_motor_set(&unexpected));
}

#[test]
fn profile_table_covers_exactly_24_unique_contacts() {
    let profiles = all_profiles().unwrap();
    assert_eq!(profiles.len(), 24);

    let tokens: BTreeSet<_> = profiles
        .iter()
        .map(|profile| profile.arm_value.as_str())
        .collect();
    assert_eq!(tokens.len(), 24);

    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Upper, JointKind::Hip, JointKind::Lower] {
            for side in [ContactSide::Min, ContactSide::Max] {
                assert!(profiles.iter().any(|profile| {
                    profile.leg == leg && profile.joint == joint && profile.side == side
                }));
            }
        }
    }
}

#[test]
fn validated_m12_min_profile_preserves_hardware_pilot_numbers() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    assert_eq!(profile.motor_id, 12);
    assert_eq!(profile.probe_sign, -1);
    assert_eq!(profile.urdf_limit_tick, 1451);
    assert_eq!(profile.guard_tick, 1387);
    assert_eq!(profile.baseline_target_tick, 1984);
    assert_eq!(profile.allowed_motor_ids, &LF_ALLOWED);

    let parked_lh_upper = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA).unwrap();
    assert!(profile.prerequisites.contains(&parked_lh_upper));
    assert_eq!(parked_lh_upper.motor_id, 42);
    assert_eq!(parked_lh_upper.target_tick, 2389);
}

#[test]
fn directions_transform_q_limits_into_leg_specific_unsigned_ticks() {
    assert_eq!(
        profile_for_arm_value("LF_HIP_M13_MIN")
            .unwrap()
            .urdf_limit_tick,
        2560
    );
    assert_eq!(
        profile_for_arm_value("LF_HIP_M13_MAX")
            .unwrap()
            .urdf_limit_tick,
        1536
    );
    assert_eq!(
        profile_for_arm_value("RF_UPPER_M22_MIN")
            .unwrap()
            .urdf_limit_tick,
        2645
    );
    assert_eq!(
        profile_for_arm_value("RF_UPPER_M22_MAX")
            .unwrap()
            .urdf_limit_tick,
        654
    );
    assert_eq!(
        profile_for_arm_value("RH_LOWER_M31_MIN")
            .unwrap()
            .urdf_limit_tick,
        1001
    );
    assert_eq!(
        profile_for_arm_value("LH_LOWER_M41_MAX")
            .unwrap()
            .urdf_limit_tick,
        1621
    );
}

#[test]
fn every_guard_extends_only_64_ticks_beyond_its_urdf_limit() {
    for profile in all_profiles().unwrap() {
        assert_eq!(
            i32::from(profile.guard_tick) - i32::from(profile.urdf_limit_tick),
            i32::from(profile.probe_sign) * i32::from(GUARD_OVERSHOOT_TICKS)
        );
        assert!(profile.guard_tick <= protocol::MAX_ANGLE_STEP);
        assert_eq!(
            i32::from(profile.baseline_target_tick) - i32::from(HOME_TICK),
            i32::from(profile.probe_sign) * i32::from(BASELINE_TRAVEL_TICKS)
        );
    }
}

#[test]
fn front_profiles_park_only_the_ipsilateral_rear_upper() {
    for profile in all_profiles().unwrap() {
        let has_lh_parking = profile
            .prerequisites
            .iter()
            .any(|target| target.motor_id == 42 && target.target_tick == 2389);
        let has_rh_parking = profile
            .prerequisites
            .iter()
            .any(|target| target.motor_id == 32 && target.target_tick == 1707);
        match profile.leg {
            Leg::Lf => {
                assert!(has_lh_parking);
                assert!(!has_rh_parking);
            }
            Leg::Rf => {
                assert!(has_rh_parking);
                assert!(!has_lh_parking);
            }
            Leg::Rh | Leg::Lh => {
                assert!(!has_lh_parking);
                assert!(!has_rh_parking);
            }
        }
    }
}

#[test]
fn hip_and_lower_prerequisites_match_geometry_checkpoint() {
    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        let hip = build_profile(leg, JointKind::Hip, ContactSide::Min).unwrap();
        let upper_50 = static_target(leg, JointKind::Upper, UPPER_50_DELTA).unwrap();
        assert!(hip.prerequisites.contains(&upper_50));

        let lower = build_profile(leg, JointKind::Lower, ContactSide::Min).unwrap();
        let upper_90 = static_target(leg, JointKind::Upper, UPPER_90_DELTA).unwrap();
        let hip_home = static_target(leg, JointKind::Hip, 0).unwrap();
        assert!(lower.prerequisites.contains(&upper_90));
        assert!(lower.prerequisites.contains(&hip_home));
    }
}

#[test]
fn armed_motor_allowlists_are_leg_scoped_and_include_front_parking_joint() {
    assert_eq!(
        build_profile(Leg::Lf, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &LF_ALLOWED
    );
    assert_eq!(
        build_profile(Leg::Rf, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &RF_ALLOWED
    );
    assert_eq!(
        build_profile(Leg::Rh, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &RH_ALLOWED
    );
    assert_eq!(
        build_profile(Leg::Lh, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &LH_ALLOWED
    );
}

#[test]
fn robust_current_baseline_uses_median_and_mad() {
    let baseline = BaselineStats::from_samples(&[10, 11, 10, 12, 10, 90]).unwrap();
    assert_eq!(baseline.median_current, 11);
    assert_eq!(baseline.mad_current, 1);
    assert_eq!(baseline.contact_threshold(), 16);
}

#[test]
fn direction_generic_detector_confirms_stall_in_both_tick_directions() {
    let baseline = BaselineStats {
        median_current: 1,
        mad_current: 0,
    };

    let mut decreasing = HybridContactDetector::new(HOME_TICK, baseline, -1);
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            decreasing.observe(observation(1470, 0, 1, 1431), 1431),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::ContactSuspected
    );
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::ContactSuspected
    );
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::ContactConfirmed
    );

    let mut increasing = HybridContactDetector::new(HOME_TICK, baseline, 1);
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            increasing.observe(observation(2620, 0, 1, 2660), 2660),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::ContactSuspected
    );
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::ContactSuspected
    );
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::ContactConfirmed
    );
}

#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, -1);
    for position in [2016, 1984, 1952, 1920] {
        assert_ne!(
            detector.observe(observation(position, 25, 40, position - 32), position - 32),
            ContactState::ContactConfirmed
        );
    }
}

#[test]
fn hard_abort_inputs_are_direction_independent() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    for sign in [-1, 1] {
        let mut detector = HybridContactDetector::new(HOME_TICK, baseline, sign);
        let mut status = observation(1984, 0, 20, 1968);
        status.status = 1;
        assert_eq!(detector.observe(status, 1968), ContactState::HardAbort);

        let mut detector = HybridContactDetector::new(HOME_TICK, baseline, sign);
        assert_eq!(
            detector.observe(observation(1984, 0, HARD_CURRENT_ABORT_RAW, 1968), 1968),
            ContactState::HardAbort
        );
    }
}

#[test]
fn wrap_math_is_local_but_goal_targets_remain_unsigned() {
    assert_eq!(circular_distance(4092, 8), 12);
    assert_eq!(signed_tick_delta(8, 4092), 12);
    assert_eq!(signed_tick_delta(4092, 8), -12);
    assert_eq!(advance_tick(2048, -1, 32).unwrap(), 2016);
    assert_eq!(advance_tick(2048, 1, 32).unwrap(), 2080);
    assert!(advance_tick(0, -1, 1).is_err());
    assert!(advance_tick(protocol::MAX_ANGLE_STEP, 1, 1).is_err());
}

#[test]
fn only_ram_motion_registers_are_allowlisted() {
    assert!(is_allowed_matdog_ram_register(RamRegister::TorqueEnable));
    assert!(is_allowed_matdog_ram_register(RamRegister::Acc));
    assert!(is_allowed_matdog_ram_register(RamRegister::GoalPosition));
    assert!(is_allowed_matdog_ram_register(RamRegister::GoalSpeed));
    assert!(is_allowed_matdog_ram_register(RamRegister::TorqueLimit));
    assert!(!is_allowed_matdog_ram_register(RamRegister::Status));
}

#[test]
fn armed_ram_gate_restricts_registers_values_motors_and_goal_windows() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    let allowed = |motor_id, register: RamRegister, value: &[u8]| {
        ram_write_allowed_for_profile(&profile, motor_id, register.address() as u32, value)
    };

    let goal = 1431_u16.to_le_bytes();
    assert!(allowed(12, RamRegister::GoalPosition, &goal));
    assert!(!allowed(11, RamRegister::GoalPosition, &goal));
    assert!(!allowed(
        12,
        RamRegister::GoalPosition,
        &1000_u16.to_le_bytes()
    ));
    assert!(allowed(
        42,
        RamRegister::GoalPosition,
        &2389_u16.to_le_bytes()
    ));
    assert!(!allowed(
        42,
        RamRegister::GoalPosition,
        &3000_u16.to_le_bytes()
    ));
    assert!(allowed(
        12,
        RamRegister::TorqueLimit,
        &TORQUE_LIMIT.to_le_bytes()
    ));
    assert!(!allowed(
        12,
        RamRegister::TorqueLimit,
        &(TORQUE_LIMIT + 1).to_le_bytes()
    ));
    assert!(!allowed(12, RamRegister::Status, &[0]));
}

#[test]
fn front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    let order = prerequisite_restore_order(&profile.prerequisites, profile.motor_id);
    assert_eq!(order, vec![12, 13, 42]);

    let profile = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();
    let order = prerequisite_restore_order(&profile.prerequisites, profile.motor_id);
    assert_eq!(order, vec![22, 23, 32]);
}

#[test]
fn prerequisites_are_unique_and_never_include_the_probe_motor() {
    for profile in all_profiles().unwrap() {
        let ids: BTreeSet<_> = profile
            .prerequisites
            .iter()
            .map(|target| target.motor_id)
            .collect();
        assert_eq!(ids.len(), profile.prerequisites.len());
        assert!(!ids.contains(&profile.motor_id));
        assert!(ids
            .iter()
            .all(|motor_id| profile.allowed_motor_ids.contains(motor_id)));
    }
}

#[test]
fn unsupported_arming_values_are_rejected() {
    assert!(profile_for_arm_value("LF_UPPER_M12_MIN").is_ok());
    assert!(profile_for_arm_value("LF_UPPER_M12_BOTH").is_err());
    assert!(profile_for_arm_value("ALL_24").is_err());
    assert!(profile_for_arm_value("").is_err());
}

#[test]
fn observation_reads_required_live_registers_and_error_state() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    let mut bytes = vec![0; RamRegister::PresentCurrent.address() as usize + 2];
    set_register(&mut bytes, RamRegister::TorqueEnable, &[1]);
    set_register(
        &mut bytes,
        RamRegister::GoalPosition,
        &profile.urdf_limit_tick.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::TorqueLimit,
        &TORQUE_LIMIT.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentPosition,
        &1460_u16.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentSpeed,
        &0x8007_u16.to_le_bytes(),
    );
    set_register(&mut bytes, RamRegister::Status, &[0x04]);
    set_register(
        &mut bytes,
        RamRegister::PresentCurrent,
        &123_u16.to_le_bytes(),
    );

    let mut motor = motor_state(profile.motor_id as u32, bytes);
    motor.monotonic_stamp_ns = 42;
    motor.error = Some(crate::st3215_proto::St3215Error::default());
    let state = inference_state("matdog-bus", vec![motor]);
    let observed = observation_from_state(&state, "matdog-bus", profile.motor_id).unwrap();

    assert_eq!(observed.monotonic_stamp_ns, 42);
    assert_eq!(observed.position, 1460);
    assert_eq!(speed_magnitude(observed.velocity), 7);
    assert_eq!(observed.current, 123);
    assert_eq!(observed.goal_position, profile.urdf_limit_tick);
    assert_eq!(observed.torque_limit, TORQUE_LIMIT);
    assert!(observed.torque_enabled);
    assert_eq!(observed.status, 0x04);
    assert!(observed.has_driver_error);
}

#[test]
fn command_result_and_ram_readback_are_matched_exactly() {
    let command_id = make_command_id(1, 2, 3);
    let mut bytes = vec![0; RamRegister::PresentCurrent.address() as usize + 2];
    set_register(
        &mut bytes,
        RamRegister::GoalPosition,
        &1451_u16.to_le_bytes(),
    );
    let mut motor = motor_state(12, bytes);
    motor.last_command = Some(crate::st3215_proto::InferenceCommandState {
        command: Some(TxEnvelope {
            command_id: command_id.clone(),
            target_bus_serial: "matdog-bus".to_string(),
            ..Default::default()
        }),
        result: CommandResult::CrSuccess as i32,
    });
    let state = inference_state("matdog-bus", vec![motor]);
    let motor = find_motor(&state, "matdog-bus", 12).unwrap();

    assert_eq!(
        command_result_for(&state, "matdog-bus", &command_id),
        Some(CommandResult::CrSuccess as i32)
    );
    assert_eq!(command_result_for(&state, "other-bus", &command_id), None);
    assert_eq!(
        command_result_for(&state, "matdog-bus", &make_command_id(1, 2, 4)),
        None
    );
    assert!(motor_ram_register_matches(
        motor,
        RamRegister::GoalPosition,
        &1451_u16.to_le_bytes()
    ));
    assert!(!motor_ram_register_matches(
        motor,
        RamRegister::GoalPosition,
        &HOME_TICK.to_le_bytes()
    ));
}

#[test]
fn command_ids_are_scoped_and_monotonic() {
    let first = make_command_id(10, 20, 1);
    let second = make_command_id(10, 20, 2);
    let other_run = make_command_id(10, 21, 1);
    assert_eq!(first.len(), 24);
    assert_ne!(first, second);
    assert_ne!(first, other_run);
}

#[test]
fn global_torque_off_cleanup_is_exact() {
    let writes = global_torque_off_writes();
    assert_eq!(writes.len(), MATDOG_MOTOR_IDS.len());
    assert!(is_exact_matdog_motor_set(
        &writes
            .iter()
            .map(|(motor_id, _)| *motor_id)
            .collect::<Vec<_>>()
    ));
    assert!(writes.iter().all(|(_, value)| value.as_slice() == &[0]));
}

#[test]
fn repeatability_uses_circular_distance_and_preserves_unsigned_goals() {
    assert_eq!(repeatability_spread(4092, 8).unwrap(), 12);
    assert_eq!(
        repeatability_spread(1000, 1000 + REPEATABILITY_TOLERANCE_TICKS).unwrap(),
        REPEATABILITY_TOLERANCE_TICKS
    );
    assert!(repeatability_spread(1000, 1001 + REPEATABILITY_TOLERANCE_TICKS).is_err());
}

#[test]
fn canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path() {
    let source = include_str!("matdog.rs");
    for forbidden in [
        "EepromRegister",
        "RamRegister::Lock",
        "ST3215Request::",
        "reg_write: Some",
        "reset: Some",
        "reset_calibration: Some",
        "freeze_calibration: Some",
        "action: Some",
        "Offset.address",
    ] {
        assert!(!source.contains(forbidden), "forbidden token: {forbidden}");
    }
}
