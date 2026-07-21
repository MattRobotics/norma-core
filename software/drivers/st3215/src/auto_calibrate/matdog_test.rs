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
        monotonic_stamp_ns: position as u64 + 1,
        position,
        velocity,
        current,
        goal_position: goal,
        torque_limit: PILOT_TORQUE_LIMIT,
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
    let mut duplicate = MATDOG_MOTOR_IDS.to_vec();
    duplicate.push(43);
    assert!(!is_exact_matdog_motor_set(&duplicate));
}

#[test]
fn inference_motor_ids_reject_out_of_range_aliases() {
    let state = inference_state("matdog-bus", vec![motor_state(267, Vec::new())]);
    assert!(motor_ids_for_bus(&state, "matdog-bus").is_err());
}

#[test]
fn pilot_profile_is_m12_min_unsigned_and_guarded() {
    assert_eq!(PILOT_MOTOR_ID, 12);
    assert_eq!(HOME_TICK, 2048);
    assert_eq!(M12_URDF_MIN_TICK, 1451);
    assert!(M12_MIN_GUARD_TICK < M12_URDF_MIN_TICK);
    assert!(M12_MIN_GUARD_TICK <= protocol::MAX_ANGLE_STEP);
    assert!(COARSE_STEP_TICKS > FINE_STEP_TICKS);
}

#[test]
fn robust_current_baseline_uses_median_and_mad() {
    let baseline = BaselineStats::from_samples(&[10, 11, 10, 12, 10, 90]).unwrap();
    assert_eq!(baseline.median_current, 11);
    assert_eq!(baseline.mad_current, 1);
    assert_eq!(baseline.contact_threshold(), 16);
}

#[test]
fn adaptive_current_threshold_has_noise_floor_and_saturates_safely() {
    let flat = BaselineStats::from_samples(&[12, 12, 12, 12]).unwrap();
    assert_eq!(flat.contact_threshold(), 17);

    let noisy = BaselineStats::from_samples(&[10, 12, 14, 16, 200]).unwrap();
    assert_eq!(noisy.median_current, 14);
    assert_eq!(noisy.mad_current, 2);
    assert_eq!(noisy.contact_threshold(), 22);

    let saturated = BaselineStats {
        median_current: u16::MAX - 1,
        mad_current: 10,
    };
    assert_eq!(saturated.contact_threshold(), u16::MAX);
}

#[test]
fn hybrid_contact_requires_position_velocity_current_and_persistence() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector =
        HybridContactDetector::new(HOME_TICK, baseline, HybridContactConfig::default());

    assert_eq!(
        detector.observe(observation(2016, 20, 12, 1984), 1984),
        ContactState::FreeMotion
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 20, 1968), 1968),
        ContactState::FreeMotion
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 20, 1968), 1968),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 20, 1968), 1968),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 20, 1968), 1968),
        ContactState::ContactConfirmed
    );
}

#[test]
fn current_rise_without_kinematic_stall_does_not_confirm_contact() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector =
        HybridContactDetector::new(HOME_TICK, baseline, HybridContactConfig::default());
    for position in [2016, 1984, 1952, 1920] {
        assert_ne!(
            detector.observe(observation(position, 25, 40, position - 32), position - 32),
            ContactState::ContactConfirmed
        );
    }
}

#[test]
fn hybrid_detector_resets_persistence_after_free_motion() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector =
        HybridContactDetector::new(HOME_TICK, baseline, HybridContactConfig::default());

    assert_eq!(
        detector.observe(observation(1984, 0, 20, 1968), 1968),
        ContactState::FreeMotion
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 20, 1968), 1968),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(1976, 20, 20, 1968), 1968),
        ContactState::FreeMotion
    );
    assert_eq!(
        detector.observe(observation(1976, 0, 20, 1960), 1960),
        ContactState::ContactSuspected
    );
}

#[test]
fn stall_without_current_rise_becomes_ambiguous() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector =
        HybridContactDetector::new(HOME_TICK, baseline, HybridContactConfig::default());
    assert_eq!(
        detector.observe(observation(1984, 0, 12, 1968), 1968),
        ContactState::FreeMotion
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 12, 1968), 1968),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 12, 1968), 1968),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(1984, 0, 12, 1968), 1968),
        ContactState::AmbiguousContact
    );
}

#[test]
fn servo_status_driver_error_torque_loss_and_hard_current_abort() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let config = HybridContactConfig::default();

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    let mut status = observation(1984, 0, 20, 1968);
    status.status = 1;
    assert_eq!(detector.observe(status, 1968), ContactState::HardAbort);

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    let mut driver_error = observation(1984, 0, 20, 1968);
    driver_error.has_driver_error = true;
    assert_eq!(
        detector.observe(driver_error, 1968),
        ContactState::HardAbort
    );

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    let mut torque_lost = observation(1984, 0, 20, 1968);
    torque_lost.torque_enabled = false;
    assert_eq!(detector.observe(torque_lost, 1968), ContactState::HardAbort);

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    assert_eq!(
        detector.observe(observation(1984, 0, HARD_CURRENT_ABORT_RAW, 1968), 1968),
        ContactState::HardAbort
    );

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    let mut torque_limit_changed = observation(1984, 0, 20, 1968);
    torque_limit_changed.torque_limit = PILOT_TORQUE_LIMIT + 1;
    assert_eq!(
        detector.observe(torque_limit_changed, 1968),
        ContactState::HardAbort
    );

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    let stale_goal = observation(1984, 0, 20, 1976);
    assert_eq!(detector.observe(stale_goal, 1968), ContactState::HardAbort);

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    assert_eq!(
        detector.observe(
            observation(2016, 20, HARD_CURRENT_ABORT_RAW - 1, 1984),
            1984
        ),
        ContactState::FreeMotion
    );
}

#[test]
fn repeatability_distance_is_circular_but_goal_position_remains_unsigned() {
    assert_eq!(circular_distance(4092, 8), 12);
    assert_eq!(signed_tick_delta(8, 4092), 12);
    assert_eq!(signed_tick_delta(4092, 8), -12);
    assert_eq!(repeatability_spread(4092, 8), Ok(12));
    assert_eq!(
        repeatability_spread(1000, 1000 + REPEATABILITY_TOLERANCE_TICKS),
        Ok(REPEATABILITY_TOLERANCE_TICKS)
    );
    assert!(repeatability_spread(1000, 1001 + REPEATABILITY_TOLERANCE_TICKS).is_err());
}

#[test]
fn observation_reads_all_required_live_registers_and_error_state() {
    let mut bytes = vec![0; RamRegister::PresentCurrent.address() as usize + 2];
    set_register(&mut bytes, RamRegister::TorqueEnable, &[1]);
    set_register(
        &mut bytes,
        RamRegister::GoalPosition,
        &M12_URDF_MIN_TICK.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::TorqueLimit,
        &PILOT_TORQUE_LIMIT.to_le_bytes(),
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

    let mut motor = motor_state(PILOT_MOTOR_ID as u32, bytes);
    motor.monotonic_stamp_ns = 42;
    motor.error = Some(crate::st3215_proto::St3215Error::default());
    let state = inference_state("matdog-bus", vec![motor]);
    let observed = observation_from_state(&state, "matdog-bus", PILOT_MOTOR_ID).unwrap();

    assert_eq!(observed.monotonic_stamp_ns, 42);
    assert_eq!(observed.position, 1460);
    assert_eq!(speed_magnitude(observed.velocity), 7);
    assert_eq!(observed.current, 123);
    assert_eq!(observed.goal_position, M12_URDF_MIN_TICK);
    assert_eq!(observed.torque_limit, PILOT_TORQUE_LIMIT);
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
        &M12_URDF_MIN_TICK.to_le_bytes(),
    );
    let mut motor = motor_state(PILOT_MOTOR_ID as u32, bytes);
    motor.last_command = Some(crate::st3215_proto::InferenceCommandState {
        command: Some(TxEnvelope {
            command_id: command_id.clone(),
            target_bus_serial: "matdog-bus".to_string(),
            ..Default::default()
        }),
        result: CommandResult::CrSuccess as i32,
    });
    let state = inference_state("matdog-bus", vec![motor]);
    let motor = find_motor(&state, "matdog-bus", PILOT_MOTOR_ID).unwrap();

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
        &M12_URDF_MIN_TICK.to_le_bytes()
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
fn ram_write_allowlist_rejects_lock_and_wrong_sizes() {
    for register in [
        RamRegister::TorqueEnable,
        RamRegister::Acc,
        RamRegister::GoalPosition,
        RamRegister::GoalSpeed,
        RamRegister::TorqueLimit,
    ] {
        assert!(is_allowed_matdog_ram_register(register));
        assert!(validate_ram_write(register, &vec![0; register.size() as usize]).is_ok());
    }
    assert!(!is_allowed_matdog_ram_register(RamRegister::Lock));
    assert!(validate_ram_write(RamRegister::Lock, &[1]).is_err());
    assert!(validate_ram_write(RamRegister::GoalPosition, &[0]).is_err());
}

#[test]
fn global_torque_off_cleanup_is_exact_and_failure_preserving() {
    let writes = global_torque_off_writes();
    assert_eq!(writes.len(), MATDOG_MOTOR_IDS.len());
    assert!(is_exact_matdog_motor_set(
        &writes
            .iter()
            .map(|(motor_id, _)| *motor_id)
            .collect::<Vec<_>>()
    ));
    assert!(writes.iter().all(|(_, value)| value.as_slice() == &[0]));

    let contact = ContactResult {
        first_tick: 1450,
        second_tick: 1452,
        spread_ticks: 2,
        baseline: BaselineStats {
            median_current: 10,
            mad_current: 1,
        },
    };
    assert_eq!(combine_pilot_and_cleanup(Ok(contact), Ok(())), Ok(contact));
    assert_eq!(
        combine_pilot_and_cleanup(Err("pilot".into()), Ok(())),
        Err("pilot".into())
    );
    assert!(
        combine_pilot_and_cleanup(Ok(contact), Err("cleanup".into()))
            .unwrap_err()
            .contains("cleanup failed")
    );
    let both = combine_pilot_and_cleanup(Err("pilot".into()), Err("cleanup".into())).unwrap_err();
    assert!(both.contains("pilot"));
    assert!(both.contains("cleanup"));
}

#[test]
fn matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path() {
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
