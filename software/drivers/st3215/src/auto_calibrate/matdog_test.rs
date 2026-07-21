use super::*;

fn observation(
    position: u16,
    velocity: u16,
    current: u16,
    goal: u16,
) -> MotorObservation {
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
    assert!(!is_exact_matdog_motor_set(&MATDOG_MOTOR_IDS[..11]));
    let mut unexpected = MATDOG_MOTOR_IDS;
    unexpected[11] = 44;
    assert!(!is_exact_matdog_motor_set(&unexpected));
    let mut duplicate = MATDOG_MOTOR_IDS.to_vec();
    duplicate.push(43);
    assert!(!is_exact_matdog_motor_set(&duplicate));
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
fn hybrid_contact_requires_position_velocity_current_and_persistence() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector = HybridContactDetector::new(
        HOME_TICK,
        baseline,
        HybridContactConfig::default(),
    );

    assert_eq!(
        detector.observe(observation(2016, 20, 12, 1984), 1984),
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
    let mut detector = HybridContactDetector::new(
        HOME_TICK,
        baseline,
        HybridContactConfig::default(),
    );
    for position in [2016, 1984, 1952, 1920] {
        assert_ne!(
            detector.observe(observation(position, 25, 40, position - 32), position - 32),
            ContactState::ContactConfirmed
        );
    }
}

#[test]
fn stall_without_current_rise_becomes_ambiguous() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector = HybridContactDetector::new(
        HOME_TICK,
        baseline,
        HybridContactConfig::default(),
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
    assert_eq!(detector.observe(driver_error, 1968), ContactState::HardAbort);

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    let mut torque_lost = observation(1984, 0, 20, 1968);
    torque_lost.torque_enabled = false;
    assert_eq!(detector.observe(torque_lost, 1968), ContactState::HardAbort);

    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, config);
    assert_eq!(
        detector.observe(
            observation(1984, 0, HARD_CURRENT_ABORT_RAW, 1968),
            1968
        ),
        ContactState::HardAbort
    );
}

#[test]
fn repeatability_distance_is_circular_but_goal_position_remains_unsigned() {
    assert_eq!(circular_distance(4092, 8), 12);
    assert_eq!(signed_tick_delta(8, 4092), 12);
    assert_eq!(signed_tick_delta(4092, 8), -12);
    assert!(REPEATABILITY_TOLERANCE_TICKS >= circular_distance(4092, 8));
}

#[test]
fn matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path() {
    let source = include_str!("matdog.rs");
    for forbidden in [
        "EepromRegister::",
        "reg_write: Some",
        "reset: Some",
        "freeze_calibration: Some",
        "action: Some",
        "Offset.address",
    ] {
        assert!(!source.contains(forbidden), "forbidden token: {forbidden}");
    }
}
