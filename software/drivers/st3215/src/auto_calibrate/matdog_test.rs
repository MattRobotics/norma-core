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
    assert!(minimum.prerequisites.contains(&StaticTarget {
        motor_id: 42,
        target_tick: 2389
    }));
    assert!(minimum.prerequisites.contains(&StaticTarget {
        motor_id: 13,
        target_tick: 2048
    }));
    assert!(minimum.prerequisites.contains(&StaticTarget {
        motor_id: 12,
        target_tick: 3072
    }));

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
fn lf_upper_m12_max_profile_matches_reviewed_geometry() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(profile.motor_id, 12);
    assert_eq!(profile.probe_sign, 1);
    assert_eq!(profile.urdf_limit_tick, 3442);
    assert_eq!(profile.guard_tick, 3506);
    assert_eq!(profile.baseline_target_tick, 2112);
    assert_eq!(profile.allowed_motor_ids, &LF_ALLOWED);

    assert!(profile
        .prerequisites
        .contains(&static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA).unwrap()));
    assert!(profile
        .prerequisites
        .contains(&static_target(Leg::Lf, JointKind::Hip, 0).unwrap()));
    assert!(profile
        .prerequisites
        .contains(&static_target(Leg::Lf, JointKind::Lower, 0).unwrap()));
}

#[test]
fn startup_home_recovery_goal_gate_is_global_but_bounded() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert!(armed_goal_target_allowed(&profile, 22, 2069));
    assert!(armed_goal_target_allowed(
        &profile,
        43,
        HOME_TICK + STARTUP_HOME_RECOVERY_LIMIT_TICKS
    ));
    assert!(!armed_goal_target_allowed(
        &profile,
        22,
        HOME_TICK + STARTUP_HOME_RECOVERY_LIMIT_TICKS + 1
    ));
    assert!(armed_goal_target_allowed(&profile, 42, 2386));
    assert!(armed_goal_target_allowed(&profile, 42, 2389));
    assert!(armed_goal_target_allowed(&profile, 12, 3442));
    assert!(!armed_goal_target_allowed(&profile, 99, HOME_TICK));
}

#[test]
fn startup_recovery_ram_gate_allows_exact_non_profile_home_sequence() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    let allowed = |register: RamRegister, value: &[u8]| {
        ram_write_allowed_for_profile(&profile, 22, register.address() as u32, value)
    };

    assert!(allowed(RamRegister::GoalPosition, &2069_u16.to_le_bytes()));
    assert!(allowed(RamRegister::GoalPosition, &HOME_TICK.to_le_bytes()));
    assert!(allowed(RamRegister::TorqueEnable, &[0]));
    assert!(allowed(RamRegister::TorqueEnable, &[1]));
    assert!(allowed(RamRegister::Acc, &[ACCELERATION]));
    assert!(allowed(RamRegister::GoalSpeed, &GOAL_SPEED.to_le_bytes()));
    assert!(allowed(
        RamRegister::TorqueLimit,
        &TORQUE_LIMIT.to_le_bytes()
    ));

    assert!(!allowed(
        RamRegister::GoalPosition,
        &(HOME_TICK + STARTUP_HOME_RECOVERY_LIMIT_TICKS + 1).to_le_bytes()
    ));
    assert!(!allowed(RamRegister::Acc, &[ACCELERATION + 1]));
}

#[test]
fn startup_v10_pose_classifies_m42_as_valid_prerequisite_residue() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(
        startup_role_for_profile(&profile, 42),
        StartupRole::Prerequisite { target_tick: 2389 }
    );
    assert!(startup_position_allowed(&profile, 42, 2386));
    assert!(!startup_position_allowed(&profile, 42, 2400));

    let mut m42 = observation(2386, 0, 0, 2389);
    m42.torque_enabled = false;
    let home_ready = BTreeSet::new();
    let established = BTreeSet::new();
    assert!(validate_profile_entry_hold(&profile, 42, 22, &home_ready, &established, m42).is_ok());

    m42.torque_enabled = true;
    assert!(validate_profile_entry_hold(&profile, 42, 22, &home_ready, &established, m42).is_err());

    let established = BTreeSet::from([42]);
    assert!(validate_profile_entry_hold(&profile, 42, 22, &home_ready, &established, m42).is_ok());
}

#[test]
fn startup_probe_and_prerequisite_corridors_are_restart_safe() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    for position in [2040, HOME_TICK, 2112, 3000, 3442, 3506, 3516] {
        assert!(
            startup_position_allowed(&profile, 12, position),
            "M12 {position}"
        );
    }
    assert!(!startup_position_allowed(&profile, 12, 3517));

    for position in [2038, HOME_TICK, 2200, 2386, 2389, 2399] {
        assert!(
            startup_position_allowed(&profile, 42, position),
            "M42 {position}"
        );
    }
    assert!(!startup_position_allowed(&profile, 42, 2400));

    assert!(startup_position_allowed(&profile, 22, 2112));
    assert!(!startup_position_allowed(&profile, 22, 2113));
}

#[test]
fn startup_prerequisite_home_endpoint_accepts_observed_m42_2037_without_weakening_target() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    let observed = 2037;
    let observed_error = circular_distance(observed, HOME_TICK);
    assert_eq!(observed_error, 11);
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(STARTUP_PREREQUISITE_HOME_SETTLE_TICKS, 16);
    assert!(observed_error > STATIC_TOLERANCE_TICKS);
    assert!(observed_error <= STARTUP_PREREQUISITE_HOME_SETTLE_TICKS);
    assert_eq!(startup_envelope(&profile, 42), (2032, 2399));
    assert!(startup_position_allowed(&profile, 42, observed));
    assert!(!startup_position_allowed(&profile, 42, 2400));
}

#[test]
fn startup_wrong_profile_residue_is_rejected() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(
        startup_role_for_profile(&profile, 32),
        StartupRole::HomeOnly
    );
    assert!(!startup_position_allowed(&profile, 32, 1707));

    let rf = profile_for_arm_value("RF_UPPER_M22_MAX").unwrap();
    assert_eq!(
        startup_role_for_profile(&rf, 32),
        StartupRole::Prerequisite { target_tick: 1707 }
    );
    assert!(startup_position_allowed(&rf, 32, 1707));
}

#[test]
fn startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks() {
    for profile in all_profiles().unwrap() {
        for motor_id in MATDOG_MOTOR_IDS {
            let role = startup_role_for_profile(&profile, motor_id);
            let expected_bounds = match role {
                StartupRole::Probe => {
                    expanded_linear_bounds(HOME_TICK, profile.guard_tick, STATIC_TOLERANCE_TICKS)
                }
                StartupRole::Prerequisite { target_tick } if target_tick != HOME_TICK => {
                    startup_prerequisite_bounds(target_tick)
                }
                StartupRole::Prerequisite { .. } | StartupRole::HomeOnly => (
                    HOME_TICK.saturating_sub(STARTUP_HOME_RECOVERY_LIMIT_TICKS),
                    HOME_TICK
                        .saturating_add(STARTUP_HOME_RECOVERY_LIMIT_TICKS)
                        .min(protocol::MAX_ANGLE_STEP),
                ),
            };
            assert_eq!(startup_envelope(&profile, motor_id), expected_bounds);
            for position in 0..=protocol::MAX_ANGLE_STEP {
                let expected = (expected_bounds.0..=expected_bounds.1).contains(&position);
                assert_eq!(
                    startup_position_allowed(&profile, motor_id, position),
                    expected,
                    "profile={} M{} position={}",
                    profile.label,
                    motor_id,
                    position
                );
            }
        }
    }
}

#[test]
fn profile_entry_order_is_restart_safe_and_probe_lifecycle_is_strict() {
    let source = include_str!("matdog.rs");
    let run_start = source.find("    async fn run(&mut self)").expect("run");
    let inspect_start = source[run_start..]
        .find("    async fn inspect_profile_entry(")
        .map(|offset| run_start + offset)
        .expect("inspect function");
    let run = &source[run_start..inspect_start];

    let inspect = run.find("Inspect restart-safe profile entry").unwrap();
    let recover = run
        .find("Recover home-only joints to digital home")
        .unwrap();
    let establish = run
        .find("Establish geometry prerequisites from restart-safe state")
        .unwrap();
    let home_probe = run.find("Prime and return probing joint home").unwrap();
    let baseline = run.find("Acquire moving-current baseline").unwrap();
    assert!(
        inspect < recover && recover < establish && establish < home_probe && home_probe < baseline
    );
    assert!(!run.contains("Verify all joints near digital home"));
    assert!(!run.contains("Apply geometry prerequisites one joint at a time"));

    let baseline_start = source
        .find("    async fn acquire_moving_current_baseline(")
        .expect("baseline function");
    let approach_start = source[baseline_start..]
        .find("    async fn approach(")
        .map(|offset| baseline_start + offset)
        .expect("approach function");
    let backoff_start = source[approach_start..]
        .find("    async fn backoff_and_verify(")
        .map(|offset| approach_start + offset)
        .expect("backoff function");
    let baseline_body = &source[baseline_start..approach_start];
    let approach_body = &source[approach_start..backoff_start];
    assert!(baseline_body.contains("self.verify_profile_holds().await?;"));
    assert!(!baseline_body.contains("self.verify_static_holds().await?;"));
    assert!(approach_body.contains("self.verify_profile_holds().await?;"));
    assert!(!approach_body.contains("self.verify_static_holds().await?;"));

    let return_home = run.find("Return probing joint home").unwrap();
    let torque_off = run[return_home..]
        .find("self.set_motor_torque_verified(self.profile.motor_id, false)")
        .map(|offset| return_home + offset)
        .unwrap();
    let restore = run
        .find("Restore prerequisite joints one at a time")
        .unwrap();
    assert!(return_home < torque_off && torque_off < restore);
}

#[test]
fn motion_timeout_covers_observed_m12_max_return() {
    let distance = circular_distance(3327, HOME_TICK);
    assert_eq!(distance, 1279);

    // Hardware V12R measured 958 ticks in the old 12-second window at
    // GOAL_SPEED=80, leaving 321 ticks. The fixed deadline is therefore
    // mathematically insufficient for this valid MAX return.
    assert!(u64::from(distance) > u64::from(GOAL_SPEED) * MOTION_TIMEOUT.as_secs());

    let timeout = motion_timeout_for_distance(distance);
    assert!(timeout > MOTION_TIMEOUT);
    assert!(timeout >= Duration::from_secs(36));
}

#[test]
fn motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile() {
    assert_eq!(motion_timeout_for_distance(64), MOTION_TIMEOUT);

    for profile in all_profiles().unwrap() {
        let distance = circular_distance(profile.guard_tick, HOME_TICK);
        let ideal_ms_at_commanded_speed =
            (u64::from(distance) * 1000 + u64::from(GOAL_SPEED) - 1) / u64::from(GOAL_SPEED);
        assert!(
            motion_timeout_for_distance(distance)
                >= Duration::from_millis(ideal_ms_at_commanded_speed)
                    .saturating_add(MOTION_SETTLE_MARGIN)
        );
    }
}

#[test]
fn probe_home_tolerance_covers_observed_m13_settle_without_weakening_static_gate() {
    let observed_error = circular_distance(2059, HOME_TICK);
    assert_eq!(observed_error, 11);
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(PROBE_HOME_TOLERANCE_TICKS, 16);
    assert!(observed_error > STATIC_TOLERANCE_TICKS);
    assert!(observed_error <= PROBE_HOME_TOLERANCE_TICKS);
}

#[test]
fn probe_home_tolerance_is_scoped_to_exactly_three_active_probe_returns() {
    let source = include_str!("matdog.rs");
    let normalized = source.split_whitespace().collect::<Vec<_>>().join(" ");
    assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 5);
    assert!(normalized.contains(
        "if circular_distance(observation.position, target.target_tick) > STATIC_TOLERANCE_TICKS"
    ));
    assert!(normalized
        .contains("if circular_distance(observation.position, target) <= STATIC_TOLERANCE_TICKS"));
}

#[test]
fn probe_home_handoff_accepts_observed_m11_2062_only_for_probe() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    let observed_error = circular_distance(2062, HOME_TICK);
    assert_eq!(observed_error, 14);
    assert!(observed_error > STATIC_TOLERANCE_TICKS);
    assert!(observed_error <= PROBE_HOME_TOLERANCE_TICKS);
    assert_eq!(home_hold_tolerance(&profile, 11, true), 16);
    assert_eq!(home_hold_tolerance(&profile, 11, false), 10);
    assert_eq!(home_hold_tolerance(&profile, 21, true), 10);
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

#[test]
fn v19_m13_2405_is_early_stall_not_contact() {
    let profile = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let mut detector = HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
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
    let mut detector = HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
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
