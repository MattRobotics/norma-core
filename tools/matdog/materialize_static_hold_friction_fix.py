#!/usr/bin/env python3
"""One-shot source materializer for the reviewed MATDOG V23 hardware findings."""

from pathlib import Path


SOURCE_PATH = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TEST_PATH = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected one marker, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    source = SOURCE_PATH.read_text()
    tests = TEST_PATH.read_text()

    held_speed_gate = '''            let speed = speed_magnitude(observation.velocity);
            if speed > LF_HELD_MAX_SPEED_RAW {
                return Err(format!(
                    "actively-held M{motor_id} moving: speed={speed}, limit={LF_HELD_MAX_SPEED_RAW}"
                ));
            }
'''
    if held_speed_gate in source:
        source = replace_once(source, held_speed_gate, "", "actively-held speed gate")

    startup_speed_gate = '''            if speed_magnitude(observation.velocity) > LF_HELD_MAX_SPEED_RAW {
                return Err(format!(
                    "M{motor_id} moved during startup-home normalization: speed={}, limit={}",
                    speed_magnitude(observation.velocity),
                    LF_HELD_MAX_SPEED_RAW
                )
                .into());
            }

'''
    if startup_speed_gate in source:
        source = replace_once(source, startup_speed_gate, "", "startup static speed gate")

    if "FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS" not in source:
        source = replace_once(
            source,
            "const ADAPTIVE_FINE_SCOUT_TICKS: u16 = 32;\n",
            '''const ADAPTIVE_FINE_SCOUT_TICKS: u16 = 32;
// A fine pass may settle a few ticks before the coarse scout because its
// command increments are smaller. More than one fine step of lag is treated
// as a friction/chamfer plateau: keep advancing in bounded FINE_STEP_TICKS
// until the scout depth is reproduced, a true contact is found, or the
// unchanged model guard stops the pass.
const FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS: u16 = FINE_STEP_TICKS;
''',
            "fine scout lag constant",
        )

    if "fn fine_contact_scout_lag_ticks(" not in source:
        source = replace_once(
            source,
            '''#[cfg(test)]
fn position_inside_adaptive_contact_acceptance(
''',
            '''fn fine_contact_scout_lag_ticks(
    candidate_tick: u16,
    coarse_scout_tick: u16,
    probe_sign: i8,
) -> u16 {
    let signed_lag = i32::from(signed_tick_delta(coarse_scout_tick, candidate_tick))
        * i32::from(probe_sign);
    signed_lag.max(0).min(i32::from(u16::MAX)) as u16
}

fn fine_contact_reproduces_coarse_depth(
    candidate_tick: u16,
    coarse_scout_tick: u16,
    probe_sign: i8,
) -> bool {
    fine_contact_scout_lag_ticks(candidate_tick, coarse_scout_tick, probe_sign)
        <= FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS
}

#[cfg(test)]
fn position_inside_adaptive_contact_acceptance(
''',
            "fine scout helper",
        )

    function_marker = '''    async fn approach_with_scout(
        &mut self,
        step_ticks: u16,
        baseline: BaselineStats,
        coarse_scout_tick: Option<u16>,
    ) -> Result<u16, DynError> {
'''
    function_start = source.index(function_marker)
    loop_index = source.index("        loop {\n", function_start)
    if "'approach_steps: loop {" not in source[function_start : loop_index + 80]:
        source = source[:loop_index] + source[loop_index:].replace(
            "        loop {\n", "        'approach_steps: loop {\n", 1
        )

    if "friction plateau bypass" not in source:
        source = replace_once(
            source,
            '''                    ContactState::ContactConfirmed => {
                        info!(
''',
            '''                    ContactState::ContactConfirmed => {
                        if let Some(scout) = coarse_scout_tick {
                            let scout_lag = fine_contact_scout_lag_ticks(
                                observation.position,
                                scout,
                                self.profile.probe_sign,
                            );
                            if !fine_contact_reproduces_coarse_depth(
                                observation.position,
                                scout,
                                self.profile.probe_sign,
                            ) {
                                info!(
                                    "MATDOG {} friction plateau bypass: target={}, present={}, coarse_scout={}, scout_lag={}, allowed_lag={}, current={}, threshold={}, velocity={}",
                                    self.profile.label,
                                    target,
                                    observation.position,
                                    scout,
                                    scout_lag,
                                    FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS,
                                    observation.current,
                                    baseline.contact_threshold(),
                                    speed_magnitude(observation.velocity),
                                );
                                continue 'approach_steps;
                            }
                        }
                        info!(
''',
            "detector friction bypass",
        )

    if "adaptive friction plateau bypass" not in source:
        source = replace_once(
            source,
            '''                    {
                        info!(
                            "MATDOG {} adaptive kinematic contact: step={}, target={}, present={}, error={}, current={}, scout={}",
''',
            '''                    {
                        let scout_lag = fine_contact_scout_lag_ticks(
                            contact.position,
                            scout,
                            self.profile.probe_sign,
                        );
                        if !fine_contact_reproduces_coarse_depth(
                            contact.position,
                            scout,
                            self.profile.probe_sign,
                        ) {
                            info!(
                                "MATDOG {} adaptive friction plateau bypass: target={}, present={}, coarse_scout={}, scout_lag={}, allowed_lag={}, current={}",
                                self.profile.label,
                                target,
                                contact.position,
                                scout,
                                scout_lag,
                                FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS,
                                contact.current,
                            );
                            continue 'approach_steps;
                        }
                        info!(
                            "MATDOG {} adaptive kinematic contact: step={}, target={}, present={}, error={}, current={}, scout={}",
''',
            "adaptive plateau friction bypass",
        )

    if "actively_held_static_role_uses_position_error_not_instantaneous_speed" not in tests:
        tests = replace_once(
            tests,
            '''#[test]
fn nonparticipating_torque_off_uses_real_position_drift_not_instantaneous_speed() {
''',
            r'''#[test]
fn actively_held_static_role_uses_position_error_not_instantaneous_speed() {
    for motor_id in MATDOG_MOTOR_IDS {
        for velocity in [LF_HELD_MAX_SPEED_RAW + 1, 50, u16::MAX] {
            let observed = observation(HOME_TICK + 1, velocity, 0, HOME_TICK);
            let now_ns = observed.monotonic_stamp_ns + 1;
            assert!(validate_lf_role_observation(
                motor_id,
                observed,
                LfMotorRole::ActivelyHeld {
                    target_tick: HOME_TICK,
                },
                now_ns,
            )
            .is_ok());
        }
    }
}

#[test]
fn actively_held_static_role_remains_fail_closed_on_real_state_errors() {
    for motor_id in MATDOG_MOTOR_IDS {
        let mut torque_off = observation(HOME_TICK, 0, 0, HOME_TICK);
        torque_off.torque_enabled = false;
        let torque_now_ns = torque_off.monotonic_stamp_ns + 1;
        assert!(validate_lf_role_observation(
            motor_id,
            torque_off,
            LfMotorRole::ActivelyHeld {
                target_tick: HOME_TICK,
            },
            torque_now_ns,
        )
        .unwrap_err()
        .contains("torque unexpectedly OFF"));

        let wrong_goal = observation(HOME_TICK, 0, 0, HOME_TICK + 1);
        let goal_now_ns = wrong_goal.monotonic_stamp_ns + 1;
        assert!(validate_lf_role_observation(
            motor_id,
            wrong_goal,
            LfMotorRole::ActivelyHeld {
                target_tick: HOME_TICK,
            },
            goal_now_ns,
        )
        .unwrap_err()
        .contains("goal changed"));

        let drifted = observation(
            HOME_TICK + STATIC_TOLERANCE_TICKS + 1,
            0,
            0,
            HOME_TICK,
        );
        let drift_now_ns = drifted.monotonic_stamp_ns + 1;
        let error = validate_lf_role_observation(
            motor_id,
            drifted,
            LfMotorRole::ActivelyHeld {
                target_tick: HOME_TICK,
            },
            drift_now_ns,
        )
        .unwrap_err();
        assert!(error.contains("actively-held"));
        assert!(error.contains("drifted"));
    }
}

#[test]
fn fine_contact_scout_depth_gate_is_direction_independent_and_bounded() {
    for probe_sign in [-1_i8, 1_i8] {
        let scout = 2000_u16;
        let at_scout = scout;
        let one_step_before = if probe_sign > 0 {
            scout - FINE_STEP_TICKS
        } else {
            scout + FINE_STEP_TICKS
        };
        let too_early = if probe_sign > 0 {
            scout - FINE_STEP_TICKS - 1
        } else {
            scout + FINE_STEP_TICKS + 1
        };
        let beyond_scout = if probe_sign > 0 { scout + 4 } else { scout - 4 };

        assert!(fine_contact_reproduces_coarse_depth(
            at_scout,
            scout,
            probe_sign
        ));
        assert!(fine_contact_reproduces_coarse_depth(
            one_step_before,
            scout,
            probe_sign
        ));
        assert!(!fine_contact_reproduces_coarse_depth(
            too_early,
            scout,
            probe_sign
        ));
        assert!(fine_contact_reproduces_coarse_depth(
            beyond_scout,
            scout,
            probe_sign
        ));
    }

    assert!(fine_contact_reproduces_coarse_depth(1438, 1434, -1));
    assert!(fine_contact_reproduces_coarse_depth(3443, 3446, 1));
    assert!(fine_contact_reproduces_coarse_depth(3093, 3097, 1));
    assert!(!fine_contact_reproduces_coarse_depth(1666, 1652, -1));
}

#[test]
fn nonparticipating_torque_off_uses_real_position_drift_not_instantaneous_speed() {
''',
            "new regression tests",
        )

    # Update the historical simulation that previously expected a speed-only
    # held-role failure. It must now prove that speed alone passes while real
    # position drift still fails.
    old_simulation = '''    bad_goal.goal_position = 2081;
    bad_goal.velocity = LF_HELD_MAX_SPEED_RAW + 1;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_err());
    bad_goal.velocity = 0;
'''
    new_simulation = '''    bad_goal.goal_position = 2081;
    bad_goal.velocity = LF_HELD_MAX_SPEED_RAW + 1;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_ok());
    bad_goal.position = 2081 + STATIC_TOLERANCE_TICKS + 1;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_err());
    bad_goal.position = 2081;
    bad_goal.velocity = 0;
'''
    if old_simulation in tests:
        tests = replace_once(
            tests,
            old_simulation,
            new_simulation,
            "historical held speed simulation",
        )

    for token in (
        "actively-held M{motor_id} moving: speed={speed}",
        "moved during startup-home normalization: speed=",
    ):
        if token in source:
            raise SystemExit(f"obsolete instantaneous static-speed abort remains: {token}")
    for token in (
        "friction plateau bypass",
        "fine_contact_reproduces_coarse_depth",
        "actively_held_static_role_uses_position_error_not_instantaneous_speed",
    ):
        if token not in source + tests:
            raise SystemExit(f"required correction token missing: {token}")

    SOURCE_PATH.write_text(source)
    TEST_PATH.write_text(tests)


if __name__ == "__main__":
    main()
