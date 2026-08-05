#!/usr/bin/env python3
"""Temporary source applicator for the RF V25 guard discretization regression."""

from pathlib import Path

SOURCE = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TESTS = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")

source = SOURCE.read_text(encoding="utf-8")
tests = TESTS.read_text(encoding="utf-8")

if "fn next_guard_bounded_target(" in source:
    print("canonical guard repair already present")
    raise SystemExit(0)

old_loop = '''            let next_target = advance_tick(target, self.profile.probe_sign, step_ticks)?;
            if passed_guard(
                next_target,
                self.profile.guard_tick,
                self.profile.probe_sign,
            ) {
                return Err(format!(
                    "{} travel guard reached without contact: next={}, URDF={}, guard={}",
                    self.profile.label,
                    next_target,
                    self.profile.urdf_limit_tick,
                    self.profile.guard_tick
                )
                .into());
            }
            self.set_motor_goal_verified(motor_id, next_target).await?;
'''
new_loop = '''            let Some(next_target) = next_guard_bounded_target(
                target,
                self.profile.probe_sign,
                step_ticks,
                self.profile.guard_tick,
            )? else {
                return Err(format!(
                    "{} travel guard reached without contact: current={}, URDF={}, guard={}",
                    self.profile.label,
                    target,
                    self.profile.urdf_limit_tick,
                    self.profile.guard_tick
                )
                .into());
            };
            self.set_motor_goal_verified(motor_id, next_target).await?;
'''
if source.count(old_loop) != 1:
    raise SystemExit(f"canonical approach loop count={source.count(old_loop)}")
source = source.replace(old_loop, new_loop, 1)

old_helper = '''fn advance_tick(value: u16, sign: i8, amount: u16) -> Result<u16, DynError> {
    let next = i32::from(value) + i32::from(sign) * i32::from(amount);
    u16::try_from(next)
        .ok()
        .filter(|tick| *tick <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| format!("unsigned GoalPosition out of range: {next}").into())
}

fn motion_timeout_for_distance'''
new_helper = '''fn advance_tick(value: u16, sign: i8, amount: u16) -> Result<u16, DynError> {
    let next = i32::from(value) + i32::from(sign) * i32::from(amount);
    u16::try_from(next)
        .ok()
        .filter(|tick| *tick <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| format!("unsigned GoalPosition out of range: {next}").into())
}

// Preserve the reviewed V25 guard exactly. Fixed coarse/fine increments may
// not divide the remaining travel evenly; the final command lands on the
// existing guard instead of skipping from the last safe target beyond it.
fn next_guard_bounded_target(
    value: u16,
    sign: i8,
    amount: u16,
    guard: u16,
) -> Result<Option<u16>, DynError> {
    if passed_guard(value, guard, sign) {
        return Err(format!("current target {value} is already beyond guard {guard}").into());
    }
    if value == guard {
        return Ok(None);
    }
    let next = i32::from(value) + i32::from(sign) * i32::from(amount);
    let candidate = u16::try_from(next)
        .ok()
        .filter(|tick| *tick <= protocol::MAX_ANGLE_STEP);
    if candidate
        .map(|tick| passed_guard(tick, guard, sign))
        .unwrap_or(true)
    {
        Ok(Some(guard))
    } else {
        Ok(candidate)
    }
}

fn motion_timeout_for_distance'''
if source.count(old_helper) != 1:
    raise SystemExit(f"advance helper count={source.count(old_helper)}")
source = source.replace(old_helper, new_helper, 1)

tests += r'''

#[test]
fn guard_bounded_final_step_is_direction_symmetric_and_never_extends_the_guard() {
    assert_eq!(next_guard_bounded_target(1420, -1, COARSE_STEP_TICKS, 1387).unwrap(), Some(1387));
    assert_eq!(next_guard_bounded_target(2676, 1, COARSE_STEP_TICKS, 2709).unwrap(), Some(2709));
    assert_eq!(next_guard_bounded_target(1387, -1, COARSE_STEP_TICKS, 1387).unwrap(), None);
    assert_eq!(next_guard_bounded_target(2709, 1, COARSE_STEP_TICKS, 2709).unwrap(), None);
    assert_eq!(next_guard_bounded_target(1984, -1, COARSE_STEP_TICKS, 1387).unwrap(), Some(1920));
    assert_eq!(next_guard_bounded_target(2112, 1, COARSE_STEP_TICKS, 2709).unwrap(), Some(2176));
}

#[test]
fn rf_m22_real_trace_confirms_contact_at_the_existing_guard_without_widening_it() {
    let profile = profile_for_arm_value("RF_UPPER_M22_MIN").unwrap();
    assert_eq!((profile.urdf_limit_tick, profile.guard_tick, profile.probe_sign), (2645, 2709, 1));
    let baseline = BaselineStats { median_current: 0, mad_current: 0 };
    let mut detector = HybridContactDetector::new_for_profile(2112, baseline, &profile);
    let target = profile.guard_tick;
    let sample = observation(2667, 0, 1, target);
    assert_eq!(detector.observe(sample, target), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(detector.observe(sample, target), ContactState::FreeMotion);
    }
    assert_eq!(detector.observe(sample, target), ContactState::ContactSuspected);
    assert_eq!(detector.observe(sample, target), ContactState::ContactSuspected);
    assert_eq!(detector.observe(sample, target), ContactState::ContactConfirmed);
}

#[test]
fn lf_m12_mirror_uses_the_same_existing_guard_rule() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    assert_eq!((profile.urdf_limit_tick, profile.guard_tick, profile.probe_sign), (1451, 1387, -1));
    let baseline = BaselineStats { median_current: 0, mad_current: 0 };
    let mut detector = HybridContactDetector::new_for_profile(1984, baseline, &profile);
    let target = profile.guard_tick;
    let sample = observation(1434, 0, 1, target);
    assert_eq!(detector.observe(sample, target), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(detector.observe(sample, target), ContactState::FreeMotion);
    }
    assert_eq!(detector.observe(sample, target), ContactState::ContactSuspected);
    assert_eq!(detector.observe(sample, target), ContactState::ContactSuspected);
    assert_eq!(detector.observe(sample, target), ContactState::ContactConfirmed);
}
'''

SOURCE.write_text(source, encoding="utf-8")
TESTS.write_text(tests, encoding="utf-8")
print("canonical guard repair applied")
