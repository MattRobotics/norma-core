#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def replace_function(text: str, signature: str, next_signature: str, transform, label: str) -> str:
    start = text.find(signature)
    if start < 0:
        raise SystemExit(f"{label}: start signature not found")
    end = text.find(next_signature, start)
    if end < 0:
        raise SystemExit(f"{label}: end signature not found")
    body = text[start:end]
    new_body = transform(body)
    if new_body == body:
        raise SystemExit(f"{label}: transform made no change")
    return text[:start] + new_body + text[end:]


def patch_move_function(body: str, label: str) -> str:
    old = """        let mut last_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let deadline = Instant::now() + MOTION_TIMEOUT;
"""
    new = """        let start = self.latest_observation(motor_id)?;
        let mut last_stamp = start.monotonic_stamp_ns;
        let distance_ticks = circular_distance(start.position, target);
        let motion_timeout = motion_timeout_for_distance(distance_ticks);
        let deadline = Instant::now() + motion_timeout;
        info!(
            \"MATDOG {} move plan: M{} start={} target={} distance={} timeout_ms={}\",
            self.profile.label,
            motor_id,
            start.position,
            target,
            distance_ticks,
            motion_timeout.as_millis()
        );
"""
    if body.count(old) != 1:
        raise SystemExit(f"{label}: fixed-deadline anchor count={body.count(old)}")
    body = body.replace(old, new, 1)

    patterns = (
        (
            '"M{motor_id} profile-entry timeout: target={target}, present={}, error={}"',
            '"M{motor_id} profile-entry timeout: target={target}, present={}, error={}, distance={}, timeout_ms={}"',
        ),
        (
            '"M{motor_id} target timeout: target={target}, present={}, error={}"',
            '"M{motor_id} target timeout: target={target}, present={}, error={}, distance={}, timeout_ms={}"',
        ),
    )
    replaced = False
    for old_message, new_message in patterns:
        if old_message in body:
            body = body.replace(old_message, new_message, 1)
            replaced = True
            break
    if not replaced:
        raise SystemExit(f"{label}: timeout message was not upgraded")

    arg_old = """            circular_distance(last.position, target)
        )
"""
    arg_new = """            circular_distance(last.position, target),
            distance_ticks,
            motion_timeout.as_millis()
        )
"""
    if body.count(arg_old) != 1:
        raise SystemExit(f"{label}: timeout args anchor count={body.count(arg_old)}")
    return body.replace(arg_old, arg_new, 1)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: apply_motion_timeout_v13.py MATDOG_RS MATDOG_TEST_RS")
    source_path = Path(sys.argv[1])
    tests_path = Path(sys.argv[2])
    source = source_path.read_text(encoding="utf-8")
    tests = tests_path.read_text(encoding="utf-8")

    constants_anchor = "const MOTION_TIMEOUT: Duration = Duration::from_secs(12);\n"
    constants = """const MOTION_TIMEOUT: Duration = Duration::from_secs(12);
// Long MAX returns and +90-degree prerequisites can exceed the original fixed
// 12-second budget at GOAL_SPEED=80. Size the deadline from the commanded
// distance using a conservative half-speed floor, retaining 12 seconds as the
// minimum for short movements and telemetry/settling overhead.
const MIN_EXPECTED_MOTION_TICKS_PER_SECOND: u64 = 40;
const MOTION_SETTLE_MARGIN: Duration = Duration::from_secs(5);
"""
    source = replace_once(source, constants_anchor, constants, "motion timeout constants")

    helper_anchor = "fn passed_guard(value: u16, guard: u16, sign: i8) -> bool {\n"
    helper = """fn motion_timeout_for_distance(distance_ticks: u16) -> Duration {
    let travel_ms = u64::from(distance_ticks)
        .saturating_mul(1000)
        .saturating_add(MIN_EXPECTED_MOTION_TICKS_PER_SECOND - 1)
        / MIN_EXPECTED_MOTION_TICKS_PER_SECOND;
    Duration::from_millis(travel_ms)
        .saturating_add(MOTION_SETTLE_MARGIN)
        .max(MOTION_TIMEOUT)
}

"""
    source = replace_once(source, helper_anchor, helper + helper_anchor, "distance timeout helper")

    source = replace_function(
        source,
        "    async fn move_profile_entry_motor_to_target(\n",
        "    async fn verify_profile_entry_holds_except(\n",
        lambda body: patch_move_function(body, "profile-entry move"),
        "profile-entry move",
    )
    source = replace_function(
        source,
        "    async fn move_motor_to(\n",
        "    async fn verify_profile_holds(\n",
        lambda body: patch_move_function(body, "ordinary move"),
        "ordinary move",
    )

    test_anchor = """#[test]
fn robust_current_baseline_uses_median_and_mad() {
"""
    new_tests = r'''#[test]
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
        let ideal_ms_at_commanded_speed = (u64::from(distance) * 1000
            + u64::from(GOAL_SPEED) - 1)
            / u64::from(GOAL_SPEED);
        assert!(
            motion_timeout_for_distance(distance)
                >= Duration::from_millis(ideal_ms_at_commanded_speed)
                    .saturating_add(MOTION_SETTLE_MARGIN)
        );
    }
}

'''
    tests = replace_once(tests, test_anchor, new_tests + test_anchor, "motion timeout tests")

    source_path.write_text(source, encoding="utf-8")
    tests_path.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
