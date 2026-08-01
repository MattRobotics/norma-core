#!/usr/bin/env python3
"""Apply the V35 MATDOG reverse-recovery probe re-home correction.

V34 already proved the complete LF_LOWER_M11_MAX contact cycle and accepted
the measured startup settle. V35 changes only the final reverse recovery:
- the torque-off probe may drift passively by at most 32 ticks while geometry
  prerequisites are restored;
- after the prerequisites reach digital home, the probe is actively re-homed;
- the probe is then torque-disabled and verified within the existing 16-tick
  home tolerance before final global torque OFF.

Contact detection, guard, corridor, prerequisites, speed, torque limit,
timeouts, RAM-only writes and unsigned GoalPosition remain unchanged.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"

EXPECTED_V34_SOURCE_SHA256 = (
    "6dfe739a55778a9cc3da1a866c9f902aaad091ba3cc815753c10e056848ff345"
)
EXPECTED_V34_TESTS_SHA256 = (
    "96c5cfae355822fa19686dd1c015d0d3c01e7f8db78e6d2ff72584f6bfa332d8"
)


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def replace_exact(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    tests = TESTS.read_text(encoding="utf-8")

    if sha256_text(source) != EXPECTED_V34_SOURCE_SHA256:
        raise SystemExit("matdog.rs is not the exact rustfmt-normalized V34 source")
    if sha256_text(tests) != EXPECTED_V34_TESTS_SHA256:
        raise SystemExit("matdog_test.rs is not the exact rustfmt-normalized V34 test source")

    old_probe_tolerance = """const PROBE_HOME_TOLERANCE_TICKS: u16 = 16;
"""
    new_probe_tolerance = """const PROBE_HOME_TOLERANCE_TICKS: u16 = 16;
// During reverse recovery the torque-off probe can be passively displaced by
// the upper-link motion. Keep this bounded separately, then actively re-home
// and verify the probe before final global torque OFF.
const PROBE_PASSIVE_RESTORE_DRIFT_TICKS: u16 = 32;
"""
    source = replace_exact(
        source,
        old_probe_tolerance,
        new_probe_tolerance,
        "add bounded passive restore drift",
    )

    old_home_hold = """fn home_hold_tolerance(
    profile: &ContactProfile,
    motor_id: u8,
    probe_home_handoff_active: bool,
) -> u16 {
    if probe_home_handoff_active && motor_id == profile.motor_id {
        PROBE_HOME_TOLERANCE_TICKS
    } else {
        STATIC_TOLERANCE_TICKS
    }
}
"""
    new_home_hold = """fn home_hold_tolerance(
    profile: &ContactProfile,
    motor_id: u8,
    probe_home_handoff_active: bool,
) -> u16 {
    if probe_home_handoff_active && motor_id == profile.motor_id {
        PROBE_PASSIVE_RESTORE_DRIFT_TICKS
    } else {
        STATIC_TOLERANCE_TICKS
    }
}
"""
    source = replace_exact(
        source,
        old_home_hold,
        new_home_hold,
        "bound passive probe drift during reverse recovery",
    )

    old_recovery = """        self.next_phase("Restore prerequisite joints one at a time")?;
        self.restore_prerequisites().await?;
        self.probe_home_handoff_active = false;

        self.next_phase("Final verified global torque OFF")?;
"""
    new_recovery = """        self.next_phase("Restore prerequisite joints one at a time")?;
        self.restore_prerequisites().await?;
        self.probe_home_handoff_active = false;

        // Restoring the upper link can passively pull the torque-off lower
        // probe a few ticks away from digital home. Re-prime only that probe,
        // settle it tightly at home, then release and verify the off-state.
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(
            self.profile.motor_id,
            HOME_TICK,
            STATIC_TOLERANCE_TICKS,
        )
        .await?;
        self.set_motor_torque_verified(self.profile.motor_id, false)
            .await?;
        let probe_at_rest = self.latest_observation(self.profile.motor_id)?;
        self.ensure_observation_fresh(self.profile.motor_id, probe_at_rest)?;
        if circular_distance(probe_at_rest.position, HOME_TICK) > PROBE_HOME_TOLERANCE_TICKS {
            return Err(format!(
                "M{} post-restore home settle failed: present={}, expected={}, tolerance={}",
                self.profile.motor_id,
                probe_at_rest.position,
                HOME_TICK,
                PROBE_HOME_TOLERANCE_TICKS
            )
            .into());
        }

        self.next_phase("Final verified global torque OFF")?;
"""
    source = replace_exact(
        source,
        old_recovery,
        new_recovery,
        "re-home probe after prerequisite restore",
    )

    tests = replace_exact(
        tests,
        'assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 7);',
        'assert_eq!(source.matches("PROBE_HOME_TOLERANCE_TICKS").count(), 6);',
        "update V35 probe-home scope count",
    )

    old_handoff_test = """#[test]
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
"""
    new_handoff_test = """#[test]
fn probe_reverse_recovery_accepts_observed_2031_then_requires_final_rehome() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    let observed_error = circular_distance(2031, HOME_TICK);
    assert_eq!(observed_error, 17);
    assert_eq!(PROBE_PASSIVE_RESTORE_DRIFT_TICKS, 32);
    assert_eq!(home_hold_tolerance(&profile, 11, true), 32);
    assert_eq!(home_hold_tolerance(&profile, 11, false), 10);
    assert_eq!(home_hold_tolerance(&profile, 12, true), 10);
    assert!(observed_error <= PROBE_PASSIVE_RESTORE_DRIFT_TICKS);
    assert!(circular_distance(2016, HOME_TICK) <= PROBE_PASSIVE_RESTORE_DRIFT_TICKS);
    assert!(circular_distance(2015, HOME_TICK) > PROBE_PASSIVE_RESTORE_DRIFT_TICKS);

    let source = include_str!("matdog.rs");
    let run_start = source
        .find("    async fn run(&mut self)")
        .expect("run function");
    let inspect_start = source[run_start..]
        .find("    async fn inspect_profile_entry(")
        .map(|offset| run_start + offset)
        .expect("inspect function");
    let run = &source[run_start..inspect_start];

    let restore = run.find("self.restore_prerequisites().await?;").unwrap();
    let rehome_prepare = run[restore..]
        .find("self.prepare_motor(self.profile.motor_id).await?;")
        .map(|offset| restore + offset)
        .unwrap();
    let rehome_move = run[rehome_prepare..]
        .find("STATIC_TOLERANCE_TICKS")
        .map(|offset| rehome_prepare + offset)
        .unwrap();
    let rehome_torque_off = run[rehome_move..]
        .find("self.set_motor_torque_verified(self.profile.motor_id, false)")
        .map(|offset| rehome_move + offset)
        .unwrap();
    let final_phase = run
        .find(r#"self.next_phase("Final verified global torque OFF")?;"#)
        .unwrap();

    assert!(
        restore < rehome_prepare
            && rehome_prepare < rehome_move
            && rehome_move < rehome_torque_off
            && rehome_torque_off < final_phase
    );
}
"""
    tests = replace_exact(
        tests,
        old_handoff_test,
        new_handoff_test,
        "replace passive handoff regression with V35 recovery regression",
    )

    SOURCE.write_text(source, encoding="utf-8")
    TESTS.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
