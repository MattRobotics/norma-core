#!/usr/bin/env python3
"""Phase-aware runtime contract for the MATDOG q0-first LF runner.

The reviewed Rust state machine normalizes every canonical joint to saved q0
before assigning strict LF session roles.  The base observer predates that
ordering and treats the eight non-LF-session motors as permanently passive.
This adapter preserves every base payload/freshness check while admitting only
the exact, sequential q0 recovery envelope during the named startup phase.
"""

from __future__ import annotations

from typing import Any


HOME_TICK = 2048
STARTUP_HOME_RECOVERY_LIMIT_TICKS = 64
STARTUP_HOME_POSITION_MARGIN_TICKS = 16
Q0_NORMALIZATION_PHASE = (
    "LF_LEG_STATE_MACHINE: Normalize every displaced MATDOG joint to q=0 "
    "with one uniform rule"
)


def build_phase_aware_contract(runner: Any):
    """Return a FrameContract subclass bound to the reviewed runner module."""

    base = runner.FrameContract
    if getattr(base, "_matdog_q0_phase_aware", False):
        return base

    class PhaseAwareFrameContract(base):
        _matdog_q0_phase_aware = True

        def _require_initial_state(self, motor_id: int) -> tuple[int, int]:
            if motor_id not in self.initial_goals:
                raise runner.RunnerError(
                    f"M{motor_id} has no passive-preflight goal baseline"
                )
            if motor_id not in self.initial_positions:
                raise runner.RunnerError(
                    f"M{motor_id} has no passive-preflight position baseline"
                )
            return self.initial_goals[motor_id], self.initial_positions[motor_id]

        def _validate_q0_normalization(self, frame: Any) -> None:
            self.validate_common(frame)
            enabled = [
                motor_id
                for motor_id, sample in frame.motors.items()
                if sample.torque_enabled
            ]
            if len(enabled) > 1:
                raise runner.RunnerError(
                    "q0 normalization has more than one torque-enabled motor: "
                    f"{enabled}"
                )

            for motor_id in runner.EXPECTED_MOTOR_IDS:
                sample = frame.motors[motor_id]
                initial_goal, initial_position = self._require_initial_state(motor_id)
                initial_distance = runner.circular_distance(
                    initial_position,
                    HOME_TICK,
                )
                if initial_distance > STARTUP_HOME_RECOVERY_LIMIT_TICKS:
                    raise runner.RunnerError(
                        f"q0 normalization M{motor_id} initial distance "
                        f"{initial_distance} > {STARTUP_HOME_RECOVERY_LIMIT_TICKS}"
                    )

                allowed_goals = {initial_goal, HOME_TICK}
                if sample.goal_position not in allowed_goals:
                    raise runner.RunnerError(
                        f"q0 normalization M{motor_id} goal "
                        f"{sample.goal_position} is neither passive baseline "
                        f"{initial_goal} nor canonical home {HOME_TICK}"
                    )

                maximum_home_distance = (
                    initial_distance + STARTUP_HOME_POSITION_MARGIN_TICKS
                )
                home_distance = runner.circular_distance(
                    sample.position,
                    HOME_TICK,
                )
                if home_distance > maximum_home_distance:
                    raise runner.RunnerError(
                        f"q0 normalization M{motor_id} moved away from home: "
                        f"distance={home_distance}, allowed={maximum_home_distance}"
                    )

                if sample.torque_enabled:
                    if sample.goal_position != HOME_TICK:
                        raise runner.RunnerError(
                            f"q0 normalization active M{motor_id} goal "
                            f"{sample.goal_position} != {HOME_TICK}"
                        )
                    if sample.torque_limit != runner.EXPECTED_ACTIVE_TORQUE_LIMIT:
                        raise runner.RunnerError(
                            f"q0 normalization active M{motor_id} torque limit "
                            f"{sample.torque_limit} != "
                            f"{runner.EXPECTED_ACTIVE_TORQUE_LIMIT}"
                        )
                elif sample.speed_raw > runner.MAX_IDLE_SPEED_RAW:
                    raise runner.RunnerError(
                        f"q0 normalization passive M{motor_id} "
                        f"speed={sample.speed_raw}"
                    )

        def _validate_strict_lf_session(self, frame: Any) -> None:
            self.validate_common(frame)
            for motor_id in runner.CONTROLLED_MOTOR_IDS:
                sample = frame.motors[motor_id]
                if sample.torque_enabled:
                    if sample.torque_limit != runner.EXPECTED_ACTIVE_TORQUE_LIMIT:
                        raise runner.RunnerError(
                            f"controlled M{motor_id} torque limit changed: "
                            f"{sample.torque_limit} != "
                            f"{runner.EXPECTED_ACTIVE_TORQUE_LIMIT}"
                        )
                    position_low, position_high = (
                        runner.CONTROLLED_POSITION_CORRIDORS[motor_id]
                    )
                    if not position_low <= sample.position <= position_high:
                        raise runner.RunnerError(
                            f"controlled M{motor_id} position {sample.position} "
                            f"outside {position_low}..={position_high}"
                        )
                    goal_low, goal_high = runner.CONTROLLED_GOAL_CORRIDORS[motor_id]
                    if not goal_low <= sample.goal_position <= goal_high:
                        raise runner.RunnerError(
                            f"controlled M{motor_id} goal {sample.goal_position} "
                            f"outside {goal_low}..={goal_high}"
                        )

            for motor_id in runner.NONPARTICIPATING_MOTOR_IDS:
                sample = frame.motors[motor_id]
                initial_goal, _ = self._require_initial_state(motor_id)
                if sample.torque_enabled:
                    raise runner.RunnerError(
                        f"nonparticipant M{motor_id} torque became ON"
                    )
                if sample.goal_position not in {initial_goal, HOME_TICK}:
                    raise runner.RunnerError(
                        f"nonparticipant M{motor_id} goal {sample.goal_position} "
                        f"is neither passive baseline {initial_goal} nor "
                        f"canonical home {HOME_TICK}"
                    )
                home_distance = runner.circular_distance(
                    sample.position,
                    HOME_TICK,
                )
                if home_distance > runner.MAX_NONPARTICIPANT_DRIFT:
                    raise runner.RunnerError(
                        f"nonparticipant M{motor_id} is {home_distance} ticks "
                        f"from canonical home"
                    )
                if sample.speed_raw > runner.MAX_IDLE_SPEED_RAW:
                    raise runner.RunnerError(
                        f"nonparticipant M{motor_id} speed={sample.speed_raw}"
                    )

        def validate_running(self, frame: Any) -> None:
            if frame.calibration.phase == Q0_NORMALIZATION_PHASE:
                self._validate_q0_normalization(frame)
                return
            self._validate_strict_lf_session(frame)

    PhaseAwareFrameContract.__name__ = "PhaseAwareFrameContract"
    PhaseAwareFrameContract.__qualname__ = "PhaseAwareFrameContract"
    return PhaseAwareFrameContract


def install(runner: Any):
    """Install the adapter once and return the effective contract class."""

    runner.FrameContract = build_phase_aware_contract(runner)
    return runner.FrameContract
