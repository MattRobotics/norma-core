from __future__ import annotations

from dataclasses import replace
import unittest

from tools.matdog import matdog_headless_auto_calibrate as runner
from tools.matdog import matdog_q0_phase_contract as q0_contract


INITIAL_POSITIONS = {
    11: 2047,
    12: 2052,
    13: 2048,
    21: 2057,
    22: 2046,
    23: 2060,
    31: 2057,
    32: 2055,
    33: 2056,
    41: 2048,
    42: 2006,
    43: 2048,
}


def motor(
    motor_id: int,
    *,
    position: int | None = None,
    goal: int = 0,
    torque: bool = False,
    torque_limit: int = 1000,
    speed: int = 0,
) -> runner.MotorSample:
    stamp = runner.station_monotonic_stamp_ns()
    return runner.MotorSample(
        motor_id=motor_id,
        monotonic_stamp_ns=stamp,
        system_stamp_ns=stamp,
        app_start_id=1234,
        state_length=71,
        rx_pointer_hex="01",
        rx_pointer_decimal=1,
        raw_ram_0x28_0x46_hex="00" * 31,
        position=INITIAL_POSITIONS[motor_id] if position is None else position,
        speed_raw=speed,
        current_raw=0,
        voltage_raw=111,
        voltage_v=11.1,
        temperature_c=30,
        temperature_limit_c=70,
        status=0,
        torque_raw=1 if torque else 0,
        torque_enabled=torque,
        goal_position=goal,
        torque_limit=torque_limit,
        driver_error_present=False,
    )


def frame(
    *,
    phase: str,
    step: int,
    motors: dict[int, runner.MotorSample] | None = None,
    status: int = 1,
    status_name: str = "IN_PROGRESS",
) -> runner.FrameSample:
    samples = motors or {
        motor_id: motor(motor_id) for motor_id in runner.EXPECTED_MOTOR_IDS
    }
    return runner.FrameSample(
        bus_serial=runner.EXPECTED_BUS_SERIAL,
        bus_monotonic_stamp_ns=runner.station_monotonic_stamp_ns(),
        bus_system_stamp_ns=1,
        bus_app_start_id=1234,
        calibration=runner.CalibrationSample(
            status=status,
            status_name=status_name,
            current_step=step,
            total_steps=58 if status else 0,
            phase=phase,
            error_message="",
        ),
        motors=samples,
    )


def initialized_contract():
    contract_class = q0_contract.build_phase_aware_contract(runner)
    contract = contract_class()
    contract.validate_preflight(
        frame(phase="", step=0, status=0, status_name="IDLE")
    )
    return contract


def normalized_samples() -> dict[int, runner.MotorSample]:
    return {
        motor_id: motor(motor_id, position=2048, goal=2048)
        for motor_id in runner.EXPECTED_MOTOR_IDS
    }


class Q0PhaseAwareContractTests(unittest.TestCase):
    def test_hardware_replay_accepts_m23_sequential_q0_recovery(self) -> None:
        contract = initialized_contract()
        samples = {
            motor_id: motor(motor_id) for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        samples[23] = motor(
            23,
            position=2060,
            goal=2048,
            torque=True,
            torque_limit=500,
        )
        contract.validate_running(
            frame(
                phase=q0_contract.Q0_NORMALIZATION_PHASE,
                step=3,
                motors=samples,
            )
        )

    def test_q0_recovery_rejects_two_active_motors(self) -> None:
        contract = initialized_contract()
        samples = {
            motor_id: motor(motor_id) for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        samples[23] = motor(23, goal=2048, torque=True, torque_limit=500)
        samples[42] = motor(42, goal=2048, torque=True, torque_limit=500)
        with self.assertRaisesRegex(runner.RunnerError, "more than one"):
            contract.validate_running(
                frame(
                    phase=q0_contract.Q0_NORMALIZATION_PHASE,
                    step=3,
                    motors=samples,
                )
            )

    def test_q0_recovery_rejects_non_home_active_goal(self) -> None:
        contract = initialized_contract()
        samples = {
            motor_id: motor(motor_id) for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        samples[23] = motor(23, goal=2050, torque=True, torque_limit=500)
        with self.assertRaisesRegex(runner.RunnerError, "neither passive baseline"):
            contract.validate_running(
                frame(
                    phase=q0_contract.Q0_NORMALIZATION_PHASE,
                    step=3,
                    motors=samples,
                )
            )

    def test_q0_recovery_rejects_wrong_torque_limit(self) -> None:
        contract = initialized_contract()
        samples = {
            motor_id: motor(motor_id) for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        samples[23] = motor(23, goal=2048, torque=True, torque_limit=1000)
        with self.assertRaisesRegex(runner.RunnerError, "torque limit"):
            contract.validate_running(
                frame(
                    phase=q0_contract.Q0_NORMALIZATION_PHASE,
                    step=3,
                    motors=samples,
                )
            )

    def test_hardware_replay_accepts_m42_prime_then_parking_goal(self) -> None:
        contract = initialized_contract()
        samples = normalized_samples()
        samples[42] = motor(
            42,
            position=2047,
            goal=2047,
            torque=True,
            torque_limit=500,
        )
        contract.validate_running(
            frame(
                phase=q0_contract.M42_PARKING_PHASE,
                step=5,
                motors=samples,
            )
        )

        samples[42] = replace(samples[42], goal_position=2389)
        contract.validate_running(
            frame(
                phase=q0_contract.M42_PARKING_PHASE,
                step=5,
                motors=samples,
            )
        )

    def test_m42_prime_rejects_goal_below_exact_q0_tolerance(self) -> None:
        contract = initialized_contract()
        samples = normalized_samples()
        samples[42] = motor(
            42,
            position=2047,
            goal=2037,
            torque=True,
            torque_limit=500,
        )
        with self.assertRaisesRegex(runner.RunnerError, "controlled M42 goal"):
            contract.validate_running(
                frame(
                    phase=q0_contract.M42_PARKING_PHASE,
                    step=5,
                    motors=samples,
                )
            )

    def test_m42_prime_is_rejected_outside_exact_parking_phase(self) -> None:
        contract = initialized_contract()
        samples = normalized_samples()
        samples[42] = motor(
            42,
            position=2047,
            goal=2047,
            torque=True,
            torque_limit=500,
        )
        with self.assertRaisesRegex(runner.RunnerError, "controlled M42 goal"):
            contract.validate_running(
                frame(
                    phase="LF_LEG_STATE_MACHINE: Prepare LF UPPER M12 MIN",
                    step=6,
                    motors=samples,
                )
            )

    def test_strict_session_accepts_normalized_nonparticipant(self) -> None:
        contract = initialized_contract()
        contract.validate_running(
            frame(
                phase=(
                    "LF_LEG_STATE_MACHINE: Create LF state machine from "
                    "verified q=0 session entry"
                ),
                step=4,
                motors=normalized_samples(),
            )
        )

    def test_strict_session_rejects_nonparticipant_torque(self) -> None:
        contract = initialized_contract()
        samples = normalized_samples()
        samples[23] = replace(
            samples[23],
            torque_raw=1,
            torque_enabled=True,
            torque_limit=500,
        )
        with self.assertRaisesRegex(runner.RunnerError, "nonparticipant M23"):
            contract.validate_running(
                frame(
                    phase=q0_contract.M42_PARKING_PHASE,
                    step=6,
                    motors=samples,
                )
            )


if __name__ == "__main__":
    unittest.main()
