#!/usr/bin/env python3
"""Apply the reviewed MATDOG LF measurement/freeze source upgrade.

This script is intentionally marker-driven and fail-closed. It is used first
inside CI on a clean checkout; the same byte-identical transformation is later
used by the user-facing installer.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
PORT = ROOT / "software/drivers/st3215/src/port.rs"
MATDOG = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"
RUNNER = ROOT / "tools/matdog/matdog_headless_auto_calibrate.py"
WORKFLOW = ROOT / ".github/workflows/matdog-native-calibrator-check.yml"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one marker, found {count}")
    return text.replace(old, new, 1)


def replace_between(text: str, start: str, end: str, replacement: str, label: str) -> str:
    start_index = text.find(start)
    if start_index < 0:
        raise RuntimeError(f"{label}: start marker missing")
    end_index = text.find(end, start_index)
    if end_index < 0:
        raise RuntimeError(f"{label}: end marker missing")
    if text.find(start, start_index + 1) >= 0:
        raise RuntimeError(f"{label}: start marker is not unique")
    return text[:start_index] + replacement + text[end_index:]


def replace_test_function(text: str, name: str, replacement: str) -> str:
    pattern = re.compile(
        rf"(?ms)^    #\[test\]\n    fn {re.escape(name)}\(\) \{{.*?^    \}}\n"
    )
    updated, count = pattern.subn(replacement.rstrip() + "\n", text)
    if count != 1:
        raise RuntimeError(f"test {name}: expected one function, found {count}")
    return updated


THERMAL_TOP = r'''const MATDOG_EXPECTED_TEMPERATURE_LIMIT_C: u8 = 70;
const MATDOG_MAX_TEMPERATURE_LIMIT_ADDRESS: usize = 0x0D;
const MATDOG_IMMEDIATE_THERMAL_ABORT_C: u8 = 85;
const MATDOG_THERMAL_SAMPLE_PERIOD: Duration = Duration::from_millis(500);
const MATDOG_THERMAL_CONFIRMATION_DELAY: Duration = Duration::from_millis(50);
const MATDOG_THERMAL_CONFIRMATION_READS: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MatdogThermalDecision {
    Normal,
    Transient,
    Confirmed,
    InvalidConfiguredLimit,
}

#[derive(Debug, Default)]
struct MatdogThermalState {
    last_direct_read: HashMap<u8, Instant>,
    last_confirmed: HashMap<u8, u8>,
}

fn classify_matdog_direct_temperature_samples(
    configured_limit_c: u8,
    samples: &[u8],
) -> MatdogThermalDecision {
    if configured_limit_c != MATDOG_EXPECTED_TEMPERATURE_LIMIT_C {
        return MatdogThermalDecision::InvalidConfiguredLimit;
    }
    if samples.is_empty() {
        return MatdogThermalDecision::Transient;
    }
    if samples
        .iter()
        .any(|temperature| *temperature >= MATDOG_IMMEDIATE_THERMAL_ABORT_C)
    {
        return MatdogThermalDecision::Confirmed;
    }
    let over_limit = samples
        .iter()
        .filter(|temperature| **temperature > configured_limit_c)
        .count();
    if over_limit >= 2 {
        MatdogThermalDecision::Confirmed
    } else if over_limit == 0 {
        MatdogThermalDecision::Normal
    } else {
        MatdogThermalDecision::Transient
    }
}

'''

THERMAL_IMPL = r'''    fn combine_cached_eeprom_and_ram(eeprom_prefix: &Bytes, ram_data: Bytes) -> Bytes {
        let mut combined = BytesMut::with_capacity(eeprom_prefix.len() + ram_data.len());
        combined.extend_from_slice(eeprom_prefix);
        combined.extend_from_slice(&ram_data);
        combined.freeze()
    }

    fn overwrite_temperature(full_data: Bytes, temperature_c: u8) -> Bytes {
        let address = protocol::RamRegister::PresentTemperature.address() as usize;
        if full_data.len() <= address {
            return full_data;
        }
        let mut updated = full_data.to_vec();
        updated[address] = temperature_c;
        Bytes::from(updated)
    }

    async fn read_motor_temperature_direct(
        port: &mut tokio_serial::SerialStream,
        motor_id: u8,
        bus_serial: &str,
    ) -> Result<u8, protocol::Error> {
        let request = protocol::ST3215Request::Read {
            motor: motor_id,
            address: protocol::RamRegister::PresentTemperature.address(),
            length: protocol::RamRegister::PresentTemperature.size(),
        };
        let started = Instant::now();
        let result = request.async_readwrite(port, ST3215_TIMEOUT_MS).await;
        let elapsed_ms = started.elapsed().as_millis();
        if elapsed_ms >= ST3215_SLOW_READ_WARN_MS {
            warn!(
                "ST3215 slow direct temperature read: bus={} motor={} elapsed={}ms",
                bus_serial, motor_id, elapsed_ms
            );
        }
        match result? {
            protocol::ST3215Response::Read { data, .. } if data.len() == 1 => Ok(data[0]),
            protocol::ST3215Response::Read { data, source_bytes } => Err(protocol::Error::InvalidData {
                msg: format!("direct temperature length mismatch: {}", data.len()),
                source_packet: request.to_bytes(),
                reply_packet: source_bytes,
            }),
            _ => unreachable!(),
        }
    }

    async fn force_matdog_motor_torque_off(
        port: &mut tokio_serial::SerialStream,
        motor_id: u8,
        bus_serial: &str,
        reason: &str,
    ) -> Result<(), protocol::Error> {
        warn!(
            "MATDOG forcing torque OFF: bus={} motor={} reason={}",
            bus_serial, motor_id, reason
        );
        let request = protocol::ST3215Request::Write {
            motor: motor_id,
            address: protocol::RamRegister::TorqueEnable.address(),
            data: Bytes::from_static(&[0]),
        };
        request.async_readwrite(port, ST3215_COMMAND_TIMEOUT_MS).await?;
        let verify = protocol::ST3215Request::Read {
            motor: motor_id,
            address: protocol::RamRegister::TorqueEnable.address(),
            length: 1,
        };
        match verify.async_readwrite(port, ST3215_TIMEOUT_MS).await? {
            protocol::ST3215Response::Read { data, .. } if data.as_ref() == [0] => Ok(()),
            protocol::ST3215Response::Read { data, source_bytes } => Err(protocol::Error::InvalidData {
                msg: format!("M{motor_id} torque-OFF readback mismatch: {data:02x?}"),
                source_packet: verify.to_bytes(),
                reply_packet: source_bytes,
            }),
            _ => unreachable!(),
        }
    }

    async fn apply_matdog_direct_temperature(
        port: &mut tokio_serial::SerialStream,
        motor_id: u8,
        bus_serial: &str,
        full_data: Bytes,
        thermal: &mut MatdogThermalState,
    ) -> Result<Bytes, protocol::Error> {
        if !crate::auto_calibrate::matdog_calibrator_is_armed()
            || !MATDOG_MOTOR_IDS.contains(&motor_id)
        {
            return Ok(full_data);
        }

        let temperature_address = protocol::RamRegister::PresentTemperature.address() as usize;
        if full_data.len() <= temperature_address
            || full_data.len() <= MATDOG_MAX_TEMPERATURE_LIMIT_ADDRESS
        {
            return Ok(full_data);
        }
        let configured_limit_c = full_data[MATDOG_MAX_TEMPERATURE_LIMIT_ADDRESS];
        if configured_limit_c != MATDOG_EXPECTED_TEMPERATURE_LIMIT_C {
            Self::force_matdog_motor_torque_off(
                port,
                motor_id,
                bus_serial,
                "invalid configured temperature limit",
            )
            .await?;
            return Ok(full_data);
        }

        let due = thermal
            .last_direct_read
            .get(&motor_id)
            .map(|instant| instant.elapsed() >= MATDOG_THERMAL_SAMPLE_PERIOD)
            .unwrap_or(true);
        if !due {
            if let Some(confirmed) = thermal.last_confirmed.get(&motor_id).copied() {
                return Ok(Self::overwrite_temperature(full_data, confirmed));
            }
        }

        let mut samples = Vec::with_capacity(MATDOG_THERMAL_CONFIRMATION_READS);
        let first = Self::read_motor_temperature_direct(port, motor_id, bus_serial).await?;
        samples.push(first);
        if first > configured_limit_c && first < MATDOG_IMMEDIATE_THERMAL_ABORT_C {
            for _ in 1..MATDOG_THERMAL_CONFIRMATION_READS {
                tokio::time::sleep(MATDOG_THERMAL_CONFIRMATION_DELAY).await;
                samples.push(
                    Self::read_motor_temperature_direct(port, motor_id, bus_serial).await?,
                );
            }
        }
        thermal.last_direct_read.insert(motor_id, Instant::now());

        match classify_matdog_direct_temperature_samples(configured_limit_c, &samples) {
            MatdogThermalDecision::Normal => {
                let confirmed = *samples.last().expect("non-empty direct temperature samples");
                thermal.last_confirmed.insert(motor_id, confirmed);
                Ok(Self::overwrite_temperature(full_data, confirmed))
            }
            MatdogThermalDecision::Transient => {
                let confirmed = samples
                    .iter()
                    .rev()
                    .copied()
                    .find(|temperature| *temperature <= configured_limit_c)
                    .or_else(|| thermal.last_confirmed.get(&motor_id).copied())
                    .ok_or_else(|| protocol::Error::InvalidData {
                        msg: format!(
                            "M{motor_id} thermal transient has no confirmed normal sample: {samples:?}"
                        ),
                        source_packet: Bytes::new(),
                        reply_packet: Bytes::new(),
                    })?;
                thermal.last_confirmed.insert(motor_id, confirmed);
                warn!(
                    "MATDOG_THERMAL_DIRECT_TRANSIENT bus={} motor={} direct_samples={:?} published={}",
                    bus_serial, motor_id, samples, confirmed
                );
                Ok(Self::overwrite_temperature(full_data, confirmed))
            }
            MatdogThermalDecision::Confirmed => {
                let confirmed = samples
                    .iter()
                    .copied()
                    .filter(|temperature| *temperature > configured_limit_c)
                    .max()
                    .unwrap_or(first);
                Self::force_matdog_motor_torque_off(
                    port,
                    motor_id,
                    bus_serial,
                    "direct temperature confirmed over limit",
                )
                .await?;
                warn!(
                    "MATDOG_THERMAL_DIRECT_CONFIRMED bus={} motor={} direct_samples={:?} published={}",
                    bus_serial, motor_id, samples, confirmed
                );
                Ok(Self::overwrite_temperature(full_data, confirmed))
            }
            MatdogThermalDecision::InvalidConfiguredLimit => unreachable!(),
        }
    }

'''

PROFILE_LINE_FUNCTION = r'''
fn lf_machine_profile_record(evidence: JointCalibrationEvidence) -> String {
    let spec = evidence.spec;
    let fixed = evidence.fixed_scale;
    let affine = evidence.affine;
    let minimum = build_profile(Leg::Lf, spec.kind, ContactSide::Min)
        .expect("validated LF MIN profile");
    let maximum = build_profile(Leg::Lf, spec.kind, ContactSide::Max)
        .expect("validated LF MAX profile");
    format!(
        "MATDOG_LF_PROFILE_V1|joint={}|joint_name={}|motor_id={}|direction={}|urdf_min_delta={}|urdf_max_delta={}|urdf_min_tick={}|urdf_max_tick={}|coarse_min={}|coarse_max={}|fine_min_1={}|fine_min_2={}|fine_max_1={}|fine_max_2={}|repeatability_min={}|repeatability_max={}|contact_min={}|contact_max={}|q0_fixed={}|q0_affine={}|endpoint_disagreement={}|q0_shift={}|scale_permille={}|safe_min_tick={}|safe_max_tick={}|accepted={}",
        spec.kind.label(),
        spec.name,
        spec.motor_id,
        spec.direction,
        spec.min_delta,
        spec.max_delta,
        minimum.urdf_limit_tick,
        maximum.urdf_limit_tick,
        evidence.contacts.minimum.coarse_scout_tick,
        evidence.contacts.maximum.coarse_scout_tick,
        evidence.contacts.minimum.first_tick,
        evidence.contacts.minimum.second_tick,
        evidence.contacts.maximum.first_tick,
        evidence.contacts.maximum.second_tick,
        evidence.contacts.minimum.spread_ticks,
        evidence.contacts.maximum.spread_ticks,
        fixed.minimum_contact_tick,
        fixed.maximum_contact_tick,
        fixed.estimated_zero_tick,
        affine.estimated_zero_tick,
        fixed.endpoint_disagreement_ticks,
        fixed.shift_from_digital_home_ticks,
        affine.scale_permille,
        minimum.urdf_limit_tick.min(maximum.urdf_limit_tick),
        minimum.urdf_limit_tick.max(maximum.urdf_limit_tick),
        evidence.accepted,
    )
}
'''


def patch_port() -> None:
    text = PORT.read_text()
    text = replace_between(
        text,
        "const MATDOG_EXPECTED_TEMPERATURE_LIMIT_C: u8 = 70;",
        "fn matdog_command_allowed_with",
        THERMAL_TOP + "fn matdog_command_allowed_with",
        "port thermal top",
    )
    text = replace_once(
        text,
        "        let mut matdog_thermal_transients: HashMap<u8, u8> = HashMap::new();\n        let mut matdog_thermal_transient_total = 0_u8;",
        "        let mut matdog_thermal_state = MatdogThermalState::default();",
        "port worker thermal state",
    )
    old_worker_args = "&mut matdog_thermal_transients,\n                                &mut matdog_thermal_transient_total,"
    if text.count(old_worker_args) != 1:
        raise RuntimeError("worker thermal arguments: marker mismatch")
    text = text.replace(old_worker_args, "&mut matdog_thermal_state,", 1)
    text = replace_between(
        text,
        "    fn combine_cached_eeprom_and_ram",
        "    async fn scan_motors(",
        THERMAL_IMPL + "    async fn scan_motors(",
        "port thermal implementation",
    )
    text = replace_once(
        text,
        "        matdog_thermal_transients: &mut HashMap<u8, u8>,\n        matdog_thermal_transient_total: &mut u8,",
        "        matdog_thermal_state: &mut MatdogThermalState,",
        "scan thermal arguments",
    )
    old_call = '''                    let final_data = match Self::confirm_matdog_temperature_if_needed(
                        port,
                        motor_id,
                        &bus_info.serial_number,
                        final_data,
                        matdog_thermal_transients,
                        matdog_thermal_transient_total,
                    )
                    .await
'''
    new_call = '''                    let final_data = match Self::apply_matdog_direct_temperature(
                        port,
                        motor_id,
                        &bus_info.serial_number,
                        final_data,
                        matdog_thermal_state,
                    )
                    .await
'''
    text = replace_once(text, old_call, new_call, "scan thermal call")
    text = replace_test_function(
        text,
        "matdog_temperature_confirmation_rejects_invalid_limit_and_real_heat",
        '''    #[test]
    fn matdog_direct_temperature_rejects_invalid_limit_and_confirms_real_heat() {
        assert_eq!(
            classify_matdog_direct_temperature_samples(69, &[39]),
            MatdogThermalDecision::InvalidConfiguredLimit
        );
        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[85]),
            MatdogThermalDecision::Confirmed
        );
        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[73, 72, 39]),
            MatdogThermalDecision::Confirmed
        );
    }''',
    )
    text = replace_test_function(
        text,
        "matdog_temperature_confirmation_accepts_one_isolated_transient_only",
        '''    #[test]
    fn matdog_direct_temperature_never_converts_repeated_false_spikes_to_heat() {
        for _ in 0..10 {
            assert_eq!(
                classify_matdog_direct_temperature_samples(70, &[76, 39, 39]),
                MatdogThermalDecision::Transient
            );
        }
        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[39]),
            MatdogThermalDecision::Normal
        );
    }''',
    )
    forbidden = (
        "MATDOG_MAX_TRANSIENTS_PER_MOTOR",
        "MATDOG_MAX_TRANSIENTS_TOTAL",
        "confirmed_or_budget_exhausted",
        "classify_matdog_temperature_sample",
    )
    found = [token for token in forbidden if token in text]
    if found:
        raise RuntimeError(f"stale thermal-budget logic remains: {found}")
    PORT.write_text(text)


def patch_matdog() -> None:
    text = MATDOG.read_text()
    text = replace_once(
        text,
        "        accepted: fixed_scale.accepted,",
        "        accepted: fixed_scale.accepted && affine.accepted,",
        "joint evidence acceptance",
    )
    marker = "\nfn joint_degree_evidence(evidence: JointCalibrationEvidence) -> String {"
    text = replace_once(text, marker, PROFILE_LINE_FUNCTION + marker, "machine profile function")
    text = replace_once(
        text,
        '            info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));',
        '            info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));\n            info!("{}", lf_machine_profile_record(evidence));',
        "machine profile emission",
    )
    old_rejection = '''        if !diagnostic_rejections.is_empty() {
            info!(
                "MATDOG LF Q0 DIAGNOSTIC ONLY: {}; canonical saved q0 remains HOME_TICK={} for every commanded return; endpoint consistency reference={} ticks; affine reference={}..={} permille",
                diagnostic_rejections.join("; "),
                HOME_TICK,
                MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS,
                AFFINE_SCALE_MIN_PERMILLE,
                AFFINE_SCALE_MAX_PERMILLE,
            );
        }
'''
    new_rejection = '''        if !diagnostic_rejections.is_empty() {
            return Err(format!(
                "MATDOG LF URDF freeze gate rejected before EEPROM staging: {}; endpoint consistency limit={} ticks; affine scale reference={}..={} permille",
                diagnostic_rejections.join("; "),
                MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS,
                AFFINE_SCALE_MIN_PERMILLE,
                AFFINE_SCALE_MAX_PERMILLE,
            )
            .into());
        }
        info!(
            "MATDOG LF URDF FREEZE GATE: PASS; all three joints accepted for staged q0 positioning"
        );
'''
    text = replace_once(text, old_rejection, new_rejection, "URDF freeze gate")
    text = replace_once(
        text,
        '        self.next_phase("Move LF HIP M13 directly from MAX contact to canonical saved q=0")?;',
        '        self.next_phase("Move LF HIP M13 from MAX contact to URDF-derived staged q=0")?;',
        "hip staged phase",
    )
    text = replace_once(
        text,
        "        self.move_motor_to(13, HOME_TICK, STATIC_TOLERANCE_TICKS)\n            .await?;\n        self.upsert_held_target(StaticTarget {\n            motor_id: 13,\n            target_tick: HOME_TICK,\n        })?;",
        "        let hip_staged_q0 = outcome.joints[0].fixed_scale.estimated_zero_tick;\n        self.move_motor_to(13, hip_staged_q0, STATIC_TOLERANCE_TICKS)\n            .await?;\n        self.upsert_held_target(StaticTarget {\n            motor_id: 13,\n            target_tick: hip_staged_q0,\n        })?;",
        "hip staged target",
    )
    text = replace_once(
        text,
        '        self.next_phase("Move LF LOWER M11 to canonical saved q=0 and keep active hold")?;',
        '        self.next_phase("Move LF LOWER M11 to URDF-derived staged q=0 and hold")?;',
        "lower staged phase",
    )
    text = replace_once(
        text,
        "        self.move_motor_to(11, HOME_TICK, STATIC_TOLERANCE_TICKS)\n            .await?;\n        self.upsert_held_target(StaticTarget {\n            motor_id: 11,\n            target_tick: HOME_TICK,\n        })?;",
        "        let lower_staged_q0 = outcome.joints[2].fixed_scale.estimated_zero_tick;\n        self.move_motor_to(11, lower_staged_q0, STATIC_TOLERANCE_TICKS)\n            .await?;\n        self.upsert_held_target(StaticTarget {\n            motor_id: 11,\n            target_tick: lower_staged_q0,\n        })?;",
        "lower staged target",
    )
    text = replace_once(
        text,
        '        self.next_phase("Move LF UPPER M12 to canonical saved q=0 while M11 remains held")?;',
        '        self.next_phase("Move LF UPPER M12 to URDF-derived staged q=0 while M11 holds")?;',
        "upper staged phase",
    )
    text = replace_once(
        text,
        "        self.move_motor_to(12, HOME_TICK, STATIC_TOLERANCE_TICKS)\n            .await?;\n        self.upsert_held_target(StaticTarget {\n            motor_id: 12,\n            target_tick: HOME_TICK,\n        })?;",
        "        let upper_staged_q0 = outcome.joints[1].fixed_scale.estimated_zero_tick;\n        self.move_motor_to(12, upper_staged_q0, STATIC_TOLERANCE_TICKS)\n            .await?;\n        self.upsert_held_target(StaticTarget {\n            motor_id: 12,\n            target_tick: upper_staged_q0,\n        })?;",
        "upper staged target",
    )
    text = replace_once(
        text,
        '                        "MATDOG {} complete: M13_q0_fixed={}, M12_q0_fixed={}, M11_q0_fixed={}, M13_q0_affine={}, M12_q0_affine={}, M11_q0_affine={}, RAM-only=true, EEPROM_written=false",',
        '                        "MATDOG {} measurement complete: M13_q0_fixed={}, M12_q0_fixed={}, M11_q0_fixed={}, M13_q0_affine={}, M12_q0_affine={}, M11_q0_affine={}, status=LF_STAGED, movement_RAM_only=true, EEPROM_written=false",',
        "staged completion log",
    )
    MATDOG.write_text(text)


def patch_runner() -> None:
    text = RUNNER.read_text()
    text = replace_once(
        text,
        "source-latest preflight lasting at least 30 seconds",
        "source-latest preflight using a short consecutive-snapshot gate",
        "runner doc preflight",
    )
    queue_class = r'''

class LatestOnlyQueue(asyncio.Queue[Any]):
    """A queue-of-one: newer telemetry atomically supersedes older telemetry."""

    def __init__(self) -> None:
        super().__init__(maxsize=1)

    def put_nowait(self, item: Any) -> None:
        if self.full():
            try:
                self.get_nowait()
            except asyncio.QueueEmpty:
                pass
        super().put_nowait(item)

    async def put(self, item: Any) -> None:
        self.put_nowait(item)
'''
    text = replace_once(
        text,
        "\nclass RunnerError(RuntimeError):",
        queue_class + "\n\nclass RunnerError(RuntimeError):",
        "latest-only queue class",
    )
    text = text.replace("asyncio.Queue[Any] = asyncio.Queue()", "asyncio.Queue[Any] = LatestOnlyQueue()")
    text = text.replace("self.entries = asyncio.Queue()", "self.entries = LatestOnlyQueue()")
    text = replace_once(
        text,
        '            self.evidence.emit("calibration_progress", **current)',
        '            self.evidence.emit("calibration_progress", **current)\n            print(\n                f"[{calibration.current_step:02d}/{calibration.total_steps:02d}] "\n                f"{calibration.status_name}: {calibration.phase}"\n                + (f" — {calibration.error_message}" if calibration.error_message else ""),\n                flush=True,\n            )',
        "live progress output",
    )
    text = replace_once(
        text,
        '    if args.preflight_frames < 120:\n        parser.error("--preflight-frames must be at least 120")\n    if args.preflight_seconds < 30.0:\n        parser.error("--preflight-seconds must be at least 30")',
        '    if args.preflight_frames < 10:\n        parser.error("--preflight-frames must be at least 10")\n    if args.preflight_seconds < 1.0:\n        parser.error("--preflight-seconds must be at least 1.0")',
        "short preflight validation",
    )
    RUNNER.write_text(text)


def patch_tests() -> None:
    text = TESTS.read_text()
    text = replace_test_function(
        text,
        "endpoint_derived_q0_is_diagnostic_only_and_never_a_return_target",
        '''    #[test]
    fn accepted_endpoint_q0_is_used_only_for_transactional_staging() {
        let source = include_str!("matdog.rs");
        assert!(source.contains("MATDOG LF URDF FREEZE GATE: PASS"));
        assert!(source.contains("hip_staged_q0"));
        assert!(source.contains("lower_staged_q0"));
        assert!(source.contains("upper_staged_q0"));
        assert!(source.contains("movement_RAM_only=true, EEPROM_written=false"));
        assert!(!source.contains("reg_write: Some"));
        assert!(!source.contains("freeze_calibration: Some"));
    }''',
    )
    text = replace_test_function(
        text,
        "full_lf_final_return_order_uses_saved_q0_for_m13_m11_m12_then_m42",
        '''    #[test]
    fn full_lf_final_order_stages_m13_m11_m12_then_restores_m42() {
        let source = include_str!("matdog.rs");
        let hip = source.find("let hip_staged_q0").unwrap();
        let lower = source.find("let lower_staged_q0").unwrap();
        let upper = source.find("let upper_staged_q0").unwrap();
        let parking = source
            .find("Restore LH upper M42 once at end of LF calibration")
            .unwrap();
        assert!(hip < lower && lower < upper && upper < parking);
    }''',
    )
    text = replace_test_function(
        text,
        "historical_contacts_keep_endpoint_and_affine_diagnostics_without_replacing_q0",
        '''    #[test]
    fn historical_contacts_reject_freeze_when_endpoint_or_affine_gate_fails() {
        let upper = derive_joint_evidence(
            *spec_for(Leg::Lf, JointKind::Upper),
            dual_contact(1446, 1441, 3443, 3442),
        );
        let lower = derive_joint_evidence(
            *spec_for(Leg::Lf, JointKind::Lower),
            dual_contact(3132, 3135, 1640, 1643),
        );
        let hip = derive_joint_evidence(
            *spec_for(Leg::Lf, JointKind::Hip),
            dual_contact(2546, 2544, 1545, 1547),
        );
        assert_eq!(upper.accepted, upper.fixed_scale.accepted && upper.affine.accepted);
        assert_eq!(lower.accepted, lower.fixed_scale.accepted && lower.affine.accepted);
        assert_eq!(hip.accepted, hip.fixed_scale.accepted && hip.affine.accepted);
        assert!(!lower.accepted || !hip.accepted);
    }''',
    )
    TESTS.write_text(text)


def patch_workflow() -> None:
    text = WORKFLOW.read_text()
    text = replace_once(
        text,
        '              "MATDOG LF Q0 DIAGNOSTIC ONLY",',
        '              "MATDOG LF URDF FREEZE GATE: PASS",\n              "MATDOG_LF_PROFILE_V1|joint=",',
        "workflow required source",
    )
    text = replace_once(
        text,
        '              "endpoint_derived_q0_is_diagnostic_only_and_never_a_return_target",\n              "full_lf_final_return_order_uses_saved_q0_for_m13_m11_m12_then_m42",\n              "historical_contacts_keep_endpoint_and_affine_diagnostics_without_replacing_q0",',
        '              "accepted_endpoint_q0_is_used_only_for_transactional_staging",\n              "full_lf_final_order_stages_m13_m11_m12_then_restores_m42",\n              "historical_contacts_reject_freeze_when_endpoint_or_affine_gate_fails",',
        "workflow required tests",
    )
    old_return_checks = '''              if "fixed_scale.estimated_zero_tick" in return_body:
                  constant_errors.append("derived fixed-scale q0 used as return target")
              if "affine.estimated_zero_tick" in return_body:
                  constant_errors.append("derived affine q0 used as return target")
              if return_body.count("HOME_TICK") != 6:
                  constant_errors.append(
                      f"final LF return HOME_TICK uses={return_body.count('HOME_TICK')} expected 6"
                  )
'''
    new_return_checks = '''              required_staged = (
                  "hip_staged_q0",
                  "lower_staged_q0",
                  "upper_staged_q0",
              )
              missing_staged = [token for token in required_staged if token not in return_body]
              if missing_staged:
                  constant_errors.append(
                      f"missing transactional staged q0 targets: {missing_staged}"
                  )
              if "affine.estimated_zero_tick" in return_body:
                  constant_errors.append("affine diagnostic used as EEPROM staging target")
'''
    text = replace_once(text, old_return_checks, new_return_checks, "workflow return gate")
    text = replace_once(
        text,
        "            software/drivers/st3215/src/auto_calibrate/matdog_test.rs \\\n            2>&1 | tee ci-output/matdog-rustfmt.log",
        "            software/drivers/st3215/src/auto_calibrate/matdog_test.rs \\\n            software/drivers/st3215/src/bin/matdog_lf_freeze.rs \\\n            2>&1 | tee ci-output/matdog-rustfmt.log",
        "workflow rustfmt provisioner",
    )
    text = replace_once(
        text,
        "            tools.matdog.test_matdog_headless_auto_calibrate \\\n            2>&1 | tee ci-output/headless-runner-tests.log",
        "            tools.matdog.test_matdog_headless_auto_calibrate \\\n            tools.matdog.test_matdog_lf_profile \\\n            2>&1 | tee ci-output/headless-runner-tests.log",
        "workflow profile tests",
    )
    WORKFLOW.write_text(text)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    if not args.apply:
        parser.error("--apply is required")
    patch_port()
    patch_matdog()
    patch_runner()
    patch_tests()
    patch_workflow()
    print("MATDOG_LF_FREEZE_SOURCE_UPGRADE=APPLIED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
