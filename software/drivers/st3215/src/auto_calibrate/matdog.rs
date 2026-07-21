//! MATDOG-specific, RAM-only ST3215 mechanical end-stop calibrator.
//!
//! The first hardware scope is deliberately restricted to LF_UPPER / M12 /
//! URDF MIN. This module never resets a servo, never unlocks or writes EEPROM,
//! never changes Position Offset, and never freezes calibration arcs.

use crate::protocol::{self, RamRegister};
use crate::st3215_proto::{CommandResult, InferenceState, TxEnvelope};
use crate::state::{CalibrationStatus, ST3215BusCommunicator};
use bytes::Bytes;
use log::{error, info};
use prost::Message;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::watch;
use tokio::time::{Duration, Instant};

type DynError = Box<dyn std::error::Error + Send + Sync>;

pub const MATDOG_MOTOR_IDS: [u8; 12] = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43];
const PILOT_MOTOR_ID: u8 = 12;
const HOME_TICK: u16 = 2048;
const M12_URDF_MIN_TICK: u16 = 1451;
const M12_MIN_GUARD_TICK: u16 = 1434;
const BASELINE_TARGET_TICK: u16 = 1984;
const PILOT_TORQUE_LIMIT: u16 = 400;
const PILOT_SPEED: u16 = 80;
const PILOT_ACCELERATION: u8 = 4;
const COARSE_STEP_TICKS: u16 = 32;
const FINE_STEP_TICKS: u16 = 8;
const BACKOFF_TICKS: u16 = 96;
const HOME_TOLERANCE_TICKS: u16 = 10;
const REPEATABILITY_TOLERANCE_TICKS: u16 = 16;
const BASELINE_MIN_SAMPLES: usize = 6;
const MINIMUM_CONTACT_TRAVEL_TICKS: u16 = 24;
const HARD_CURRENT_ABORT_RAW: u16 = 200;
const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);
const TELEMETRY_TIMEOUT: Duration = Duration::from_secs(2);
const MOTION_TIMEOUT: Duration = Duration::from_secs(12);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactState {
    FreeMotion,
    ContactSuspected,
    ContactConfirmed,
    AmbiguousContact,
    HardAbort,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MotorObservation {
    monotonic_stamp_ns: u64,
    position: u16,
    velocity: u16,
    current: u16,
    goal_position: u16,
    torque_limit: u16,
    torque_enabled: bool,
    status: u8,
    has_driver_error: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BaselineStats {
    median_current: u16,
    mad_current: u16,
}

impl BaselineStats {
    fn from_samples(samples: &[u16]) -> Result<Self, &'static str> {
        if samples.is_empty() {
            return Err("empty current baseline");
        }
        let median_current = median(samples);
        let deviations: Vec<u16> = samples
            .iter()
            .map(|value| value.abs_diff(median_current))
            .collect();
        Ok(Self {
            median_current,
            mad_current: median(&deviations),
        })
    }

    fn contact_threshold(self) -> u16 {
        self.median_current
            .saturating_add(self.mad_current.saturating_mul(4).max(5))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HybridContactConfig {
    max_progress_ticks: u16,
    max_velocity_raw: u16,
    min_goal_error_ticks: u16,
    min_travel_ticks: u16,
    persistence_samples: u8,
    hard_current_abort_raw: u16,
}

impl Default for HybridContactConfig {
    fn default() -> Self {
        Self {
            max_progress_ticks: 2,
            max_velocity_raw: 10,
            min_goal_error_ticks: 4,
            min_travel_ticks: MINIMUM_CONTACT_TRAVEL_TICKS,
            persistence_samples: 3,
            hard_current_abort_raw: HARD_CURRENT_ABORT_RAW,
        }
    }
}

#[derive(Debug)]
struct HybridContactDetector {
    start_position: u16,
    previous_position: u16,
    baseline: BaselineStats,
    config: HybridContactConfig,
    confirming_samples: u8,
    ambiguous_samples: u8,
}

impl HybridContactDetector {
    fn new(
        start_position: u16,
        baseline: BaselineStats,
        config: HybridContactConfig,
    ) -> Self {
        Self {
            start_position,
            previous_position: start_position,
            baseline,
            config,
            confirming_samples: 0,
            ambiguous_samples: 0,
        }
    }

    fn observe(&mut self, observation: MotorObservation, commanded_target: u16) -> ContactState {
        if observation.has_driver_error
            || observation.status != 0
            || !observation.torque_enabled
            || observation.torque_limit != PILOT_TORQUE_LIMIT
            || observation.goal_position != commanded_target
            || observation.current >= self.config.hard_current_abort_raw
        {
            return ContactState::HardAbort;
        }

        let travel = negative_direction_progress(observation.position, self.start_position);
        let progress = negative_direction_progress(observation.position, self.previous_position);
        self.previous_position = observation.position;

        let enough_travel = travel >= self.config.min_travel_ticks;
        let low_progress = progress <= self.config.max_progress_ticks;
        let low_velocity = speed_magnitude(observation.velocity) <= self.config.max_velocity_raw;
        let goal_error = circular_distance(observation.position, commanded_target);
        let target_ahead = signed_tick_delta(commanded_target, observation.position) < 0;
        let current_high = observation.current >= self.baseline.contact_threshold();

        let kinematic_stall = enough_travel
            && low_progress
            && low_velocity
            && goal_error >= self.config.min_goal_error_ticks
            && target_ahead;

        if kinematic_stall && current_high {
            self.confirming_samples = self.confirming_samples.saturating_add(1);
            self.ambiguous_samples = 0;
            if self.confirming_samples >= self.config.persistence_samples {
                ContactState::ContactConfirmed
            } else {
                ContactState::ContactSuspected
            }
        } else if kinematic_stall {
            self.ambiguous_samples = self.ambiguous_samples.saturating_add(1);
            self.confirming_samples = 0;
            if self.ambiguous_samples >= self.config.persistence_samples {
                ContactState::AmbiguousContact
            } else {
                ContactState::ContactSuspected
            }
        } else {
            self.confirming_samples = 0;
            self.ambiguous_samples = 0;
            ContactState::FreeMotion
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ContactResult {
    first_tick: u16,
    second_tick: u16,
    spread_ticks: u16,
    baseline: BaselineStats,
}

fn combine_pilot_and_cleanup(
    pilot: Result<ContactResult, String>,
    cleanup: Result<(), String>,
) -> Result<ContactResult, String> {
    match (pilot, cleanup) {
        (Ok(contact), Ok(())) => Ok(contact),
        (Err(pilot_err), Ok(())) => Err(pilot_err),
        (Ok(_), Err(cleanup_err)) => Err(format!(
            "MATDOG pilot completed but torque-OFF cleanup failed: {cleanup_err}"
        )),
        (Err(pilot_err), Err(cleanup_err)) => Err(format!(
            "{pilot_err}; torque-OFF cleanup also failed: {cleanup_err}"
        )),
    }
}

fn is_allowed_matdog_ram_register(register: RamRegister) -> bool {
    matches!(
        register,
        RamRegister::TorqueEnable
            | RamRegister::Acc
            | RamRegister::GoalPosition
            | RamRegister::GoalSpeed
            | RamRegister::TorqueLimit
    )
}

fn validate_ram_write(register: RamRegister, value: &[u8]) -> Result<(), DynError> {
    if !is_allowed_matdog_ram_register(register) {
        return Err(format!("MATDOG RAM write is not allowlisted: {}", register.name()).into());
    }
    if value.len() != register.size() as usize {
        return Err(format!(
            "MATDOG RAM write size mismatch for {}: expected={}, actual={}",
            register.name(),
            register.size(),
            value.len()
        )
        .into());
    }
    Ok(())
}

fn global_torque_off_writes() -> Vec<(u8, Vec<u8>)> {
    MATDOG_MOTOR_IDS
        .iter()
        .map(|&motor_id| (motor_id, vec![0]))
        .collect()
}

pub fn is_exact_matdog_motor_set(found: &[u8]) -> bool {
    if found.len() != MATDOG_MOTOR_IDS.len() {
        return false;
    }
    let found: BTreeSet<u8> = found.iter().copied().collect();
    found == MATDOG_MOTOR_IDS.into_iter().collect()
}

pub async fn auto_calibrate(
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
) -> Result<Arc<AtomicBool>, Box<dyn std::error::Error>> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err(format!(
            "MATDOG requires exact IDs {:?}; found {:?}",
            MATDOG_MOTOR_IDS, found_motors
        )
        .into());
    }

    let (inference_tx, inference_rx) = watch::channel(InferenceState::default());
    let inference_queue_id = comm.normfs.resolve("st3215/inference");
    let normfs = comm.normfs.clone();
    tokio::spawn(async move {
        let _ = normfs.subscribe(
            &inference_queue_id,
            Box::new(move |entries: &[(normfs::UintN, bytes::Bytes)]| {
                for (_, data) in entries {
                    if let Ok(state) = InferenceState::decode(data.as_ref()) {
                        if inference_tx.send(state).is_err() {
                            return false;
                        }
                    }
                }
                true
            }),
        );
    });

    let stop_requested = Arc::new(AtomicBool::new(false));
    let stop_flag = stop_requested.clone();
    let serial_for_task = target_bus_serial.clone();
    let serial_for_cleanup = target_bus_serial.clone();
    let comm_for_cleanup = comm.clone();

    tokio::spawn(async move {
        if let Err(err) = run_native_m12_min_pilot(
            serial_for_task,
            found_motors,
            comm,
            inference_rx,
            stop_requested,
        )
        .await
        {
            error!("MATDOG native M12 MIN pilot failed: {err}");
        }
        comm_for_cleanup.clear_calibration_stop(&serial_for_cleanup);
    });

    Ok(stop_flag)
}

async fn run_native_m12_min_pilot(
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before pilot start".into());
    }

    let mut calibrator = MatdogRamOnlyCalibrator::new(
        target_bus_serial.clone(),
        comm.clone(),
        inference_rx,
        stop_requested,
    );
    calibrator.total_steps = 11;
    comm.update_calibration_progress(
        &target_bus_serial,
        0,
        calibrator.total_steps,
        "MATDOG native M12 MIN preflight",
        CalibrationStatus::InProgress,
        None,
    );

    let result = calibrator.run_pilot().await.map_err(|err| err.to_string());
    // This is the single final cleanup point for both success and every error path.
    let cleanup = calibrator
        .global_torque_off_verified()
        .await
        .map_err(|err| err.to_string());

    match combine_pilot_and_cleanup(result, cleanup) {
        Ok(contact) => {
            info!(
                "MATDOG M12 MIN complete: first={}, second={}, spread={}, baseline_median={}, baseline_mad={}",
                contact.first_tick,
                contact.second_tick,
                contact.spread_ticks,
                contact.baseline.median_current,
                contact.baseline.mad_current
            );
            calibrator.mark_done();
            Ok(())
        }
        Err(message) => {
            calibrator.mark_failed(&message);
            Err(message.into())
        }
    }
}

struct MatdogRamOnlyCalibrator {
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
    command_nonce: u64,
    command_counter: u64,
    current_step: u32,
    total_steps: u32,
}

impl MatdogRamOnlyCalibrator {
    fn new(
        target_bus_serial: String,
        comm: Arc<ST3215BusCommunicator>,
        inference_rx: watch::Receiver<InferenceState>,
        stop_requested: Arc<AtomicBool>,
    ) -> Self {
        Self {
            target_bus_serial,
            comm,
            inference_rx,
            stop_requested,
            command_nonce: systime::get_monotonic_stamp_ns(),
            command_counter: 0,
            current_step: 0,
            total_steps: 0,
        }
    }

    async fn run_pilot(&mut self) -> Result<ContactResult, DynError> {
        self.next_phase("Verify exact MATDOG ID set")?;
        self.wait_for_exact_motor_set().await?;

        self.next_phase("Verified global torque OFF")?;
        self.global_torque_off_verified().await?;

        self.next_phase("Prime and configure M12 RAM only")?;
        let initial = self.latest_observation(PILOT_MOTOR_ID)?;
        self.ensure_observation_safe(initial, false, None)?;
        self.set_goal_verified(initial.position).await?;
        self.write_ram_verified(
            RamRegister::TorqueLimit,
            PILOT_TORQUE_LIMIT.to_le_bytes().to_vec(),
        )
        .await?;
        self.write_ram_verified(RamRegister::Acc, vec![PILOT_ACCELERATION])
            .await?;
        self.write_ram_verified(
            RamRegister::GoalSpeed,
            PILOT_SPEED.to_le_bytes().to_vec(),
        )
        .await?;
        self.set_torque_verified(true).await?;

        self.next_phase("Return M12 to home 2048")?;
        self.move_to(HOME_TICK, HOME_TOLERANCE_TICKS).await?;

        self.next_phase("Acquire M12 moving-current baseline")?;
        let baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("Coarse hybrid approach to M12 MIN")?;
        let first_tick = self.approach_min(COARSE_STEP_TICKS, baseline).await?;

        self.next_phase("Backoff and verify recovery")?;
        self.backoff_and_verify(first_tick, baseline).await?;

        self.next_phase("Fine hybrid repeat approach to M12 MIN")?;
        let second_tick = self.approach_min(FINE_STEP_TICKS, baseline).await?;

        self.next_phase("Verify repeatability")?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)
            .map_err(|message| -> DynError { message.into() })?;

        self.next_phase("Return M12 home")?;
        self.stop_pressure(second_tick).await?;
        self.move_to(HOME_TICK, HOME_TOLERANCE_TICKS).await?;

        self.next_phase("Final verified global torque OFF")?;

        Ok(ContactResult {
            first_tick,
            second_tick,
            spread_ticks,
            baseline,
        })
    }

    async fn acquire_moving_current_baseline(&mut self) -> Result<BaselineStats, DynError> {
        let initial = self.latest_observation(PILOT_MOTOR_ID)?;
        let mut samples = Vec::new();
        let mut last_stamp = initial.monotonic_stamp_ns;
        let mut previous_position = initial.position;
        self.set_goal_verified(BASELINE_TARGET_TICK).await?;
        let deadline = Instant::now() + MOTION_TIMEOUT;

        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_observation_after(last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(observation, true, Some(BASELINE_TARGET_TICK))?;

            if circular_distance(observation.position, previous_position) > 0
                || speed_magnitude(observation.velocity) > 0
            {
                samples.push(observation.current);
            }
            previous_position = observation.position;

            if circular_distance(observation.position, BASELINE_TARGET_TICK)
                <= HOME_TOLERANCE_TICKS
                && samples.len() >= BASELINE_MIN_SAMPLES
            {
                break;
            }
        }

        if samples.len() < BASELINE_MIN_SAMPLES {
            return Err(format!(
                "insufficient moving baseline samples: {} < {}",
                samples.len(), BASELINE_MIN_SAMPLES
            )
            .into());
        }

        let baseline = BaselineStats::from_samples(&samples)
            .map_err(|message| -> DynError { message.into() })?;
        self.move_to(HOME_TICK, HOME_TOLERANCE_TICKS).await?;
        Ok(baseline)
    }

    async fn approach_min(
        &mut self,
        step_ticks: u16,
        baseline: BaselineStats,
    ) -> Result<u16, DynError> {
        let start = self.latest_observation(PILOT_MOTOR_ID)?;
        self.ensure_observation_safe(start, true, None)?;
        let mut detector = HybridContactDetector::new(
            start.position,
            baseline,
            HybridContactConfig::default(),
        );
        let mut target = start.position;
        let mut last_stamp = start.monotonic_stamp_ns;

        loop {
            self.check_stop()?;
            let next_target = target.saturating_sub(step_ticks);
            if next_target < M12_MIN_GUARD_TICK {
                return Err(format!(
                    "M12 travel guard reached without contact: next={next_target}, URDF_MIN={M12_URDF_MIN_TICK}, guard={M12_MIN_GUARD_TICK}"
                )
                .into());
            }

            self.set_goal_verified(next_target).await?;
            target = next_target;
            let settle_deadline = Instant::now() + Duration::from_millis(700);
            let mut last_observation = None;

            while Instant::now() < settle_deadline {
                let observation = self
                    .wait_for_observation_after(last_stamp, TELEMETRY_TIMEOUT)
                    .await?;
                last_stamp = observation.monotonic_stamp_ns;
                last_observation = Some(observation);

                match detector.observe(observation, target) {
                    ContactState::FreeMotion | ContactState::ContactSuspected => {}
                    ContactState::ContactConfirmed => {
                        self.stop_pressure(observation.position).await?;
                        return Ok(observation.position);
                    }
                    ContactState::AmbiguousContact => {
                        self.stop_pressure(observation.position).await?;
                        return Err(format!(
                            "M12 ambiguous contact: tick={}, current={}, velocity={}",
                            observation.position,
                            observation.current,
                            speed_magnitude(observation.velocity)
                        )
                        .into());
                    }
                    ContactState::HardAbort => {
                        self.stop_pressure(observation.position).await?;
                        return Err(format!(
                            "M12 hard abort: tick={}, goal={}, current={}, torque_enabled={}, torque_limit={}, status=0x{:02X}, driver_error={}",
                            observation.position,
                            observation.goal_position,
                            observation.current,
                            observation.torque_enabled,
                            observation.torque_limit,
                            observation.status,
                            observation.has_driver_error
                        )
                        .into());
                    }
                }
            }

            let observation = last_observation.ok_or("M12 settle window produced no telemetry")?;
            self.ensure_observation_safe(observation, true, Some(target))?;
            if circular_distance(observation.position, target) > step_ticks.saturating_add(4) {
                self.stop_pressure(observation.position).await?;
                return Err(format!(
                    "M12 tracking failed without confirmed contact: target={target}, present={}, current={}",
                    observation.position, observation.current
                )
                .into());
            }
        }
    }

    async fn backoff_and_verify(
        &mut self,
        contact_tick: u16,
        baseline: BaselineStats,
    ) -> Result<(), DynError> {
        let target = contact_tick
            .checked_add(BACKOFF_TICKS)
            .ok_or("M12 backoff overflow")?;
        if target > HOME_TICK {
            return Err(format!("M12 backoff crosses home: {target}").into());
        }
        let recovered = self
            .move_to(target, HOME_TOLERANCE_TICKS.saturating_add(2))
            .await?;
        if recovered.current > baseline.contact_threshold() {
            return Err(format!(
                "M12 current did not recover after backoff: {} > {}",
                recovered.current,
                baseline.contact_threshold()
            )
            .into());
        }
        Ok(())
    }

    async fn stop_pressure(&mut self, present_position: u16) -> Result<(), DynError> {
        self.set_goal_verified(present_position).await
    }

    async fn move_to(&mut self, target: u16, tolerance: u16) -> Result<MotorObservation, DynError> {
        self.set_goal_verified(target).await?;
        let mut last_stamp = self.latest_observation(PILOT_MOTOR_ID)?.monotonic_stamp_ns;
        let deadline = Instant::now() + MOTION_TIMEOUT;

        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_observation_after(last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(observation, true, Some(target))?;
            if circular_distance(observation.position, target) <= tolerance {
                return Ok(observation);
            }
        }
        let last = self.latest_observation(PILOT_MOTOR_ID)?;
        Err(format!(
            "M12 target timeout: target={target}, present={}, error={}",
            last.position,
            circular_distance(last.position, target)
        )
        .into())
    }

    async fn set_goal_verified(&mut self, target: u16) -> Result<(), DynError> {
        if target > protocol::MAX_ANGLE_STEP {
            return Err(format!("unsigned GoalPosition out of range: {target}").into());
        }
        self.write_ram_verified(
            RamRegister::GoalPosition,
            target.to_le_bytes().to_vec(),
        )
        .await
    }

    async fn set_torque_verified(&mut self, enabled: bool) -> Result<(), DynError> {
        self.write_ram_verified(RamRegister::TorqueEnable, vec![u8::from(enabled)])
            .await?;
        let observation = self.latest_observation(PILOT_MOTOR_ID)?;
        if observation.torque_enabled != enabled {
            return Err(format!(
                "M12 torque readback mismatch: expected={enabled}, observed={}",
                observation.torque_enabled
            )
            .into());
        }
        Ok(())
    }

    async fn global_torque_off_verified(&mut self) -> Result<(), DynError> {
        let writes = global_torque_off_writes();
        self.sync_write_ram_verified(RamRegister::TorqueEnable, &writes)
            .await?;
        for motor_id in MATDOG_MOTOR_IDS {
            let observation = observation_from_state(
                &self.current_state(),
                &self.target_bus_serial,
                motor_id,
            )?;
            if observation.torque_enabled {
                return Err(format!("M{motor_id} remained torque-enabled after global OFF").into());
            }
        }
        Ok(())
    }

    async fn write_ram_verified(
        &mut self,
        register: RamRegister,
        value: Vec<u8>,
    ) -> Result<(), DynError> {
        validate_ram_write(register, &value)?;
        let initial_stamp = self.latest_observation(PILOT_MOTOR_ID)?.monotonic_stamp_ns;
        let command_id = self.next_command_id();
        let envelope = TxEnvelope {
            monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
            local_stamp_ns: systime::get_local_stamp_ns(),
            app_start_id: systime::get_app_start_id(),
            target_bus_serial: self.target_bus_serial.clone(),
            command_id: command_id.clone(),
            write: Some(crate::st3215_proto::St3215WriteCommand {
                motor_id: PILOT_MOTOR_ID as u32,
                address: register.address() as u32,
                value: value.clone().into(),
            }),
            ..Default::default()
        };
        self.comm.send_tx(&envelope)?;
        self.wait_for_command_result(&command_id).await?;
        self.wait_for_register_value(
            PILOT_MOTOR_ID,
            register,
            &value,
            initial_stamp,
        )
        .await
    }

    async fn sync_write_ram_verified(
        &mut self,
        register: RamRegister,
        writes: &[(u8, Vec<u8>)],
    ) -> Result<(), DynError> {
        if writes.is_empty() {
            return Err("MATDOG sync-write cannot be empty".into());
        }
        let mut unique_motor_ids = BTreeSet::new();
        for (motor_id, value) in writes {
            validate_ram_write(register, value)?;
            if !MATDOG_MOTOR_IDS.contains(motor_id) {
                return Err(format!("non-MATDOG sync-write motor ID: {motor_id}").into());
            }
            if !unique_motor_ids.insert(*motor_id) {
                return Err(format!("duplicate MATDOG sync-write motor ID: {motor_id}").into());
            }
        }
        let initial_stamps: Vec<(u8, u64)> = writes
            .iter()
            .map(|(motor_id, _)| {
                observation_from_state(
                    &self.current_state(),
                    &self.target_bus_serial,
                    *motor_id,
                )
                .map(|observation| (*motor_id, observation.monotonic_stamp_ns))
            })
            .collect::<Result<_, _>>()?;
        let command_id = self.next_command_id();
        let envelope = TxEnvelope {
            monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
            local_stamp_ns: systime::get_local_stamp_ns(),
            app_start_id: systime::get_app_start_id(),
            target_bus_serial: self.target_bus_serial.clone(),
            command_id: command_id.clone(),
            sync_write: Some(crate::st3215_proto::St3215SyncWriteCommand {
                address: register.address() as u32,
                motors: writes
                    .iter()
                    .map(|(motor_id, value)| {
                        crate::st3215_proto::st3215_sync_write_command::MotorWrite {
                            motor_id: *motor_id as u32,
                            value: value.clone().into(),
                        }
                    })
                    .collect(),
            }),
            ..Default::default()
        };
        self.comm.send_tx(&envelope)?;
        self.wait_for_command_result(&command_id).await?;

        for ((motor_id, value), (_, initial_stamp)) in writes.iter().zip(initial_stamps) {
            self.wait_for_register_value(*motor_id, register, value, initial_stamp)
                .await?;
        }
        Ok(())
    }

    async fn wait_for_register_value(
        &mut self,
        motor_id: u8,
        register: RamRegister,
        expected: &[u8],
        initial_stamp: u64,
    ) -> Result<(), DynError> {
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        let mut last_stamp = initial_stamp;

        while Instant::now() < deadline {
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            let state = self.current_state();
            let motor = find_motor(&state, &self.target_bus_serial, motor_id)?;
            if motor_ram_register_matches(motor, register, expected) {
                return Ok(());
            }
        }
        Err(format!(
            "M{motor_id} RAM readback timeout for {} at 0x{:02X}",
            register.name(),
            register.address()
        )
        .into())
    }

    async fn wait_for_command_result(&mut self, command_id: &Bytes) -> Result<(), DynError> {
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        loop {
            let state = self.current_state();
            if let Some(result) = command_result_for(&state, &self.target_bus_serial, command_id) {
                match CommandResult::try_from(result) {
                    Ok(CommandResult::CrSuccess) => return Ok(()),
                    Ok(CommandResult::CrRejected) => {
                        return Err("ST3215 command rejected".into());
                    }
                    Ok(CommandResult::CrFailed) => return Err("ST3215 command failed".into()),
                    Ok(CommandResult::CrProcessing) => {}
                    Err(_) => {
                        return Err(format!("invalid ST3215 command result: {result}").into());
                    }
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() {
                        return Err("ST3215 inference channel closed".into());
                    }
                }
                _ = tokio::time::sleep_until(deadline) => {
                    return Err("ST3215 command result timeout".into());
                }
            }
        }
    }

    async fn wait_for_exact_motor_set(&mut self) -> Result<(), DynError> {
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        loop {
            let state = self.current_state();
            if let Ok(found) = motor_ids_for_bus(&state, &self.target_bus_serial) {
                if is_exact_matdog_motor_set(&found) {
                    return Ok(());
                }
                if found.len() >= MATDOG_MOTOR_IDS.len() {
                    return Err(format!(
                        "MATDOG inference ID mismatch: expected {:?}, found {:?}",
                        MATDOG_MOTOR_IDS, found
                    )
                    .into());
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() {
                        return Err("ST3215 inference channel closed".into());
                    }
                }
                _ = tokio::time::sleep_until(deadline) => {
                    return Err("MATDOG exact ID set timeout".into());
                }
            }
        }
    }

    fn latest_observation(&self, motor_id: u8) -> Result<MotorObservation, DynError> {
        observation_from_state(&self.current_state(), &self.target_bus_serial, motor_id)
    }

    async fn wait_for_observation_after(
        &mut self,
        minimum_stamp: u64,
        timeout: Duration,
    ) -> Result<MotorObservation, DynError> {
        self.wait_for_motor_observation_after(PILOT_MOTOR_ID, minimum_stamp, timeout)
            .await
    }

    async fn wait_for_motor_observation_after(
        &mut self,
        motor_id: u8,
        minimum_stamp: u64,
        timeout: Duration,
    ) -> Result<MotorObservation, DynError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Ok(observation) = observation_from_state(
                &self.current_state(),
                &self.target_bus_serial,
                motor_id,
            ) {
                if observation.monotonic_stamp_ns > minimum_stamp {
                    return Ok(observation);
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() {
                        return Err("ST3215 inference channel closed".into());
                    }
                }
                _ = tokio::time::sleep_until(deadline) => {
                    return Err(format!("M{motor_id} fresh telemetry timeout").into());
                }
            }
        }
    }

    fn current_state(&self) -> InferenceState {
        self.inference_rx.borrow().clone()
    }

    fn ensure_observation_safe(
        &self,
        observation: MotorObservation,
        require_torque: bool,
        expected_goal: Option<u16>,
    ) -> Result<(), DynError> {
        if observation.has_driver_error {
            return Err("M12 driver error present".into());
        }
        if observation.status != 0 {
            return Err(format!("M12 servo status is 0x{:02X}", observation.status).into());
        }
        if require_torque && !observation.torque_enabled {
            return Err("M12 torque unexpectedly disabled".into());
        }
        if require_torque && observation.torque_limit != PILOT_TORQUE_LIMIT {
            return Err(format!(
                "M12 torque-limit readback changed: expected={}, observed={}",
                PILOT_TORQUE_LIMIT, observation.torque_limit
            )
            .into());
        }
        if let Some(expected_goal) = expected_goal {
            if observation.goal_position != expected_goal {
                return Err(format!(
                    "M12 goal-position readback changed: expected={expected_goal}, observed={}",
                    observation.goal_position
                )
                .into());
            }
        }
        if observation.current >= HARD_CURRENT_ABORT_RAW {
            return Err(format!(
                "M12 hard current abort: {} >= {}",
                observation.current, HARD_CURRENT_ABORT_RAW
            )
            .into());
        }
        Ok(())
    }

    fn check_stop(&self) -> Result<(), DynError> {
        if self.stop_requested.load(Ordering::Relaxed) {
            Err("MATDOG calibration stopped by operator".into())
        } else {
            Ok(())
        }
    }

    fn next_phase(&mut self, phase: &str) -> Result<(), DynError> {
        self.check_stop()?;
        self.current_step += 1;
        self.comm.update_calibration_progress(
            &self.target_bus_serial,
            self.current_step,
            self.total_steps,
            phase,
            CalibrationStatus::InProgress,
            None,
        );
        Ok(())
    }

    fn mark_done(&self) {
        self.comm.update_calibration_progress(
            &self.target_bus_serial,
            self.total_steps,
            self.total_steps,
            "MATDOG M12 MIN completed",
            CalibrationStatus::Done,
            None,
        );
    }

    fn mark_failed(&self, message: &str) {
        self.comm.update_calibration_progress(
            &self.target_bus_serial,
            self.current_step,
            self.total_steps,
            "MATDOG M12 MIN failed",
            CalibrationStatus::Failed,
            Some(message),
        );
    }

    fn next_command_id(&mut self) -> Bytes {
        self.command_counter += 1;
        make_command_id(
            systime::get_app_start_id(),
            self.command_nonce,
            self.command_counter,
        )
    }
}

fn find_motor<'a>(
    state: &'a InferenceState,
    bus_serial: &str,
    motor_id: u8,
) -> Result<&'a crate::st3215_proto::inference_state::MotorState, DynError> {
    let bus = state
        .buses
        .iter()
        .find(|bus| bus.bus.as_ref().map(|bus| bus.serial_number.as_str()) == Some(bus_serial))
        .ok_or_else(|| format!("ST3215 bus not found: {bus_serial}"))?;
    bus.motors
        .iter()
        .find(|motor| motor.id == motor_id as u32)
        .ok_or_else(|| format!("M{motor_id} not found on bus {bus_serial}").into())
}

fn motor_ids_for_bus(state: &InferenceState, bus_serial: &str) -> Result<Vec<u8>, DynError> {
    let bus = state
        .buses
        .iter()
        .find(|bus| bus.bus.as_ref().map(|bus| bus.serial_number.as_str()) == Some(bus_serial))
        .ok_or_else(|| format!("ST3215 bus not found: {bus_serial}"))?;
    bus.motors
        .iter()
        .map(|motor| -> Result<u8, DynError> {
            u8::try_from(motor.id)
                .map_err(|_| format!("invalid ST3215 motor ID in inference: {}", motor.id).into())
        })
        .collect()
}

fn motor_ram_register_matches(
    motor: &crate::st3215_proto::inference_state::MotorState,
    register: RamRegister,
    expected: &[u8],
) -> bool {
    let address = register.address() as usize;
    motor.state.len() >= address + expected.len()
        && &motor.state[address..address + expected.len()] == expected
}

fn observation_from_state(
    state: &InferenceState,
    bus_serial: &str,
    motor_id: u8,
) -> Result<MotorObservation, DynError> {
    let motor = find_motor(state, bus_serial, motor_id)?;
    let bytes = motor.state.as_ref();
    let torque_limit_addr = RamRegister::TorqueLimit.address() as usize;
    let status_addr = RamRegister::Status.address() as usize;
    if bytes.len() < RamRegister::PresentCurrent.address() as usize + 2
        || bytes.len() < torque_limit_addr + 2
        || bytes.len() <= status_addr
    {
        return Err(format!("M{motor_id} state too short: {} bytes", bytes.len()).into());
    }
    Ok(MotorObservation {
        monotonic_stamp_ns: motor.monotonic_stamp_ns,
        position: protocol::get_motor_position(bytes),
        velocity: protocol::get_motor_velocity(bytes),
        current: protocol::get_motor_current(bytes),
        goal_position: protocol::get_motor_goal_position(bytes),
        torque_limit: u16::from_le_bytes([
            bytes[torque_limit_addr],
            bytes[torque_limit_addr + 1],
        ]),
        torque_enabled: protocol::is_torque_enabled(bytes),
        status: bytes[status_addr],
        has_driver_error: motor.error.is_some(),
    })
}

fn command_result_for(state: &InferenceState, bus_serial: &str, command_id: &Bytes) -> Option<i32> {
    let bus = state
        .buses
        .iter()
        .find(|bus| bus.bus.as_ref().map(|bus| bus.serial_number.as_str()) == Some(bus_serial))?;
    bus.motors.iter().find_map(|motor| {
        let last = motor.last_command.as_ref()?;
        let command = last.command.as_ref()?;
        if &command.command_id == command_id {
            Some(last.result)
        } else {
            None
        }
    })
}

fn median(values: &[u16]) -> u16 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

fn repeatability_spread(first_tick: u16, second_tick: u16) -> Result<u16, String> {
    let spread_ticks = circular_distance(first_tick, second_tick);
    if spread_ticks > REPEATABILITY_TOLERANCE_TICKS {
        Err(format!(
            "M12 contact not repeatable: first={first_tick}, second={second_tick}, spread={spread_ticks}"
        ))
    } else {
        Ok(spread_ticks)
    }
}

fn make_command_id(app_start_id: u64, nonce: u64, counter: u64) -> Bytes {
    let mut bytes = Vec::with_capacity(24);
    bytes.extend_from_slice(&app_start_id.to_le_bytes());
    bytes.extend_from_slice(&nonce.to_le_bytes());
    bytes.extend_from_slice(&counter.to_le_bytes());
    Bytes::from(bytes)
}

fn signed_tick_delta(value: u16, reference: u16) -> i16 {
    let delta = (value as i32 - reference as i32 + 2048).rem_euclid(4096) - 2048;
    delta as i16
}

fn circular_distance(a: u16, b: u16) -> u16 {
    signed_tick_delta(a, b).unsigned_abs()
}

fn negative_direction_progress(value: u16, reference: u16) -> u16 {
    (-i32::from(signed_tick_delta(value, reference))).max(0) as u16
}

fn speed_magnitude(raw: u16) -> u16 {
    raw & 0x7FFF
}

#[cfg(test)]
#[path = "matdog_test.rs"]
mod tests;
