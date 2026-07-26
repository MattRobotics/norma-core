//! MATDOG-specific, data-driven, RAM-only ST3215 mechanical end-stop calibrator.
//!
//! One explicitly armed MIN/MAX contact profile is executed per run.  The
//! active probing joint is the only joint whose target advances during contact
//! search.  Prerequisite joints are moved sequentially to geometry-validated
//! static poses, monitored for drift, and restored before the mandatory global
//! torque-OFF cleanup.

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
pub(crate) const MATDOG_ARM_ENV: &str = "MATDOG_NATIVE_CALIBRATOR_ARM";

const HOME_TICK: u16 = 2048;
const TICKS_PER_REVOLUTION: i32 = 4096;
const GUARD_OVERSHOOT_TICKS: u16 = 64;
const BASELINE_TRAVEL_TICKS: u16 = 64;
const TORQUE_LIMIT: u16 = 400;
const GOAL_SPEED: u16 = 80;
const ACCELERATION: u8 = 4;
const COARSE_STEP_TICKS: u16 = 32;
const FINE_STEP_TICKS: u16 = 8;
const BACKOFF_TICKS: u16 = 96;
const STATIC_TOLERANCE_TICKS: u16 = 10;
const REPEATABILITY_TOLERANCE_TICKS: u16 = 16;
const BASELINE_MIN_SAMPLES: usize = 6;
const MINIMUM_CONTACT_TRAVEL_TICKS: u16 = 24;
const TARGET_STARTUP_SAMPLES: u8 = 4;
const CONTACT_SETTLE_WINDOW: Duration = Duration::from_millis(1500);
const HARD_CURRENT_ABORT_RAW: u16 = 200;
const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);
const TELEMETRY_TIMEOUT: Duration = Duration::from_secs(2);
const MAX_TELEMETRY_AGE: Duration = Duration::from_secs(3);
const MOTION_TIMEOUT: Duration = Duration::from_secs(12);

const LF_ALLOWED: [u8; 4] = [11, 12, 13, 42];
const RF_ALLOWED: [u8; 4] = [21, 22, 23, 32];
const RH_ALLOWED: [u8; 3] = [31, 32, 33];
const LH_ALLOWED: [u8; 3] = [41, 42, 43];

const HIP_MIN_DELTA: i16 = -512;
const HIP_MAX_DELTA: i16 = 512;
const UPPER_MIN_DELTA: i16 = -597;
const UPPER_MAX_DELTA: i16 = 1394;
const LOWER_MIN_DELTA: i16 = -1047;
const LOWER_MAX_DELTA: i16 = 427;
const UPPER_30_DELTA: i16 = 341;
const UPPER_50_DELTA: i16 = 569;
const UPPER_90_DELTA: i16 = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Leg {
    Lf,
    Rf,
    Rh,
    Lh,
}

impl Leg {
    const fn label(self) -> &'static str {
        match self {
            Self::Lf => "LF",
            Self::Rf => "RF",
            Self::Rh => "RH",
            Self::Lh => "LH",
        }
    }

    const fn allowed_motor_ids(self) -> &'static [u8] {
        match self {
            Self::Lf => &LF_ALLOWED,
            Self::Rf => &RF_ALLOWED,
            Self::Rh => &RH_ALLOWED,
            Self::Lh => &LH_ALLOWED,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JointKind {
    Hip,
    Upper,
    Lower,
}

impl JointKind {
    const fn label(self) -> &'static str {
        match self {
            Self::Hip => "HIP",
            Self::Upper => "UPPER",
            Self::Lower => "LOWER",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ContactSide {
    Min,
    Max,
}

impl ContactSide {
    const fn label(self) -> &'static str {
        match self {
            Self::Min => "MIN",
            Self::Max => "MAX",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct JointSpec {
    leg: Leg,
    kind: JointKind,
    name: &'static str,
    motor_id: u8,
    direction: i8,
    min_delta: i16,
    max_delta: i16,
}

impl JointSpec {
    const fn limit_delta(self, side: ContactSide) -> i16 {
        match side {
            ContactSide::Min => self.min_delta,
            ContactSide::Max => self.max_delta,
        }
    }

    fn tick_for_delta(self, q_delta: i16) -> Result<u16, String> {
        let tick = i32::from(HOME_TICK) + i32::from(self.direction) * i32::from(q_delta);
        u16::try_from(tick)
            .ok()
            .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
            .ok_or_else(|| {
                format!(
                    "{} target is outside unsigned ST3215 range: {tick}",
                    self.name
                )
            })
    }
}

const JOINT_SPECS: [JointSpec; 12] = [
    JointSpec {
        leg: Leg::Lf,
        kind: JointKind::Hip,
        name: "lf_hip_joint",
        motor_id: 13,
        direction: -1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lf,
        kind: JointKind::Upper,
        name: "lf_upper_leg_joint",
        motor_id: 12,
        direction: 1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lf,
        kind: JointKind::Lower,
        name: "lf_lower_leg_joint",
        motor_id: 11,
        direction: -1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rf,
        kind: JointKind::Hip,
        name: "rf_hip_joint",
        motor_id: 23,
        direction: -1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rf,
        kind: JointKind::Upper,
        name: "rf_upper_leg_joint",
        motor_id: 22,
        direction: -1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rf,
        kind: JointKind::Lower,
        name: "rf_lower_leg_joint",
        motor_id: 21,
        direction: 1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rh,
        kind: JointKind::Hip,
        name: "rh_hip_joint",
        motor_id: 33,
        direction: 1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rh,
        kind: JointKind::Upper,
        name: "rh_upper_leg_joint",
        motor_id: 32,
        direction: -1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rh,
        kind: JointKind::Lower,
        name: "rh_lower_leg_joint",
        motor_id: 31,
        direction: 1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lh,
        kind: JointKind::Hip,
        name: "lh_hip_joint",
        motor_id: 43,
        direction: 1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lh,
        kind: JointKind::Upper,
        name: "lh_upper_leg_joint",
        motor_id: 42,
        direction: 1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lh,
        kind: JointKind::Lower,
        name: "lh_lower_leg_joint",
        motor_id: 41,
        direction: -1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StaticTarget {
    motor_id: u8,
    target_tick: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ContactProfile {
    pub(crate) arm_value: String,
    pub(crate) label: String,
    pub(crate) leg: Leg,
    pub(crate) joint: JointKind,
    pub(crate) side: ContactSide,
    pub(crate) joint_name: &'static str,
    pub(crate) motor_id: u8,
    pub(crate) probe_sign: i8,
    pub(crate) urdf_limit_tick: u16,
    pub(crate) guard_tick: u16,
    pub(crate) baseline_target_tick: u16,
    pub(crate) allowed_motor_ids: &'static [u8],
    prerequisites: Vec<StaticTarget>,
}

fn spec_for(leg: Leg, kind: JointKind) -> &'static JointSpec {
    JOINT_SPECS
        .iter()
        .find(|spec| spec.leg == leg && spec.kind == kind)
        .expect("complete MATDOG joint table")
}

fn static_target(leg: Leg, kind: JointKind, q_delta: i16) -> Result<StaticTarget, String> {
    let spec = spec_for(leg, kind);
    Ok(StaticTarget {
        motor_id: spec.motor_id,
        target_tick: spec.tick_for_delta(q_delta)?,
    })
}

fn prerequisites_for(leg: Leg, kind: JointKind) -> Result<Vec<StaticTarget>, String> {
    let mut targets = Vec::new();
    match leg {
        Leg::Lf => targets.push(static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)?),
        Leg::Rf => targets.push(static_target(Leg::Rh, JointKind::Upper, UPPER_30_DELTA)?),
        Leg::Rh | Leg::Lh => {}
    }

    match kind {
        JointKind::Upper => {
            targets.push(static_target(leg, JointKind::Hip, 0)?);
            targets.push(static_target(leg, JointKind::Lower, 0)?);
        }
        JointKind::Hip => {
            targets.push(static_target(leg, JointKind::Upper, UPPER_50_DELTA)?);
            targets.push(static_target(leg, JointKind::Lower, 0)?);
        }
        JointKind::Lower => {
            targets.push(static_target(leg, JointKind::Hip, 0)?);
            targets.push(static_target(leg, JointKind::Upper, UPPER_90_DELTA)?);
        }
    }
    Ok(targets)
}

#[cfg(test)]
fn prerequisite_restore_order(prerequisites: &[StaticTarget], probing_motor_id: u8) -> Vec<u8> {
    prerequisites
        .iter()
        .filter(|target| target.motor_id != probing_motor_id)
        .rev()
        .map(|target| target.motor_id)
        .collect()
}

fn build_profile(leg: Leg, joint: JointKind, side: ContactSide) -> Result<ContactProfile, String> {
    let spec = *spec_for(leg, joint);
    let urdf_limit_tick = spec.tick_for_delta(spec.limit_delta(side))?;
    let q_sign = match side {
        ContactSide::Min => -1,
        ContactSide::Max => 1,
    };
    let probe_sign = spec.direction * q_sign;
    let guard =
        i32::from(urdf_limit_tick) + i32::from(probe_sign) * i32::from(GUARD_OVERSHOOT_TICKS);
    let baseline = i32::from(HOME_TICK) + i32::from(probe_sign) * i32::from(BASELINE_TRAVEL_TICKS);
    let guard_tick = u16::try_from(guard)
        .ok()
        .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| {
            format!(
                "{} {} guard leaves unsigned range: {guard}",
                spec.name,
                side.label()
            )
        })?;
    let baseline_target_tick = u16::try_from(baseline)
        .ok()
        .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| format!("{} baseline leaves unsigned range: {baseline}", spec.name))?;

    let arm_value = format!(
        "{}_{}_M{}_{}",
        leg.label(),
        joint.label(),
        spec.motor_id,
        side.label()
    );
    Ok(ContactProfile {
        label: arm_value.clone(),
        arm_value,
        leg,
        joint,
        side,
        joint_name: spec.name,
        motor_id: spec.motor_id,
        probe_sign,
        urdf_limit_tick,
        guard_tick,
        baseline_target_tick,
        allowed_motor_ids: leg.allowed_motor_ids(),
        prerequisites: prerequisites_for(leg, joint)?,
    })
}

pub(crate) fn all_profiles() -> Result<Vec<ContactProfile>, String> {
    let mut profiles = Vec::with_capacity(24);
    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Upper, JointKind::Hip, JointKind::Lower] {
            for side in [ContactSide::Min, ContactSide::Max] {
                profiles.push(build_profile(leg, joint, side)?);
            }
        }
    }
    Ok(profiles)
}

pub(crate) fn profile_for_arm_value(value: &str) -> Result<ContactProfile, String> {
    all_profiles()?
        .into_iter()
        .find(|profile| profile.arm_value == value)
        .ok_or_else(|| {
            let supported = all_profiles()
                .unwrap_or_default()
                .into_iter()
                .map(|profile| profile.arm_value)
                .collect::<Vec<_>>()
                .join(", ");
            format!("unsupported {MATDOG_ARM_ENV}={value:?}; expected one of: {supported}")
        })
}

pub(crate) fn active_profile() -> Result<ContactProfile, String> {
    match std::env::var(MATDOG_ARM_ENV) {
        Ok(value) => profile_for_arm_value(&value),
        Err(_) => Err(format!(
            "MATDOG calibrator is not armed: set {MATDOG_ARM_ENV} explicitly"
        )),
    }
}

pub(crate) fn armed_ram_write_allowed(motor_id: u8, address: u32, value: &[u8]) -> bool {
    let Ok(profile) = active_profile() else {
        return false;
    };
    ram_write_allowed_for_profile(&profile, motor_id, address, value)
}

pub(crate) fn ram_write_allowed_for_profile(
    profile: &ContactProfile,
    motor_id: u8,
    address: u32,
    value: &[u8],
) -> bool {
    if !profile.allowed_motor_ids.contains(&motor_id) {
        return false;
    }

    let register = [
        RamRegister::TorqueEnable,
        RamRegister::Acc,
        RamRegister::GoalPosition,
        RamRegister::GoalSpeed,
        RamRegister::TorqueLimit,
    ]
    .into_iter()
    .find(|register| {
        register.address() as u32 == address && register.size() as usize == value.len()
    });

    match register {
        Some(RamRegister::TorqueEnable) => matches!(value, [0] | [1]),
        Some(RamRegister::Acc) => value == [ACCELERATION],
        Some(RamRegister::GoalSpeed) => value == GOAL_SPEED.to_le_bytes(),
        Some(RamRegister::TorqueLimit) => value == TORQUE_LIMIT.to_le_bytes(),
        Some(RamRegister::GoalPosition) => {
            let target = u16::from_le_bytes([value[0], value[1]]);
            armed_goal_target_allowed(profile, motor_id, target)
        }
        _ => false,
    }
}

fn armed_goal_target_allowed(profile: &ContactProfile, motor_id: u8, target: u16) -> bool {
    if motor_id == profile.motor_id {
        let low = profile
            .guard_tick
            .min(HOME_TICK)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = profile
            .guard_tick
            .max(HOME_TICK)
            .saturating_add(STATIC_TOLERANCE_TICKS);
        return (low..=high.min(protocol::MAX_ANGLE_STEP)).contains(&target);
    }

    let Some(prerequisite) = profile
        .prerequisites
        .iter()
        .find(|prerequisite| prerequisite.motor_id == motor_id)
    else {
        return false;
    };
    let low = prerequisite
        .target_tick
        .min(HOME_TICK)
        .saturating_sub(STATIC_TOLERANCE_TICKS);
    let high = prerequisite
        .target_tick
        .max(HOME_TICK)
        .saturating_add(STATIC_TOLERANCE_TICKS)
        .min(protocol::MAX_ANGLE_STEP);
    (low..=high).contains(&target)
}

pub fn is_exact_matdog_motor_set(found: &[u8]) -> bool {
    if found.len() != MATDOG_MOTOR_IDS.len() {
        return false;
    }
    let found: BTreeSet<u8> = found.iter().copied().collect();
    found == MATDOG_MOTOR_IDS.into_iter().collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactState {
    FreeMotion,
    ContactSuspected,
    ContactConfirmed,
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
    target_reached_tolerance_ticks: u16,
    min_travel_ticks: u16,
    persistence_samples: u8,
    hard_current_abort_raw: u16,
}

impl Default for HybridContactConfig {
    fn default() -> Self {
        Self {
            max_progress_ticks: 2,
            max_velocity_raw: 10,
            target_reached_tolerance_ticks: STATIC_TOLERANCE_TICKS,
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
    probe_sign: i8,
    confirming_samples: u8,
    active_target: Option<u16>,
    target_samples_seen: u8,
}

impl HybridContactDetector {
    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {
        Self {
            start_position,
            previous_position: start_position,
            baseline,
            config: HybridContactConfig::default(),
            probe_sign,
            confirming_samples: 0,
            active_target: None,
            target_samples_seen: 0,
        }
    }

    fn observe(&mut self, observation: MotorObservation, commanded_target: u16) -> ContactState {
        if observation.has_driver_error
            || observation.status != 0
            || !observation.torque_enabled
            || observation.torque_limit != TORQUE_LIMIT
            || observation.goal_position != commanded_target
            || observation.current >= self.config.hard_current_abort_raw
        {
            return ContactState::HardAbort;
        }

        if self.active_target != Some(commanded_target) {
            self.active_target = Some(commanded_target);
            self.target_samples_seen = 0;
            self.previous_position = observation.position;
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }
        self.target_samples_seen = self.target_samples_seen.saturating_add(1);

        let travel =
            directional_progress(observation.position, self.start_position, self.probe_sign);
        let progress = directional_progress(
            observation.position,
            self.previous_position,
            self.probe_sign,
        );
        self.previous_position = observation.position;
        let low_velocity = speed_magnitude(observation.velocity) <= self.config.max_velocity_raw;
        let low_progress = progress <= self.config.max_progress_ticks;
        let enough_travel = travel >= self.config.min_travel_ticks;
        let goal_error = circular_distance(observation.position, commanded_target);
        let target_ahead = i32::from(signed_tick_delta(commanded_target, observation.position))
            * i32::from(self.probe_sign)
            > 0;
        let _current_supports_contact = observation.current >= self.baseline.contact_threshold();

        if goal_error <= self.config.target_reached_tolerance_ticks {
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }
        if self.target_samples_seen <= TARGET_STARTUP_SAMPLES {
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }

        if enough_travel && low_progress && low_velocity && target_ahead {
            self.confirming_samples = self.confirming_samples.saturating_add(1);
            if self.confirming_samples >= self.config.persistence_samples {
                ContactState::ContactConfirmed
            } else {
                ContactState::ContactSuspected
            }
        } else {
            self.confirming_samples = 0;
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
    let profile =
        active_profile().map_err(|message| -> Box<dyn std::error::Error> { message.into() })?;

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
        if let Err(err) = run_profile(
            profile,
            serial_for_task,
            found_motors,
            comm,
            inference_rx,
            stop_requested,
        )
        .await
        {
            error!("MATDOG native profile failed: {err}");
        }
        comm_for_cleanup.clear_calibration_stop(&serial_for_cleanup);
    });

    Ok(stop_flag)
}

async fn run_profile(
    profile: ContactProfile,
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before profile start".into());
    }

    let mut calibrator = MatdogRamOnlyCalibrator::new(
        profile,
        target_bus_serial.clone(),
        comm.clone(),
        inference_rx,
        stop_requested,
    );
    calibrator.total_steps = 13;
    calibrator.publish_progress(
        0,
        "MATDOG native profile preflight",
        CalibrationStatus::InProgress,
        None,
    );

    let result = calibrator.run().await.map_err(|err| err.to_string());
    let cleanup = calibrator
        .global_torque_off_verified()
        .await
        .map_err(|err| err.to_string());
    match (result, cleanup) {
        (Ok(contact), Ok(())) => {
            info!(
                "MATDOG {} complete: first={}, second={}, spread={}, baseline_median={}, baseline_mad={}",
                calibrator.profile.label,
                contact.first_tick,
                contact.second_tick,
                contact.spread_ticks,
                contact.baseline.median_current,
                contact.baseline.mad_current
            );
            calibrator.mark_done();
            Ok(())
        }
        (Err(run_err), Ok(())) => {
            calibrator.mark_failed(&run_err);
            Err(run_err.into())
        }
        (Ok(_), Err(cleanup_err)) => {
            let message =
                format!("MATDOG profile completed but torque-OFF cleanup failed: {cleanup_err}");
            calibrator.mark_failed(&message);
            Err(message.into())
        }
        (Err(run_err), Err(cleanup_err)) => {
            let message = format!("{run_err}; torque-OFF cleanup also failed: {cleanup_err}");
            calibrator.mark_failed(&message);
            Err(message.into())
        }
    }
}

struct MatdogRamOnlyCalibrator {
    profile: ContactProfile,
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
    command_nonce: u64,
    command_counter: u64,
    current_step: u32,
    total_steps: u32,
    held_targets: Vec<StaticTarget>,
}

impl MatdogRamOnlyCalibrator {
    fn new(
        profile: ContactProfile,
        target_bus_serial: String,
        comm: Arc<ST3215BusCommunicator>,
        inference_rx: watch::Receiver<InferenceState>,
        stop_requested: Arc<AtomicBool>,
    ) -> Self {
        Self {
            profile,
            target_bus_serial,
            comm,
            inference_rx,
            stop_requested,
            command_nonce: systime::get_monotonic_stamp_ns(),
            command_counter: 0,
            current_step: 0,
            total_steps: 0,
            held_targets: Vec::new(),
        }
    }

    async fn run(&mut self) -> Result<ContactResult, DynError> {
        self.next_phase("Verify exact MATDOG ID set")?;
        self.wait_for_exact_motor_set().await?;

        self.next_phase("Verified global torque OFF")?;
        self.global_torque_off_verified().await?;

        self.next_phase("Verify all joints near digital home")?;
        self.verify_all_near_home().await?;

        self.next_phase("Apply geometry prerequisites one joint at a time")?;
        self.apply_prerequisites().await?;

        self.next_phase("Prime and configure probing joint RAM only")?;
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
            .await?;

        self.next_phase("Acquire moving-current baseline")?;
        let baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("Coarse approach")?;
        let first_tick = self.approach(COARSE_STEP_TICKS, baseline).await?;

        self.next_phase("Backoff and verify recovery")?;
        self.backoff_and_verify(first_tick, baseline).await?;

        self.next_phase("Fine repeat approach")?;
        let second_tick = self.approach(FINE_STEP_TICKS, baseline).await?;

        self.next_phase("Verify repeatability")?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)?;

        self.next_phase("Return probing joint home")?;
        self.stop_pressure(self.profile.motor_id, second_tick)
            .await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
            .await?;

        self.next_phase("Restore prerequisite joints one at a time")?;
        self.restore_prerequisites().await?;

        self.next_phase("Final verified global torque OFF")?;

        Ok(ContactResult {
            first_tick,
            second_tick,
            spread_ticks,
            baseline,
        })
    }

    async fn verify_all_near_home(&mut self) -> Result<(), DynError> {
        for motor_id in MATDOG_MOTOR_IDS {
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;
            if observation.torque_enabled {
                return Err(format!(
                    "M{motor_id} unexpectedly torque-enabled during home preflight"
                )
                .into());
            }
            if observation.has_driver_error || observation.status != 0 {
                return Err(format!("M{motor_id} unhealthy during home preflight").into());
            }
            if circular_distance(observation.position, HOME_TICK) > STATIC_TOLERANCE_TICKS {
                return Err(format!(
                    "M{motor_id} is not at digital home: present={}, expected={}, tolerance={}",
                    observation.position, HOME_TICK, STATIC_TOLERANCE_TICKS
                )
                .into());
            }
        }
        Ok(())
    }

    async fn apply_prerequisites(&mut self) -> Result<(), DynError> {
        for target in self.profile.prerequisites.clone() {
            if target.motor_id == self.profile.motor_id {
                continue;
            }
            self.prepare_motor(target.motor_id).await?;
            self.move_motor_to(target.motor_id, target.target_tick, STATIC_TOLERANCE_TICKS)
                .await?;
            if self
                .held_targets
                .iter()
                .any(|held| held.motor_id == target.motor_id)
            {
                return Err(
                    format!("duplicate prerequisite target for M{}", target.motor_id).into(),
                );
            }
            self.held_targets.push(target);
            self.verify_static_holds().await?;
        }
        Ok(())
    }

    async fn restore_prerequisites(&mut self) -> Result<(), DynError> {
        while let Some(target) = self.held_targets.pop() {
            self.move_motor_to(target.motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
                .await?;
            self.set_motor_torque_verified(target.motor_id, false)
                .await?;
        }
        Ok(())
    }

    async fn prepare_motor(&mut self, motor_id: u8) -> Result<(), DynError> {
        if !self.profile.allowed_motor_ids.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside armed profile motor allowlist").into());
        }
        let initial = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, initial, false, None)?;
        self.set_motor_goal_verified(motor_id, initial.position)
            .await?;
        self.write_motor_ram_verified(
            motor_id,
            RamRegister::TorqueLimit,
            TORQUE_LIMIT.to_le_bytes().to_vec(),
        )
        .await?;
        self.write_motor_ram_verified(motor_id, RamRegister::Acc, vec![ACCELERATION])
            .await?;
        self.write_motor_ram_verified(
            motor_id,
            RamRegister::GoalSpeed,
            GOAL_SPEED.to_le_bytes().to_vec(),
        )
        .await?;
        self.set_motor_torque_verified(motor_id, true).await
    }

    async fn acquire_moving_current_baseline(&mut self) -> Result<BaselineStats, DynError> {
        let motor_id = self.profile.motor_id;
        let initial = self.latest_observation(motor_id)?;
        let mut samples = Vec::new();
        let mut last_stamp = initial.monotonic_stamp_ns;
        let mut previous_position = initial.position;
        self.set_motor_goal_verified(motor_id, self.profile.baseline_target_tick)
            .await?;
        let deadline = Instant::now() + MOTION_TIMEOUT;

        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(
                motor_id,
                observation,
                true,
                Some(self.profile.baseline_target_tick),
            )?;
            self.verify_static_holds().await?;
            if circular_distance(observation.position, previous_position) > 0
                || speed_magnitude(observation.velocity) > 0
            {
                samples.push(observation.current);
            }
            previous_position = observation.position;
            if circular_distance(observation.position, self.profile.baseline_target_tick)
                <= STATIC_TOLERANCE_TICKS
                && samples.len() >= BASELINE_MIN_SAMPLES
            {
                break;
            }
        }
        if samples.len() < BASELINE_MIN_SAMPLES {
            return Err(format!(
                "insufficient moving baseline samples: {} < {}",
                samples.len(),
                BASELINE_MIN_SAMPLES
            )
            .into());
        }
        let baseline = BaselineStats::from_samples(&samples)
            .map_err(|message| -> DynError { message.into() })?;
        self.move_motor_to(motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
            .await?;
        Ok(baseline)
    }

    async fn approach(
        &mut self,
        step_ticks: u16,
        baseline: BaselineStats,
    ) -> Result<u16, DynError> {
        let motor_id = self.profile.motor_id;
        let start = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, start, true, None)?;
        let mut detector =
            HybridContactDetector::new(start.position, baseline, self.profile.probe_sign);
        let mut target = start.position;
        let mut last_stamp = start.monotonic_stamp_ns;

        loop {
            self.check_stop()?;
            let next_target = advance_tick(target, self.profile.probe_sign, step_ticks)?;
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
            target = next_target;
            let settle_deadline = Instant::now() + CONTACT_SETTLE_WINDOW;
            let mut last_observation = None;

            while Instant::now() < settle_deadline {
                let observation = self
                    .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                    .await?;
                last_stamp = observation.monotonic_stamp_ns;
                last_observation = Some(observation);
                self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
                self.verify_static_holds().await?;
                if circular_distance(observation.position, target) <= STATIC_TOLERANCE_TICKS {
                    break;
                }
                match detector.observe(observation, target) {
                    ContactState::FreeMotion | ContactState::ContactSuspected => {}
                    ContactState::ContactConfirmed => {
                        info!(
                            "MATDOG {} contact: step={}, target={}, present={}, error={}, current={}, threshold={}, velocity={}",
                            self.profile.label,
                            step_ticks,
                            target,
                            observation.position,
                            circular_distance(observation.position, target),
                            observation.current,
                            baseline.contact_threshold(),
                            speed_magnitude(observation.velocity)
                        );
                        self.stop_pressure(motor_id, observation.position).await?;
                        return Ok(observation.position);
                    }
                    ContactState::HardAbort => {
                        return self.abort_with_global_torque_off(format!(
                            "{} hard abort: tick={}, goal={}, current={}, torque_enabled={}, torque_limit={}, status=0x{:02X}, driver_error={}",
                            self.profile.label,
                            observation.position,
                            observation.goal_position,
                            observation.current,
                            observation.torque_enabled,
                            observation.torque_limit,
                            observation.status,
                            observation.has_driver_error
                        )).await;
                    }
                }
            }

            let observation =
                last_observation.ok_or("contact settle window produced no telemetry")?;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            let goal_error = circular_distance(observation.position, target);
            if goal_error > step_ticks.saturating_add(4) {
                self.stop_pressure(motor_id, observation.position).await?;
                return Err(format!(
                    "{} tracking failed without confirmed contact: target={}, present={}, current={}",
                    self.profile.label, target, observation.position, observation.current
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
        let target = advance_tick(contact_tick, -self.profile.probe_sign, BACKOFF_TICKS)?;
        if crossed_home(target, self.profile.probe_sign) {
            return Err(format!("{} backoff crosses home: {target}", self.profile.label).into());
        }
        let recovered = self
            .move_motor_to(
                self.profile.motor_id,
                target,
                STATIC_TOLERANCE_TICKS.saturating_add(2),
            )
            .await?;
        if recovered.current > baseline.contact_threshold() {
            return Err(format!(
                "{} current did not recover after backoff: {} > {}",
                self.profile.label,
                recovered.current,
                baseline.contact_threshold()
            )
            .into());
        }
        Ok(())
    }

    async fn abort_with_global_torque_off<T>(&mut self, message: String) -> Result<T, DynError> {
        match self.global_torque_off_verified().await {
            Ok(()) => Err(message.into()),
            Err(cleanup_err) => Err(format!(
                "{message}; immediate verified global torque-OFF also failed: {cleanup_err}"
            )
            .into()),
        }
    }

    async fn stop_pressure(&mut self, motor_id: u8, present_position: u16) -> Result<(), DynError> {
        self.set_motor_goal_verified(motor_id, present_position)
            .await
    }

    async fn move_motor_to(
        &mut self,
        motor_id: u8,
        target: u16,
        tolerance: u16,
    ) -> Result<MotorObservation, DynError> {
        self.set_motor_goal_verified(motor_id, target).await?;
        let mut last_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let deadline = Instant::now() + MOTION_TIMEOUT;
        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            self.verify_static_holds_except(motor_id).await?;
            if circular_distance(observation.position, target) <= tolerance {
                return Ok(observation);
            }
        }
        let last = self.latest_observation(motor_id)?;
        Err(format!(
            "M{motor_id} target timeout: target={target}, present={}, error={}",
            last.position,
            circular_distance(last.position, target)
        )
        .into())
    }

    async fn verify_static_holds(&self) -> Result<(), DynError> {
        self.verify_static_holds_except(0).await
    }

    async fn verify_static_holds_except(&self, ignored_motor: u8) -> Result<(), DynError> {
        for motor_id in MATDOG_MOTOR_IDS {
            if motor_id == ignored_motor {
                continue;
            }

            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;

            if let Some(target) = self
                .held_targets
                .iter()
                .find(|target| target.motor_id == motor_id)
            {
                self.ensure_observation_safe(
                    motor_id,
                    observation,
                    true,
                    Some(target.target_tick),
                )?;
                if circular_distance(observation.position, target.target_tick)
                    > STATIC_TOLERANCE_TICKS
                {
                    return Err(format!(
                        "static prerequisite M{motor_id} drifted: target={}, present={}, tolerance={}",
                        target.target_tick,
                        observation.position,
                        STATIC_TOLERANCE_TICKS
                    )
                    .into());
                }
            } else {
                if observation.torque_enabled {
                    return Err(
                        format!("non-active M{motor_id} unexpectedly torque-enabled").into(),
                    );
                }
                if observation.has_driver_error || observation.status != 0 {
                    return Err(format!("non-active M{motor_id} became unhealthy").into());
                }
                if circular_distance(observation.position, HOME_TICK) > STATIC_TOLERANCE_TICKS {
                    return Err(format!(
                        "non-active M{motor_id} left home: present={}, expected={}, tolerance={}",
                        observation.position, HOME_TICK, STATIC_TOLERANCE_TICKS
                    )
                    .into());
                }
            }
        }
        Ok(())
    }

    async fn set_motor_goal_verified(&mut self, motor_id: u8, target: u16) -> Result<(), DynError> {
        if target > protocol::MAX_ANGLE_STEP {
            return Err(format!("unsigned GoalPosition out of range: {target}").into());
        }
        self.write_motor_ram_verified(
            motor_id,
            RamRegister::GoalPosition,
            target.to_le_bytes().to_vec(),
        )
        .await
    }

    async fn set_motor_torque_verified(
        &mut self,
        motor_id: u8,
        enabled: bool,
    ) -> Result<(), DynError> {
        self.write_motor_ram_verified(motor_id, RamRegister::TorqueEnable, vec![u8::from(enabled)])
            .await?;
        let observation = self.latest_observation(motor_id)?;
        if observation.torque_enabled != enabled {
            return Err(format!(
                "M{motor_id} torque readback mismatch: expected={enabled}, observed={}",
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
            let observation =
                observation_from_state(&self.current_state(), &self.target_bus_serial, motor_id)?;
            if observation.torque_enabled {
                return Err(format!("M{motor_id} remained torque-enabled after global OFF").into());
            }
        }
        self.held_targets.clear();
        Ok(())
    }

    async fn write_motor_ram_verified(
        &mut self,
        motor_id: u8,
        register: RamRegister,
        value: Vec<u8>,
    ) -> Result<(), DynError> {
        validate_ram_write(register, &value)?;
        if !self.profile.allowed_motor_ids.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside armed profile motor allowlist").into());
        }
        let initial_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let command_id = self.next_command_id();
        let envelope = TxEnvelope {
            monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
            local_stamp_ns: systime::get_local_stamp_ns(),
            app_start_id: systime::get_app_start_id(),
            target_bus_serial: self.target_bus_serial.clone(),
            command_id: command_id.clone(),
            write: Some(crate::st3215_proto::St3215WriteCommand {
                motor_id: motor_id as u32,
                address: register.address() as u32,
                value: value.clone().into(),
            }),
            ..Default::default()
        };
        self.comm.send_tx(&envelope)?;
        self.wait_for_command_result(&command_id).await?;
        self.wait_for_register_value(motor_id, register, &value, initial_stamp)
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
            if !MATDOG_MOTOR_IDS.contains(motor_id) || !unique_motor_ids.insert(*motor_id) {
                return Err(format!("invalid MATDOG sync-write motor ID: {motor_id}").into());
            }
        }
        let initial_stamps: Vec<(u8, u64)> = writes
            .iter()
            .map(|(motor_id, _)| {
                observation_from_state(&self.current_state(), &self.target_bus_serial, *motor_id)
                    .map(|obs| (*motor_id, obs.monotonic_stamp_ns))
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
                    Ok(CommandResult::CrRejected) => return Err("ST3215 command rejected".into()),
                    Ok(CommandResult::CrFailed) => return Err("ST3215 command failed".into()),
                    Ok(CommandResult::CrProcessing) => {}
                    Err(_) => return Err(format!("invalid ST3215 command result: {result}").into()),
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() { return Err("ST3215 inference channel closed".into()); }
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
                    if changed.is_err() { return Err("ST3215 inference channel closed".into()); }
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

    async fn wait_for_motor_observation_after(
        &mut self,
        motor_id: u8,
        minimum_stamp: u64,
        timeout: Duration,
    ) -> Result<MotorObservation, DynError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Ok(observation) =
                observation_from_state(&self.current_state(), &self.target_bus_serial, motor_id)
            {
                if observation.monotonic_stamp_ns > minimum_stamp {
                    return Ok(observation);
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() { return Err("ST3215 inference channel closed".into()); }
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

    fn ensure_observation_fresh(
        &self,
        motor_id: u8,
        observation: MotorObservation,
    ) -> Result<(), DynError> {
        let now = systime::get_monotonic_stamp_ns();
        let max_age_ns = u64::try_from(MAX_TELEMETRY_AGE.as_nanos()).unwrap_or(u64::MAX);
        let age_ns = now.saturating_sub(observation.monotonic_stamp_ns);
        if observation.monotonic_stamp_ns == 0 || age_ns > max_age_ns {
            return Err(format!(
                "M{motor_id} telemetry stale: age_ns={age_ns}, max_age_ns={max_age_ns}"
            )
            .into());
        }
        Ok(())
    }

    fn ensure_observation_safe(
        &self,
        motor_id: u8,
        observation: MotorObservation,
        require_torque: bool,
        expected_goal: Option<u16>,
    ) -> Result<(), DynError> {
        self.ensure_observation_fresh(motor_id, observation)?;
        if observation.has_driver_error {
            return Err(format!("M{motor_id} driver error present").into());
        }
        if observation.status != 0 {
            return Err(format!("M{motor_id} servo status is 0x{:02X}", observation.status).into());
        }
        if require_torque && !observation.torque_enabled {
            return Err(format!("M{motor_id} torque unexpectedly disabled").into());
        }
        if require_torque && observation.torque_limit != TORQUE_LIMIT {
            return Err(format!(
                "M{motor_id} torque-limit changed: expected={}, observed={}",
                TORQUE_LIMIT, observation.torque_limit
            )
            .into());
        }
        if let Some(expected_goal) = expected_goal {
            if observation.goal_position != expected_goal {
                return Err(format!(
                    "M{motor_id} goal changed: expected={expected_goal}, observed={}",
                    observation.goal_position
                )
                .into());
            }
        }
        if observation.current >= HARD_CURRENT_ABORT_RAW {
            return Err(format!(
                "M{motor_id} hard current abort: {} >= {}",
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
        self.publish_progress(
            self.current_step,
            phase,
            CalibrationStatus::InProgress,
            None,
        );
        Ok(())
    }

    fn publish_progress(
        &self,
        current: u32,
        phase: &str,
        status: CalibrationStatus,
        error: Option<&str>,
    ) {
        self.comm.update_calibration_progress(
            &self.target_bus_serial,
            current,
            self.total_steps,
            &format!("{}: {phase}", self.profile.label),
            status,
            error,
        );
    }

    fn mark_done(&self) {
        self.publish_progress(self.total_steps, "completed", CalibrationStatus::Done, None);
    }

    fn mark_failed(&self, message: &str) {
        self.publish_progress(
            self.current_step,
            "failed",
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
        .map(|motor| {
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
        torque_limit: u16::from_le_bytes([bytes[torque_limit_addr], bytes[torque_limit_addr + 1]]),
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
        (&command.command_id == command_id).then_some(last.result)
    })
}

fn median(values: &[u16]) -> u16 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

fn repeatability_spread(first_tick: u16, second_tick: u16) -> Result<u16, DynError> {
    let spread = circular_distance(first_tick, second_tick);
    if spread > REPEATABILITY_TOLERANCE_TICKS {
        Err(format!(
            "contact not repeatable: first={first_tick}, second={second_tick}, spread={spread}"
        )
        .into())
    } else {
        Ok(spread)
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
    ((value as i32 - reference as i32 + TICKS_PER_REVOLUTION / 2).rem_euclid(TICKS_PER_REVOLUTION)
        - TICKS_PER_REVOLUTION / 2) as i16
}

fn circular_distance(a: u16, b: u16) -> u16 {
    signed_tick_delta(a, b).unsigned_abs()
}

fn directional_progress(value: u16, reference: u16, sign: i8) -> u16 {
    (i32::from(signed_tick_delta(value, reference)) * i32::from(sign)).max(0) as u16
}

fn advance_tick(value: u16, sign: i8, amount: u16) -> Result<u16, DynError> {
    let next = i32::from(value) + i32::from(sign) * i32::from(amount);
    u16::try_from(next)
        .ok()
        .filter(|tick| *tick <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| format!("unsigned GoalPosition out of range: {next}").into())
}

fn passed_guard(value: u16, guard: u16, sign: i8) -> bool {
    if sign < 0 {
        value < guard
    } else {
        value > guard
    }
}

fn crossed_home(value: u16, probe_sign: i8) -> bool {
    if probe_sign < 0 {
        value > HOME_TICK
    } else {
        value < HOME_TICK
    }
}

fn speed_magnitude(raw: u16) -> u16 {
    raw & 0x7FFF
}

#[cfg(test)]
#[path = "matdog_test.rs"]
mod tests;
