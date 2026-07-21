/// Auto-calibration for ST3215 motors
///
/// Calibrates motor physical range by sweeping to limits and detecting stalls.
/// Handles register boundary conditions by adjusting the offset (midpoint) value.

use crate::port::{MAX_MOTORS_CNT, ST3215_COMMAND_TIMEOUT_MS};
use crate::protocol::{self};
use bytes::Bytes;
use std::sync::atomic::Ordering;

pub mod calibrator;
mod elrobot;
mod matdog;
mod so101;

/// Auto-calibration scan ceiling.
///
/// Keep `MAX_MOTORS_CNT` at 8 for the existing SO101/ElRobot calibration
/// presets, while scanning the sparse MATDOG ID map through M43.
const AUTO_CALIBRATE_MAX_MOTOR_ID: u8 = 43;

/// Main auto-calibration entry point.
///
/// Detects robot type and applies the matching robot-specific strategy.
pub async fn calibrate(
    port: &mut tokio_serial::SerialStream,
    bus_info: &crate::st3215_proto::St3215Bus,
    meta: &crate::port_meta::St3215PortMeta,
) -> Result<Option<std::sync::Arc<std::sync::atomic::AtomicBool>>, protocol::Error> {
    log::info!(
        "Starting auto-calibration sequence for bus {}",
        bus_info.serial_number
    );

    let mut found_motors = Vec::new();
    for motor_id in 1..=AUTO_CALIBRATE_MAX_MOTOR_ID {
        let ping_req = protocol::ST3215Request::Ping { motor: motor_id };
        if ping_req
            .async_readwrite(port, ST3215_COMMAND_TIMEOUT_MS)
            .await
            .is_ok()
        {
            found_motors.push(motor_id);
        }
    }

    if found_motors.is_empty() {
        log::warn!("No motors found on bus");
        return Ok(None);
    }

    log::info!("Found {} motor(s): {:?}", found_motors.len(), found_motors);

    let is_matdog_12 = matdog::is_exact_matdog_motor_set(&found_motors);
    let is_so101_6 = found_motors.len() == 6
        && found_motors.iter().all(|&id| id >= 1 && id <= 6);
    let is_elrobot_8 = found_motors.len() == MAX_MOTORS_CNT as usize
        && found_motors
            .iter()
            .all(|&id| id >= 1 && id <= MAX_MOTORS_CNT);

    let comm = meta.get_communicator().clone();
    if let Some(existing_stop_flag) = comm.get_calibration_stop(&bus_info.serial_number) {
        log::info!(
            "Stopping existing calibration for bus {}",
            bus_info.serial_number
        );
        existing_stop_flag.store(true, Ordering::Relaxed);
    }

    let stop_flag = if is_matdog_12 {
        matdog::auto_calibrate(
            bus_info.serial_number.clone(),
            found_motors,
            comm.clone(),
        )
        .await
        .map_err(|e| protocol::Error::InvalidData {
            msg: format!("MATDOG auto-calibration failed: {}", e),
            source_packet: Bytes::new(),
            reply_packet: Bytes::new(),
        })?
    } else if is_so101_6 {
        so101::auto_calibrate(bus_info.serial_number.clone(), comm.clone())
            .await
            .map_err(|e| protocol::Error::InvalidData {
                msg: format!("Auto-calibration failed: {}", e),
                source_packet: Bytes::new(),
                reply_packet: Bytes::new(),
            })?
    } else if is_elrobot_8 {
        elrobot::auto_calibrate(
            bus_info.serial_number.clone(),
            found_motors,
            comm.clone(),
        )
        .await
        .map_err(|e| protocol::Error::InvalidData {
            msg: format!("Auto-calibration failed: {}", e),
            source_packet: Bytes::new(),
            reply_packet: Bytes::new(),
        })?
    } else {
        return Ok(None);
    };

    comm.set_calibration_stop(&bus_info.serial_number, stop_flag.clone());
    Ok(Some(stop_flag))
}
