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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CalibrationKind {
    Matdog,
    So101,
    Elrobot,
    Unsupported,
}

fn calibration_kind(found_motors: &[u8]) -> CalibrationKind {
    if matdog::is_exact_matdog_motor_set(found_motors) {
        CalibrationKind::Matdog
    } else if found_motors.len() == 6 && found_motors.iter().all(|&id| (1..=6).contains(&id)) {
        CalibrationKind::So101
    } else if found_motors.len() == MAX_MOTORS_CNT as usize
        && found_motors
            .iter()
            .all(|&id| (1..=MAX_MOTORS_CNT).contains(&id))
    {
        CalibrationKind::Elrobot
    } else {
        CalibrationKind::Unsupported
    }
}

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

    let kind = calibration_kind(&found_motors);
    if kind == CalibrationKind::Unsupported {
        return Ok(None);
    }

    let comm = meta.get_communicator().clone();
    if let Some(existing_stop_flag) = comm.get_calibration_stop(&bus_info.serial_number) {
        log::info!(
            "Stopping existing calibration for bus {}",
            bus_info.serial_number
        );
        existing_stop_flag.store(true, Ordering::Relaxed);
    }

    let stop_flag = match kind {
        CalibrationKind::Matdog => matdog::auto_calibrate(
            bus_info.serial_number.clone(),
            found_motors,
            comm.clone(),
        )
        .await
        .map_err(|e| protocol::Error::InvalidData {
            msg: format!("MATDOG auto-calibration failed: {}", e),
            source_packet: Bytes::new(),
            reply_packet: Bytes::new(),
        })?,
        CalibrationKind::So101 => so101::auto_calibrate(bus_info.serial_number.clone(), comm.clone())
            .await
            .map_err(|e| protocol::Error::InvalidData {
                msg: format!("Auto-calibration failed: {}", e),
                source_packet: Bytes::new(),
                reply_packet: Bytes::new(),
            })?,
        CalibrationKind::Elrobot => elrobot::auto_calibrate(
            bus_info.serial_number.clone(),
            found_motors,
            comm.clone(),
        )
        .await
        .map_err(|e| protocol::Error::InvalidData {
            msg: format!("Auto-calibration failed: {}", e),
            source_packet: Bytes::new(),
            reply_packet: Bytes::new(),
        })?,
        CalibrationKind::Unsupported => unreachable!("unsupported topology returned above"),
    };

    comm.set_calibration_stop(&bus_info.serial_number, stop_flag.clone());
    Ok(Some(stop_flag))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_motor_topologies_dispatch_without_overlap() {
        assert_eq!(
            calibration_kind(&matdog::MATDOG_MOTOR_IDS),
            CalibrationKind::Matdog
        );
        assert_eq!(calibration_kind(&[1, 2, 3, 4, 5, 6]), CalibrationKind::So101);
        assert_eq!(
            calibration_kind(&[1, 2, 3, 4, 5, 6, 7, 8]),
            CalibrationKind::Elrobot
        );
        assert_eq!(
            calibration_kind(&[11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 44]),
            CalibrationKind::Unsupported
        );
        assert_eq!(
            calibration_kind(&[11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 44]),
            CalibrationKind::Unsupported
        );
        assert_eq!(
            AUTO_CALIBRATE_MAX_MOTOR_ID,
            *matdog::MATDOG_MOTOR_IDS.last().unwrap()
        );
    }
}
