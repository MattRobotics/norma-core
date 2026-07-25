#!/usr/bin/env python3
"""Deterministic post-finalize cleanup for the MATDOG 24-contact source."""

from pathlib import Path


port = Path("software/drivers/st3215/src/port.rs")
text = port.read_text(encoding="utf-8")
function_marker = (
    "fn matdog_runtime_gate_allows_only_armed_profile_ram_and_global_torque_off()"
)
function_start = text.find(function_marker)
if function_start < 0:
    raise SystemExit("MATDOG runtime-gate test function not found")
wrong_marker = "let wrong_motor = TxEnvelope"
wrong_start = text.find(wrong_marker, function_start)
if wrong_start < 0:
    raise SystemExit("wrong_motor test block not found")
motor_marker = "motor_id: 11"
motor_start = text.find(motor_marker, wrong_start)
if motor_start < 0:
    raise SystemExit("wrong_motor M11 marker not found")
function_end = text.find("\n    }\n}", motor_start)
if function_end < 0 or motor_start > function_end:
    raise SystemExit("wrong_motor marker lies outside expected test function")
text = text[:motor_start] + "motor_id: 21" + text[motor_start + len(motor_marker) :]
port.write_text(text, encoding="utf-8")

module = Path("software/drivers/st3215/src/auto_calibrate/mod.rs")
text = module.read_text(encoding="utf-8")
helper_start_marker = "pub(crate) fn matdog_armed_motor_ids()"
helper_end_marker = "pub(crate) fn matdog_armed_ram_write_allowed"
helper_start = text.find(helper_start_marker)
helper_end = text.find(helper_end_marker, helper_start)
if helper_start < 0 or helper_end < 0:
    raise SystemExit("unused MATDOG helper boundaries not found")
text = text[:helper_start] + text[helper_end:]
module.write_text(text, encoding="utf-8")

print("MATDOG post-finalize source cleanup: PASS")
