#!/usr/bin/env bash
set -Eeuo pipefail

BRANCH="matdog/native-calibrator-24-contact-profiles"
SOURCE_RUN_ID="30165101583"
LOG_DIR="${RUNNER_TEMP:-/tmp}/matdog-publish-source-logs"
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"
unset MATDOG_NATIVE_CALIBRATOR_ARM || true

python3 - <<'PY'
from pathlib import Path
import hashlib

data = Path("tools/matdog_finalize.patch.gz.b64").read_bytes()
broken = "df382f460699d572130421b75a4e51dafe160683a95fac9d7fcc162eb81f3774"
fixed = "c368ec44a30fbe14d4e517eac8e2441cc0a85f1410f570553e8ace5138f5471e"
actual = hashlib.sha256(data).hexdigest()
if len(data) == 8495 and actual == broken:
    data = data[:8017] + b"B" + data[8017:]
elif len(data) != 8496 or actual != fixed:
    raise SystemExit(f"unexpected transport: length={len(data)} sha256={actual}")
if hashlib.sha256(data).hexdigest() != fixed:
    raise SystemExit("repaired transport hash mismatch")
Path("/tmp/matdog_finalize.patch.gz.b64").write_bytes(data)
PY
base64 --decode /tmp/matdog_finalize.patch.gz.b64 | gzip -d > /tmp/matdog_finalize.patch
echo "36f51616160da7065fa6f3390fdcae314cc4f91f835845f14342e4f8c882df2b  /tmp/matdog_finalize.patch" | sha256sum -c - | tee "$LOG_DIR/patch.log"

rm -rf /tmp/matdog-source
mkdir -p /tmp/matdog-source
gh run download "$SOURCE_RUN_ID" \
  --repo "${GITHUB_REPOSITORY:-MattRobotics/norma-core}" \
  --name matdog-source-diagnostic \
  --dir /tmp/matdog-source
cp /tmp/matdog-source/generated/matdog_v2.rs software/drivers/st3215/src/auto_calibrate/matdog_v2.rs
cp /tmp/matdog-source/generated/matdog_v2_test.rs software/drivers/st3215/src/auto_calibrate/matdog_v2_test.rs
cp /tmp/matdog-source/generated/mod.rs software/drivers/st3215/src/auto_calibrate/mod.rs
cp /tmp/matdog-source/generated/port.rs software/drivers/st3215/src/port.rs
git apply --check /tmp/matdog_finalize.patch
git apply /tmp/matdog_finalize.patch
python3 tools/matdog_post_finalize.py | tee "$LOG_DIR/post-finalize.log"
mv -f software/drivers/st3215/src/auto_calibrate/matdog_v2.rs software/drivers/st3215/src/auto_calibrate/matdog.rs
mv -f software/drivers/st3215/src/auto_calibrate/matdog_v2_test.rs software/drivers/st3215/src/auto_calibrate/matdog_test.rs

rustfmt --edition 2021 --config skip_children=true \
  software/drivers/st3215/src/port.rs \
  software/drivers/st3215/src/auto_calibrate/mod.rs \
  software/drivers/st3215/src/auto_calibrate/matdog.rs \
  software/drivers/st3215/src/auto_calibrate/matdog_test.rs \
  2>&1 | tee "$LOG_DIR/rustfmt.log"
git diff --check | tee "$LOG_DIR/diff-check.log"

python3 - <<'PY' | tee "$LOG_DIR/contract.log"
from pathlib import Path
source = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs").read_text()
tests = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs").read_text()
port = Path("software/drivers/st3215/src/port.rs").read_text()
module = Path("software/drivers/st3215/src/auto_calibrate/mod.rs").read_text()
forbidden_source = (
    "EepromRegister", "RamRegister::Lock", "ST3215Request::",
    "reg_write: Some", "reset: Some", "reset_calibration: Some",
    "freeze_calibration: Some", "action: Some", "Offset.address", "matdog_v2",
)
forbidden_other = ("write.motor_id != 12", "matdog_pilot_is_armed", "matdog_armed_motor_ids")
required_source = (
    "all_profiles", "profile_for_arm_value", "ram_write_allowed_for_profile",
    "held_targets: Vec<StaticTarget>", "verify_static_holds_except",
    "MAX_TELEMETRY_AGE", "prerequisite_restore_order",
    "global_torque_off_verified", "GUARD_OVERSHOOT_TICKS",
    "UPPER_30_DELTA", "UPPER_50_DELTA", "UPPER_90_DELTA",
)
required_tests = (
    "profile_table_covers_exactly_24_unique_contacts",
    "front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home",
    "armed_ram_gate_restricts_registers_values_motors_and_goal_windows",
    "canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path",
)
found = [token for token in forbidden_source if token in source]
found += [token for token in forbidden_other if token in port or token in module]
missing = [token for token in required_source if token not in source]
missing += [token for token in required_tests if token not in tests]
if "mod matdog;" not in module or "matdog_ram_write_allowed_for_arm_value" not in module:
    missing.append("canonical matdog module exports")
if found or missing:
    raise SystemExit(f"forbidden={found}, missing={missing}")
print("MATDOG canonical 24-contact RAM-only contract: PASS")
PY

cargo test --package st3215 2>&1 | tee "$LOG_DIR/st3215-tests.log"

rm -rf tools/.matdog-v2-payload
rm -f tools/matdog_finalize.patch.gz.b64
rm -f tools/matdog-native-calibrator-check.final.yml
rm -f tools/matdog_post_finalize.py
rm -f tools/matdog_finalize_and_verify.sh
rm -f tools/matdog_publish_verified_source.sh

git diff --check | tee -a "$LOG_DIR/diff-check.log"
git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git add \
  software/drivers/st3215/src/port.rs \
  software/drivers/st3215/src/auto_calibrate/mod.rs \
  software/drivers/st3215/src/auto_calibrate/matdog.rs \
  software/drivers/st3215/src/auto_calibrate/matdog_test.rs \
  tools/
git commit -m "feat(st3215): generalize MATDOG calibrator profiles"
git push origin "HEAD:$BRANCH"
printf 'published_commit=%s\n' "$(git rev-parse HEAD)" | tee "$LOG_DIR/published.log"
