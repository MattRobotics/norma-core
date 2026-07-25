#!/usr/bin/env bash
set -Eeuo pipefail

BRANCH="matdog/native-calibrator-24-contact-profiles"
SOURCE_RUN_ID="30165101583"
PATCH_B64="tools/matdog_finalize.patch.gz.b64"
PATCH_SHA="36f51616160da7065fa6f3390fdcae314cc4f91f835845f14342e4f8c882df2b"
BROKEN_B64_SHA="df382f460699d572130421b75a4e51dafe160683a95fac9d7fcc162eb81f3774"
FIXED_B64_SHA="c368ec44a30fbe14d4e517eac8e2441cc0a85f1410f570553e8ace5138f5471e"
LOG_DIR="${RUNNER_TEMP:-/tmp}/matdog-ci-output"
VIEWER_DIR="software/station/clients/station-viewer/dist"

rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"
rm -rf "$VIEWER_DIR"
trap 'rm -rf "$VIEWER_DIR"' EXIT
unset MATDOG_NATIVE_CALIBRATOR_ARM || true

printf '=== MATDOG FINALIZE AND VERIFY ===\n' | tee "$LOG_DIR/summary.log"
printf 'branch=%s\nhead=%s\n' "$BRANCH" "$(git rev-parse HEAD)" | tee -a "$LOG_DIR/summary.log"

python3 - <<'PY'
from pathlib import Path
import hashlib

source = Path("tools/matdog_finalize.patch.gz.b64")
data = source.read_bytes()
broken = "df382f460699d572130421b75a4e51dafe160683a95fac9d7fcc162eb81f3774"
fixed = "c368ec44a30fbe14d4e517eac8e2441cc0a85f1410f570553e8ace5138f5471e"
actual = hashlib.sha256(data).hexdigest()
if len(data) == 8495 and actual == broken:
    data = data[:8017] + b"B" + data[8017:]
elif len(data) == 8496 and actual == fixed:
    pass
else:
    raise SystemExit(f"unexpected patch transport: length={len(data)} sha256={actual}")
if hashlib.sha256(data).hexdigest() != fixed:
    raise SystemExit("repaired patch transport hash mismatch")
Path("/tmp/matdog_finalize.patch.gz.b64").write_bytes(data)
print("patch transport: PASS")
PY

base64 --decode /tmp/matdog_finalize.patch.gz.b64 | gzip -d > /tmp/matdog_finalize.patch
echo "$PATCH_SHA  /tmp/matdog_finalize.patch" | sha256sum -c - | tee -a "$LOG_DIR/summary.log"

rm -rf /tmp/matdog-source
mkdir -p /tmp/matdog-source
gh run download "$SOURCE_RUN_ID" \
  --repo "${GITHUB_REPOSITORY:-MattRobotics/norma-core}" \
  --name matdog-source-diagnostic \
  --dir /tmp/matdog-source

for file in matdog_v2.rs matdog_v2_test.rs mod.rs port.rs; do
  test -f "/tmp/matdog-source/generated/$file"
done
cp /tmp/matdog-source/generated/matdog_v2.rs software/drivers/st3215/src/auto_calibrate/matdog_v2.rs
cp /tmp/matdog-source/generated/matdog_v2_test.rs software/drivers/st3215/src/auto_calibrate/matdog_v2_test.rs
cp /tmp/matdog-source/generated/mod.rs software/drivers/st3215/src/auto_calibrate/mod.rs
cp /tmp/matdog-source/generated/port.rs software/drivers/st3215/src/port.rs

git apply --check /tmp/matdog_finalize.patch
git apply /tmp/matdog_finalize.patch
python3 tools/matdog_post_finalize.py | tee -a "$LOG_DIR/summary.log"
mv -f software/drivers/st3215/src/auto_calibrate/matdog_v2.rs \
  software/drivers/st3215/src/auto_calibrate/matdog.rs
mv -f software/drivers/st3215/src/auto_calibrate/matdog_v2_test.rs \
  software/drivers/st3215/src/auto_calibrate/matdog_test.rs
test ! -e software/drivers/st3215/src/auto_calibrate/matdog_v2.rs
test ! -e software/drivers/st3215/src/auto_calibrate/matdog_v2_test.rs

rustfmt --edition 2021 --config skip_children=true \
  software/drivers/st3215/src/port.rs \
  software/drivers/st3215/src/auto_calibrate/mod.rs \
  software/drivers/st3215/src/auto_calibrate/matdog.rs \
  software/drivers/st3215/src/auto_calibrate/matdog_test.rs \
  2>&1 | tee "$LOG_DIR/rustfmt.log"
git diff --check | tee "$LOG_DIR/diff-check.log"

python3 - <<'PY' | tee "$LOG_DIR/ram-only-contract.log"
from pathlib import Path

source = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs").read_text()
tests = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs").read_text()
port = Path("software/drivers/st3215/src/port.rs").read_text()
module = Path("software/drivers/st3215/src/auto_calibrate/mod.rs").read_text()

forbidden_source = (
    "EepromRegister", "RamRegister::Lock", "ST3215Request::",
    "reg_write: Some", "reset: Some", "reset_calibration: Some",
    "freeze_calibration: Some", "action: Some", "Offset.address",
    "matdog_v2",
)
forbidden_port_or_module = (
    "write.motor_id != 12", "matdog_pilot_is_armed", "matdog_armed_motor_ids",
)
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
required_port = (
    "matdog_command_allowed_with", "matdog_armed_command_allowed",
    "native MATDOG profile arming is active",
)
required_module = ("mod matdog;", "matdog_ram_write_allowed_for_arm_value")

found = [token for token in forbidden_source if token in source]
found += [token for token in forbidden_port_or_module if token in port or token in module]
missing = [token for token in required_source if token not in source]
missing += [token for token in required_tests if token not in tests]
missing += [token for token in required_port if token not in port]
missing += [token for token in required_module if token not in module]
if found or missing:
    raise SystemExit(f"forbidden={found}, missing={missing}")
print("MATDOG canonical 24-contact RAM-only contract: PASS")
PY

set -o pipefail
cargo test --package st3215 2>&1 | tee "$LOG_DIR/st3215-tests.log"

mkdir -p "$VIEWER_DIR"
printf '<!doctype html><title>MATDOG offline build gate</title>\n' > "$VIEWER_DIR/index.html"
cargo build --release --package station 2>&1 | tee "$LOG_DIR/station-build.log"
rm -rf "$VIEWER_DIR"

cp tools/matdog-native-calibrator-check.final.yml \
  .github/workflows/matdog-native-calibrator-check.yml
rm -f .github/workflows/apply-matdog-v2-port-patch.yml
rm -f .github/workflows/matdog-test-diagnostic.yml
rm -f .github/workflows/matdog-final-test-diagnostic.yml
rm -f .github/workflows/matdog-finalize-v2.yml
rm -rf tools/.matdog-v2-payload
rm -f tools/matdog_finalize.patch.gz.b64
rm -f tools/matdog-native-calibrator-check.final.yml
rm -f tools/matdog_post_finalize.py
rm -f tools/matdog_finalize_and_verify.sh

git diff --check | tee -a "$LOG_DIR/diff-check.log"
git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git add -A
git commit -m "feat(st3215): generalize MATDOG calibrator profiles"
git push origin "HEAD:$BRANCH"

printf 'result=PASS\nverified_commit=%s\n' "$(git rev-parse HEAD)" | tee -a "$LOG_DIR/summary.log"
