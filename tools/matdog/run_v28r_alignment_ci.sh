#!/usr/bin/env bash
set -Eeuo pipefail

BASE=32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
SOURCE=software/drivers/st3215/src/auto_calibrate/matdog.rs
TESTS=software/drivers/st3215/src/auto_calibrate/matdog_test.rs
PORT=software/drivers/st3215/src/port.rs
UI=software/station/clients/station-viewer/src/pages/St3215BusCalibrationPage.tsx
VIEWER=software/station/clients/station-viewer
PATCH=tools/matdog/v28r_alignment.patch
MARKER=tools/matdog/materialized_v28r_source.env
OUT=ci-output-v28r
TOOLS="$OUT/ci-tools"
export CARGO_TERM_COLOR=never
mkdir -p "$OUT" "$TOOLS"

section() { printf '\n===== %s =====\n' "$1"; }

apply_v20_chain() {
  python3 - <<'PY'
from base64 import b64decode
from pathlib import Path
parts=[]
for index in range(4):
    parts.append(''.join(Path(f'tools/matdog/restart_safe_v11_bundle.part{index:02d}.b64').read_text().split()))
text=''.join(parts)
fixes=((3068,'U','c'),(len(parts[0])+len(parts[1])+3243,'a','i'))
for offset, observed, expected in fixes:
    assert text[offset] == observed, (offset, text[offset], observed)
    text=text[:offset]+expected+text[offset+1:]
Path('/tmp/restart_safe_v11_bundle.tar.gz').write_bytes(b64decode(text))
PY
  echo 'dc424847dcc8c7e7ccf39c9b607b3d58d3efa3555996de67a06a2c3441b63118  /tmp/restart_safe_v11_bundle.tar.gz' | sha256sum -c -
  tar -xzf /tmp/restart_safe_v11_bundle.tar.gz -C "$TOOLS"
  python3 "$TOOLS/apply_restart_safe_v11.py" "$SOURCE" "$TESTS" "$PORT"
  python3 tools/matdog/apply_motion_timeout_v13.py "$SOURCE" "$TESTS"
  python3 tools/matdog/apply_probe_home_tolerance_v18.py "$SOURCE" "$TESTS"
  python3 tools/matdog/apply_ordered_sequence_v20.py "$SOURCE" "$TESTS"
  python3 tools/matdog/fix_ordered_sequence_v20_quote.py "$SOURCE"
  python3 tools/matdog/clean_ordered_sequence_v20_warnings.py "$SOURCE"
  rustfmt --edition 2021 "$SOURCE" "$TESTS" "$PORT"
  git apply --check "$PATCH"
  git apply "$PATCH"
  rustfmt --edition 2021 "$SOURCE" "$TESTS" "$PORT"
}

section "source mode"
git merge-base --is-ancestor "$BASE" HEAD
if [[ -f "$MARKER" ]]; then
  echo 'mode=materialized-source' | tee "$OUT/source-mode.log"
else
  echo 'mode=ci-materialization' | tee "$OUT/source-mode.log"
  apply_v20_chain 2>&1 | tee "$OUT/materialize.log"
fi

git diff --check

section "alignment contract"
python3 - <<'PY' | tee "$OUT/alignment-contract.log"
from pathlib import Path
source=Path('software/drivers/st3215/src/auto_calibrate/matdog.rs').read_text()
tests=Path('software/drivers/st3215/src/auto_calibrate/matdog_test.rs').read_text()
ui=Path('software/station/clients/station-viewer/src/pages/St3215BusCalibrationPage.tsx').read_text()
required_source=(
 'for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip]',
 'const STARTUP_PREREQUISITE_HOME_SETTLE_TICKS: u16 = 16;',
 'fn startup_prerequisite_bounds(target_tick: u16)',
 'fn home_hold_tolerance(',
 'probe_home_handoff_active: bool,',
 'self.probe_home_handoff_active = true;',
 'self.probe_home_handoff_active = false;',
 'const UPPER_85_DELTA: i16 = 967;',
 'const LOWER_FOLDED_DELTA: i16 = -990;',
 'HIP_HARDWARE_BLOCK_REASON',
 'global_torque_off_verified',
)
required_tests=(
 'startup_prerequisite_home_endpoint_accepts_observed_m42_2037_without_weakening_target',
 'probe_home_handoff_accepts_observed_m11_2062_only_for_probe',
 'front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home',
 'startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks',
 'canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path',
)
required_ui=(
 'MATDOG native mode: Auto Calibrate only. Reset and Save are disabled.',
 'if (!busSerial || !hasClassifiedMotorSet || isMatdogBus) return;',
 'disabled={!hasValidMotors || isMatdogBus}',
 'disabled={!hasValidMotors || isSavePending || isMatdogBus}',
 'const showMoveOverlay = !isMatdogBus',
)
forbidden=(
 'EepromRegister','RamRegister::Lock','reg_write: Some','reset: Some',
 'reset_calibration: Some','freeze_calibration: Some','action: Some','Offset.address',
)
missing=[x for x in required_source if x not in source]
missing += [x for x in required_tests if x not in tests]
missing += [x for x in required_ui if x not in ui]
found=[x for x in forbidden if x in source]
assert not missing and not found, (missing, found)
assert source.count('PROBE_HOME_TOLERANCE_TICKS') == 5
assert source.count('STARTUP_PREREQUISITE_HOME_SETTLE_TICKS') == 4
assert source.count('probe_home_handoff_active') == 7
print('V28R source/UI RAM-only contract: PASS')
PY

section "targeted regressions"
TARGETED="$OUT/targeted.log"
: > "$TARGETED"
for filter in \
  startup_prerequisite_home_endpoint_ \
  probe_home_handoff_ \
  probe_home_tolerance_ \
  front_lower_restore_order_ \
  startup_envelopes_match_ \
  ordered_profile_table_ \
  lf_lower_profiles_ \
  isolated_hip_ \
  canonical_matdog_source_; do
  cargo test --package st3215 "$filter" -- --nocapture 2>&1 | tee -a "$TARGETED"
done
for token in \
  'startup_prerequisite_home_endpoint_accepts_observed_m42_2037_without_weakening_target ... ok' \
  'probe_home_handoff_accepts_observed_m11_2062_only_for_probe ... ok' \
  'front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home ... ok' \
  'startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks ... ok'; do
  grep -q "$token" "$TARGETED"
done
! grep -q 'warning:' "$TARGETED"

section "full ST3215 suite"
cargo test --package st3215 2>&1 | tee "$OUT/st3215-full-tests.log"
grep -q '87 passed; 0 failed' "$OUT/st3215-full-tests.log"
! grep -q 'warning:' "$OUT/st3215-full-tests.log"

section "viewer build"
(
  cd "$VIEWER"
  npm ci
  npm run build
) 2>&1 | tee "$OUT/viewer-build.log"
test -f "$VIEWER/dist/index.html"
grep -R -Fq 'MATDOG native mode: Auto Calibrate only' "$VIEWER/dist/assets"

section "Station release build"
cargo build --release --package station 2>&1 | tee "$OUT/station-build.log"
! grep -q 'warning:' "$OUT/station-build.log"
strings target/release/station | grep -F 'Inspect restart-safe profile entry'
strings target/release/station | grep -F 'early stall outside model contact corridor'
strings target/release/station | grep -F 'HIP hardware is blocked until ordered UPPER MIN/MAX and LOWER MIN/MAX phase proof is verified'

section "PASS"
printf '%s\n' \
  'result=PASS' \
  'st3215_tests=87' \
  'rust_warnings=0' \
  'viewer_build=PASS' \
  'station_release_build=PASS' \
  'hardware_started=false' \
  'serial_opened=false' | tee "$OUT/SUMMARY.env"
