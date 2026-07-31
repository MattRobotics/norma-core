#!/usr/bin/env bash
set -Eeuo pipefail

BASE=32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
SOURCE=software/drivers/st3215/src/auto_calibrate/matdog.rs
TESTS=software/drivers/st3215/src/auto_calibrate/matdog_test.rs
PORT=software/drivers/st3215/src/port.rs
OUT=ci-output
TOOLS=ci-tools
VIEWER=software/station/clients/station-viewer/dist/index.html

export CARGO_TERM_COLOR=never
mkdir -p "$OUT" "$TOOLS"

cleanup() {
  rm -rf software/station/clients/station-viewer/dist
}
trap cleanup EXIT

section() {
  printf '\n===== %s =====\n' "$1"
}

section "scope"
git merge-base --is-ancestor "$BASE" HEAD
actual="$(git diff --name-only "$BASE"...HEAD | sort)"
expected=$'.github/workflows/matdog-restart-safe-v11.yml\ntools/matdog/apply_motion_timeout_v13.py\ntools/matdog/apply_ordered_sequence_v20.py\ntools/matdog/apply_probe_home_tolerance_v18.py\ntools/matdog/clean_ordered_sequence_v20_warnings.py\ntools/matdog/fix_ordered_sequence_v20_quote.py\ntools/matdog/restart_safe_v11_bundle.part00.b64\ntools/matdog/restart_safe_v11_bundle.part01.b64\ntools/matdog/restart_safe_v11_bundle.part02.b64\ntools/matdog/restart_safe_v11_bundle.part03.b64\ntools/matdog/run_ordered_v20_ci.sh'
printf 'head=%s\nchanged_files:\n%s\n' "$(git rev-parse HEAD)" "$actual" | tee "$OUT/scope.log"
test "$actual" = "$expected"

section "decode immutable V11 bundle"
python3 - <<'PY'
from pathlib import Path
fixes = (
    (Path('tools/matdog/restart_safe_v11_bundle.part00.b64'), 3068, 'U', 'c'),
    (Path('tools/matdog/restart_safe_v11_bundle.part02.b64'), 3243, 'a', 'i'),
)
for path, offset, observed, expected in fixes:
    text = ''.join(path.read_text().split())
    assert text[offset] == observed, (path, offset, text[offset], observed)
    path.write_text(text[:offset] + expected + text[offset + 1:])
PY
cat tools/matdog/restart_safe_v11_bundle.part*.b64 | base64 -d \
  > "$OUT/restart_safe_v11_bundle.tar.gz"
echo "dc424847dcc8c7e7ccf39c9b607b3d58d3efa3555996de67a06a2c3441b63118  $OUT/restart_safe_v11_bundle.tar.gz" \
  | sha256sum -c -
tar -xzf "$OUT/restart_safe_v11_bundle.tar.gz" -C "$TOOLS"
python3 -m py_compile \
  "$TOOLS/apply_restart_safe_v11.py" \
  "$TOOLS/audit_restart_safe_v11.py" \
  "$TOOLS/replay_restart_safe_v11.py" \
  tools/matdog/apply_motion_timeout_v13.py \
  tools/matdog/apply_probe_home_tolerance_v18.py \
  tools/matdog/apply_ordered_sequence_v20.py \
  tools/matdog/fix_ordered_sequence_v20_quote.py \
  tools/matdog/clean_ordered_sequence_v20_warnings.py

section "apply V11 V13 V18 V20"
python3 "$TOOLS/apply_restart_safe_v11.py" "$SOURCE" "$TESTS" "$PORT" \
  2>&1 | tee "$OUT/apply-v11.log"
python3 tools/matdog/apply_motion_timeout_v13.py "$SOURCE" "$TESTS" \
  2>&1 | tee "$OUT/apply-v13.log"
python3 tools/matdog/apply_probe_home_tolerance_v18.py "$SOURCE" "$TESTS" \
  2>&1 | tee "$OUT/apply-v18.log"
python3 tools/matdog/apply_ordered_sequence_v20.py "$SOURCE" "$TESTS" \
  2>&1 | tee "$OUT/apply-v20.log"
python3 tools/matdog/fix_ordered_sequence_v20_quote.py "$SOURCE" \
  2>&1 | tee "$OUT/fix-v20-quote.log"
python3 tools/matdog/clean_ordered_sequence_v20_warnings.py "$SOURCE" \
  2>&1 | tee "$OUT/clean-v20-warnings.log"

section "audit and format"
python3 "$TOOLS/audit_restart_safe_v11.py" "$SOURCE" "$TESTS" "$PORT" \
  | tee "$OUT/restart-safe-audit.log"
rustfmt --edition 2021 "$SOURCE" "$TESTS" "$PORT"
git diff --check
software_changed="$(git diff --name-only | grep '^software/' | sort)"
software_expected=$'software/drivers/st3215/src/auto_calibrate/matdog.rs\nsoftware/drivers/st3215/src/auto_calibrate/matdog_test.rs\nsoftware/drivers/st3215/src/port.rs'
test "$software_changed" = "$software_expected"

python3 - <<'PY' | tee "$OUT/ordered-contract.log"
from pathlib import Path
source = Path('software/drivers/st3215/src/auto_calibrate/matdog.rs').read_text()
tests = Path('software/drivers/st3215/src/auto_calibrate/matdog_test.rs').read_text()
forbidden = (
    'EepromRegister', 'RamRegister::Lock', 'reg_write: Some',
    'reset: Some', 'reset_calibration: Some', 'freeze_calibration: Some',
    'action: Some', 'Offset.address', 'const UPPER_50_DELTA',
)
required = (
    'Inspect restart-safe profile entry', 'motion_timeout_for_distance',
    'const PROBE_HOME_TOLERANCE_TICKS: u16 = 16;',
    'for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip]',
    'const UPPER_85_DELTA: i16 = 967;',
    'const LOWER_FOLDED_DELTA: i16 = -990;',
    '#[cfg(test)]\nfn position_inside_contact_acceptance',
    '#[cfg(test)]\n    fn new(start_position: u16',
    'contact_acceptance_bounds', 'ContactState::EarlyStall',
    'early stall outside model contact corridor',
    'HIP_HARDWARE_BLOCK_REASON', 'global_torque_off_verified',
)
required_tests = (
    'ordered_profile_table_lists_upper_then_lower_then_hip',
    'lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers',
    'hip_prerequisites_are_compact_and_side_specific',
    'isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed',
    'v19_m13_2405_is_early_stall_not_contact',
)
found = [x for x in forbidden if x in source]
missing = [x for x in required if x not in source]
missing_tests = [x for x in required_tests if x not in tests]
assert source.count('PROBE_HOME_TOLERANCE_TICKS') == 4
assert not found and not missing and not missing_tests, (found, missing, missing_tests)
print('ordered RAM-only zero-warning contract: PASS')
PY

section "targeted regressions"
TARGETED="$OUT/targeted-v20.log"
: > "$TARGETED"
for filter in \
  ordered_profile_table_ \
  lf_lower_profiles_ \
  hip_prerequisites_ \
  isolated_hip_ \
  contact_acceptance_ \
  v19_m13_ \
  detector_confirms_only_ \
  startup_ \
  motion_timeout_ \
  probe_home_tolerance_; do
  cargo test --package st3215 "$filter" -- --nocapture 2>&1 | tee -a "$TARGETED"
done
for token in \
  'ordered_profile_table_lists_upper_then_lower_then_hip ... ok' \
  'lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers ... ok' \
  'hip_prerequisites_are_compact_and_side_specific ... ok' \
  'isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed ... ok' \
  'contact_acceptance_corridors_match_model_inner_boundary_and_guard ... ok' \
  'v19_m13_2405_is_early_stall_not_contact ... ok' \
  'detector_confirms_only_persistent_stall_inside_profile_corridor ... ok'; do
  grep -q "$token" "$TARGETED"
done
! grep -q 'warning:' "$TARGETED"

section "full ST3215 suite"
cargo test --package st3215 2>&1 | tee "$OUT/st3215-full-tests.log"
grep -q '85 passed; 0 failed' "$OUT/st3215-full-tests.log"
! grep -q 'warning:' "$OUT/st3215-full-tests.log"

section "restart replay"
python3 "$TOOLS/replay_restart_safe_v11.py" | tee "$OUT/restart-safe-replay.log"

section "Station release build"
test ! -e "$VIEWER"
mkdir -p "$(dirname "$VIEWER")"
printf '<!doctype html><title>MATDOG ordered V20 build gate</title>\n' > "$VIEWER"
cargo build --release --package station 2>&1 | tee "$OUT/station-build.log"
! grep -q 'warning:' "$OUT/station-build.log"
strings target/release/station | grep -F 'Inspect restart-safe profile entry'
strings target/release/station | grep -F 'early stall outside model contact corridor'
strings target/release/station | grep -F \
  'HIP hardware is blocked until ordered UPPER MIN/MAX and LOWER MIN/MAX phase proof is verified'

section "PASS"
echo 'result=PASS'
echo 'tests=85'
echo 'warnings=0'
echo 'hardware_started=false'
echo 'serial_opened=false'
