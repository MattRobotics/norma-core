#!/usr/bin/env bash
# One-command, fail-closed launcher for the supervised MATDOG RF mechanical
# end-stop measurement. Station remains the sole serial owner. This launcher
# never writes EEPROM and the native RF engine is RAM-only.
set -Eeuo pipefail

REPO="${MATDOG_NORMACORE_REPO:-/home/matteo-manicardi/norma-core}"
BRANCH="matdog/rf-native-calibrator"
BUS_SERIAL="5B14114953"
SERVER="127.0.0.1:8888"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
EVIDENCE_ROOT="${MATDOG_EVIDENCE_ROOT:-/home/matteo-manicardi/MATDOG/_archive}"
EVIDENCE_DIR="$EVIDENCE_ROOT/MATDOG_RF_MEASURE_${STAMP}"
STATION_LOG="$EVIDENCE_DIR/station.log"
RUNNER_DIR="$EVIDENCE_DIR/runner"
STATION_PID=""
RUNNER_STARTED=0

log() { printf '%s\n' "$*" | tee -a "$EVIDENCE_DIR/launcher.log"; }
fail() { log "BLOCKED  $*"; exit 2; }
pass() { log "PASS     $*"; }

mkdir -p "$EVIDENCE_DIR" "$RUNNER_DIR"

cleanup() {
    local rc=$?
    set +e
    if [[ -n "$STATION_PID" ]] && kill -0 "$STATION_PID" 2>/dev/null; then
        if [[ "$RUNNER_STARTED" -eq 1 ]]; then
            log "RECOVERY Station is still live after runner exit; sending SIGINT to verified PID $STATION_PID"
        else
            log "RECOVERY Station started but runner did not take control; sending SIGINT to PID $STATION_PID"
        fi
        kill -INT "$STATION_PID" 2>/dev/null
        for _ in $(seq 1 60); do
            kill -0 "$STATION_PID" 2>/dev/null || break
            sleep 0.25
        done
        if kill -0 "$STATION_PID" 2>/dev/null; then
            log "RECOVERY Station ignored SIGINT; sending SIGKILL to PID $STATION_PID"
            kill -KILL "$STATION_PID" 2>/dev/null
        fi
        wait "$STATION_PID" 2>/dev/null
    fi
    if (( rc != 0 )); then
        log "FINAL=BLOCKED exit_code=$rc evidence=$EVIDENCE_DIR"
    fi
}
trap cleanup EXIT INT TERM

log "MATDOG RF SUPERVISED HARDWARE TEST"
log "UTC=$STAMP"
log "REPOSITORY=$REPO"
log "EVIDENCE=$EVIDENCE_DIR"

[[ -d "$REPO/.git" ]] || fail "repository not found: $REPO"
cd "$REPO"

[[ "$(git branch --show-current)" == "$BRANCH" ]] || \
    fail "active branch must be $BRANCH"
[[ -z "$(git status --porcelain=v1 --untracked-files=all)" ]] || \
    fail "working tree is not clean"
[[ "$(git worktree list --porcelain | grep -c '^worktree ')" -eq 1 ]] || \
    fail "exactly one worktree is required"
[[ -z "$(git stash list)" ]] || fail "stash list must be empty"

log "Fetching exact reviewed RF branch"
git fetch origin "$BRANCH" --prune 2>&1 | tee -a "$EVIDENCE_DIR/git-fetch.log"
LOCAL_HEAD="$(git rev-parse HEAD)"
REMOTE_HEAD="$(git rev-parse "origin/$BRANCH")"
[[ "$LOCAL_HEAD" == "$REMOTE_HEAD" ]] || \
    fail "local HEAD $LOCAL_HEAD differs from origin/$BRANCH $REMOTE_HEAD; run git merge --ff-only origin/$BRANCH"
pass "source HEAD matches origin/$BRANCH: $LOCAL_HEAD"
printf '%s\n' "$LOCAL_HEAD" > "$EVIDENCE_DIR/source-head.txt"
git status --porcelain=v1 --untracked-files=all > "$EVIDENCE_DIR/git-status.txt"

if pgrep -af '(^|/)(station)( |$)' > "$EVIDENCE_DIR/preexisting-station.txt"; then
    cat "$EVIDENCE_DIR/preexisting-station.txt" | tee -a "$EVIDENCE_DIR/launcher.log"
    fail "Station is already running"
fi

serial_paths=()
shopt -s nullglob
for path in /dev/serial/by-id/* /dev/ttyACM* /dev/ttyUSB*; do
    serial_paths+=("$path")
done
shopt -u nullglob
for path in "${serial_paths[@]}"; do
    if command -v lsof >/dev/null 2>&1 && lsof "$path" > "$EVIDENCE_DIR/serial-owner-preflight.txt" 2>&1; then
        cat "$EVIDENCE_DIR/serial-owner-preflight.txt" | tee -a "$EVIDENCE_DIR/launcher.log"
        fail "serial device already owned: $path"
    fi
    if command -v fuser >/dev/null 2>&1 && fuser "$path" > "$EVIDENCE_DIR/serial-owner-preflight.txt" 2>&1; then
        cat "$EVIDENCE_DIR/serial-owner-preflight.txt" | tee -a "$EVIDENCE_DIR/launcher.log"
        fail "serial device already owned: $path"
    fi
done
pass "Station stopped and available serial devices are free"

resolve_config() {
    if [[ $# -ge 1 && -n "${1:-}" ]]; then
        printf '%s\n' "$1"
        return
    fi
    if [[ -n "${MATDOG_STATION_CONFIG:-}" ]]; then
        printf '%s\n' "$MATDOG_STATION_CONFIG"
        return
    fi
    local candidates=(
        "$REPO/station.yaml"
        "$REPO/station.yml"
        "/home/matteo-manicardi/MATDOG/station.yaml"
        "/home/matteo-manicardi/MATDOG/station.yml"
    )
    local candidate
    for candidate in "${candidates[@]}"; do
        if [[ -f "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return
        fi
    done
    mapfile -t discovered < <(
        find /home/matteo-manicardi/norma-core /home/matteo-manicardi/MATDOG \
            -maxdepth 6 -type f \( -name 'station.yaml' -o -name 'station.yml' \) \
            2>/dev/null | sort -u
    )
    if [[ "${#discovered[@]}" -eq 1 ]]; then
        printf '%s\n' "${discovered[0]}"
        return
    fi
    return 1
}

CONFIG="$(resolve_config "${1:-}")" || \
    fail "Station configuration not uniquely discoverable; pass its path as the first argument"
[[ -f "$CONFIG" ]] || fail "Station configuration does not exist: $CONFIG"
CONFIG="$(readlink -f "$CONFIG")"
pass "Station configuration: $CONFIG"
printf '%s\n' "$CONFIG" > "$EVIDENCE_DIR/station-config-path.txt"
sha256sum "$CONFIG" > "$EVIDENCE_DIR/station-config.sha256"

log "Building exact reviewed Station release binary"
cargo build --release --package station 2>&1 | tee "$EVIDENCE_DIR/cargo-build.log"
STATION_BIN="$REPO/target/release/station"
[[ -x "$STATION_BIN" ]] || fail "Station binary was not produced"
STATION_SHA="$(sha256sum "$STATION_BIN" | awk '{print $1}')"
printf '%s  %s\n' "$STATION_SHA" "$STATION_BIN" > "$EVIDENCE_DIR/station-binary.sha256"
pass "Station SHA256=$STATION_SHA"

log "Running offline Python self-test before hardware ownership"
python3 tools/matdog/matdog_headless_auto_calibrate.py --leg RF --self-test \
    2>&1 | tee "$EVIDENCE_DIR/runner-self-test.log"

NORMFS_DIR="$EVIDENCE_DIR/station-data"
mkdir -p "$NORMFS_DIR"
log "Starting sole Station serial owner, armed only for RF_LEG_STATE_MACHINE"
MATDOG_NATIVE_CALIBRATOR_ARM=RF_LEG_STATE_MACHINE \
RUST_LOG="${RUST_LOG:-info}" \
"$STATION_BIN" \
    --config "$CONFIG" \
    --tcp "$SERVER" \
    --normfs-base-folder "$NORMFS_DIR" \
    > "$STATION_LOG" 2>&1 &
STATION_PID=$!
printf '%s\n' "$STATION_PID" > "$EVIDENCE_DIR/station.pid"

python3 - "$SERVER" "$STATION_PID" <<'PY'
import os
import socket
import sys
import time
host, port = sys.argv[1].rsplit(':', 1)
pid = int(sys.argv[2])
port = int(port)
deadline = time.monotonic() + 30.0
while time.monotonic() < deadline:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        raise SystemExit("Station exited before TCP readiness")
    try:
        with socket.create_connection((host, port), timeout=0.5):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.25)
raise SystemExit("Station TCP readiness timed out")
PY
pass "Station ready on $SERVER with PID $STATION_PID"

RUNNER_STARTED=1
set +e
python3 tools/matdog/matdog_headless_auto_calibrate.py \
    --leg RF \
    --server "$SERVER" \
    --bus-serial "$BUS_SERIAL" \
    --station-pid "$STATION_PID" \
    --expected-station-sha256 "$STATION_SHA" \
    --output-dir "$RUNNER_DIR" \
    2>&1 | tee "$EVIDENCE_DIR/runner-console.log"
RUNNER_RC=${PIPESTATUS[0]}
set -e

if [[ "$RUNNER_RC" -ne 0 ]]; then
    fail "RF runner failed with exit code $RUNNER_RC"
fi

for _ in $(seq 1 60); do
    kill -0 "$STATION_PID" 2>/dev/null || break
    sleep 0.25
done
if kill -0 "$STATION_PID" 2>/dev/null; then
    fail "runner returned PASS but Station PID $STATION_PID is still live"
fi
STATION_PID=""

REPORT="$RUNNER_DIR/report.json"
[[ -f "$REPORT" ]] || fail "runner report missing: $REPORT"
python3 - "$REPORT" <<'PY'
import json
import sys
report = json.load(open(sys.argv[1], encoding='utf-8'))
required = {
    'result': 'PASS',
    'leg': 'RF',
    'arm_value': 'RF_LEG_STATE_MACHINE',
    'global_torque_off_verified': True,
    'eeprom_writes_sent': False,
    'register_writes_sent_by_runner': False,
}
errors = [f"{key}={report.get(key)!r} expected {value!r}" for key, value in required.items() if report.get(key) != value]
shutdown = report.get('station_shutdown') or {}
if not shutdown.get('stopped'):
    errors.append('Station shutdown was not verified')
if errors:
    raise SystemExit('; '.join(errors))
print('RF hardware report contract: PASS')
PY

for path in "${serial_paths[@]}"; do
    if command -v lsof >/dev/null 2>&1 && lsof "$path" > "$EVIDENCE_DIR/serial-owner-postflight.txt" 2>&1; then
        fail "serial device remains owned after Station shutdown: $path"
    fi
    if command -v fuser >/dev/null 2>&1 && fuser "$path" > "$EVIDENCE_DIR/serial-owner-postflight.txt" 2>&1; then
        fail "serial device remains owned after Station shutdown: $path"
    fi
done

sha256sum "$REPORT" "$STATION_LOG" "$EVIDENCE_DIR/runner-console.log" \
    > "$EVIDENCE_DIR/SHA256SUMS.txt"
pass "RF measurement completed, global torque OFF verified, Station stopped, serial released"
log "REPORT=$REPORT"
log "EVIDENCE=$EVIDENCE_DIR"
log "FINAL=PASS"
trap - EXIT INT TERM
exit 0
