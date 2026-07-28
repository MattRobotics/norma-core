#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE="LF_UPPER_M12_MAX"
BASE_COMMIT="32e3222c87016b7f5d7c1c1da497a4cea3e7b80a"
V9_COMMIT="2550dd938e5a6bf398eeb9ff4aa453ea1a9ca5d9"
ROBOT_COMMIT="4cdf440a2d37d1fe5e33c01f41687e460444a141"
CONFIG="$HOME/MATDOG/runtime/station/station_m12_pilot_deadband2.yaml"
CONFIG_SHA="f648988540d22fb38aa9b66e19bdf059f05047025b6abd229b64d7eff5f20bd1"
SERIAL_LINK="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114953-if00"
BUS="5B14114953"
ROOT="$HOME/MATDOG"
ARCHIVE="$ROOT/_archive/verification-artifacts"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ARTIFACT="$ARCHIVE/MATDOG_LF_UPPER_M12_MAX_RESTART_SAFE_V12_$STAMP"
WORKTREE="$ARTIFACT/norma-core-worktree"
TARGET="$ARTIFACT/cargo-target"
LOG="$ARTIFACT/offline_validation.log"
MARKER="$ARTIFACT/OFFLINE_VALIDATION_PASS.env"
HARDWARE_RUNNER="$HOME/Downloads/MATDOG_LF_UPPER_M12_MAX_HARDWARE_FROM_V12.sh"

section(){ printf '\n============================================================\n%s\n============================================================\n' "$1"; }
die(){ printf 'HARD BLOCK: %s\n' "$*" >&2; exit 1; }
mkdir -p "$ARTIFACT"
exec > >(tee -a "$LOG") 2>&1

section "V12 — VALIDAZIONE RESTART-SAFE, NESSUN HARDWARE"
printf '%s\n' \
  "Base software: V9 già validata con 73 test e replay completo." \
  "Correzione nuova: una prerequisite già nella propria posa valida viene conservata." \
  "Caso reale coperto: M42=2386, target prerequisite M42=2389." \
  "Station non verrà avviata e la seriale non verrà aperta."
read -r -p "Digita esattamente $PROFILE per continuare: " CONFIRM
[[ "$CONFIRM" == "$PROFILE" ]] || die "conferma non valida"

section "PREFLIGHT"
for cmd in git cargo rustfmt python3 sha256sum; do command -v "$cmd" >/dev/null || die "$cmd assente"; done
[[ -d "$HOME/norma-core/.git" ]] || die "clone norma-core assente"
[[ -d "$ROOT/robot-dog/.git" ]] || die "robot-dog assente"
[[ -f "$CONFIG" ]] || die "config assente"
[[ "$(sha256sum "$CONFIG"|awk '{print $1}')" == "$CONFIG_SHA" ]] || die "hash config inatteso"
[[ "$(git -C "$ROOT/robot-dog" rev-parse HEAD)" == "$ROBOT_COMMIT" ]] || die "robot-dog HEAD inatteso"
[[ -z "$(git -C "$ROOT/robot-dog" status --porcelain)" ]] || die "robot-dog non pulito"
[[ "$(git -C "$HOME/norma-core" rev-parse HEAD)" == "$BASE_COMMIT" ]] || die "norma-core main locale inatteso"
[[ "$(git -C "$HOME/norma-core" rev-parse matt/main)" == "$BASE_COMMIT" ]] || die "norma-core matt/main inatteso"
[[ -z "$(git -C "$HOME/norma-core" status --porcelain)" ]] || die "norma-core principale non pulito"

section "INDIVIDUAZIONE V9 VALIDATA"
V9_WORKTREE=""
while IFS= read -r candidate; do
  [[ -d "$candidate" ]] || continue
  if [[ "$(git -C "$candidate" rev-parse HEAD 2>/dev/null || true)" == "$V9_COMMIT" ]] && [[ -z "$(git -C "$candidate" status --porcelain 2>/dev/null || true)" ]]; then
    V9_WORKTREE="$candidate"; break
  fi
done < <(find "$ARCHIVE" -type d -path '*MATDOG_LF_UPPER_M12_MAX_OFFLINE_VALIDATION_V9_*/norma-core-worktree' | sort -r)
[[ -n "$V9_WORKTREE" ]] || die "worktree V9 pulita al commit $V9_COMMIT non trovata"
printf 'v9_worktree=%s\n' "$V9_WORKTREE"

BRANCH="matdog/local-m12-max-restart-safe-v12-$STAMP"
git -C "$HOME/norma-core" worktree add -b "$BRANCH" "$WORKTREE" "$V9_COMMIT"
[[ "$(git -C "$WORKTREE" rev-parse HEAD)" == "$V9_COMMIT" ]] || die "worktree non parte da V9"

section "PATCH MINIMA E ARCHITETTURALE"
python3 - "$WORKTREE" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1])
srcp=root/'software/drivers/st3215/src/auto_calibrate/matdog.rs'
tstp=root/'software/drivers/st3215/src/auto_calibrate/matdog_test.rs'
s=srcp.read_text(); t=tstp.read_text()

# Helper puro: è la policy restart-safe testabile in modo esaustivo.
anchor='fn is_allowed_matdog_ram_register(register: RamRegister) -> bool {'
helper='''fn startup_pose_is_profile_prerequisite(\n    profile: &ContactProfile,\n    motor_id: u8,\n    present_tick: u16,\n) -> Option<u16> {\n    profile\n        .prerequisites\n        .iter()\n        .find(|target| {\n            target.motor_id == motor_id\n                && circular_distance(present_tick, target.target_tick)\n                    <= STATIC_TOLERANCE_TICKS\n        })\n        .map(|target| target.target_tick)\n}\n\n'''
if 'fn startup_pose_is_profile_prerequisite(' not in s:
    if anchor not in s: raise SystemExit('anchor helper non trovato')
    s=s.replace(anchor,helper+anchor,1)

# Modifica soltanto la funzione V9 che contiene l'errore osservato da V10.
err='startup home recovery refused before motion'
ep=s.find(err)
if ep<0: raise SystemExit('funzione startup V9 non trovata')
fs=s.rfind('    async fn ',0,ep)
fe=s.find('\n    async fn ',ep)
if fe<0: fe=s.find('\n    fn ',ep)
if fe<0: raise SystemExit('fine funzione startup non trovata')
chunk=s[fs:fe]
line='            let distance = circular_distance(observation.position, HOME_TICK);\n'
if line not in chunk:
    raise SystemExit('anchor distanza startup non trovato')
inject=line+'''            if let Some(target_tick) = startup_pose_is_profile_prerequisite(\n                &self.profile,\n                motor_id,\n                observation.position,\n            ) {\n                info!(\n                    "MATDOG {} restart-safe startup: retain prerequisite M{} present={} target={} error={}",\n                    self.profile.label,\n                    motor_id,\n                    observation.position,\n                    target_tick,\n                    circular_distance(observation.position, target_tick)\n                );\n                continue;\n            }\n'''
chunk=chunk.replace(line,inject,1)
s=s[:fs]+chunk+s[fe:]

# Dopo la classificazione restart-safe non è più corretto pretendere che una
# prerequisite conservata sia a home prima di apply_prerequisites().
run=s.find('    async fn run(&mut self)')
apply=s.find('self.apply_prerequisites().await?;',run)
if run<0 or apply<0: raise SystemExit('sequenza run/apply non trovata')
verify=s.rfind('self.verify_all_near_home().await?;',run,apply)
if verify>=0:
    ls=s.rfind('\n',0,verify)+1
    le=s.find('\n',verify)+1
    prev=s.rfind('\n',0,ls-1)+1
    if 'next_phase' in s[prev:ls]: ls=prev
    s=s[:ls]+s[le:]

if s.find('startup_pose_is_profile_prerequisite(',s.find('startup home recovery refused before motion')-5000,s.find('startup home recovery refused before motion'))<0:
    raise SystemExit('helper non usato nella funzione startup')
if s.find('self.verify_all_near_home().await?;',run,apply)>=0:
    raise SystemExit('verifica all-home ancora presente prima delle prerequisite')

tests='''\n\n#[test]\nfn restart_v10_retains_m42_at_2386_for_lf_upper_max() {\n    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();\n    assert_eq!(startup_pose_is_profile_prerequisite(&profile, 42, 2386), Some(2389));\n    assert_eq!(startup_pose_is_profile_prerequisite(&profile, 42, 2389), Some(2389));\n    assert_eq!(startup_pose_is_profile_prerequisite(&profile, 42, 2048), None);\n}\n\n#[test]\nfn restart_does_not_accept_foreign_or_wrong_profile_prerequisites() {\n    let lf = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();\n    assert_eq!(startup_pose_is_profile_prerequisite(&lf, 32, 1707), None);\n    let rf = profile_for_arm_value("RF_UPPER_M22_MAX").unwrap();\n    assert_eq!(startup_pose_is_profile_prerequisite(&rf, 32, 1709), Some(1707));\n    assert_eq!(startup_pose_is_profile_prerequisite(&rf, 42, 2389), None);\n}\n\n#[test]\nfn restart_policy_is_exhaustive_for_all_profiles_motors_and_ticks() {\n    for profile in all_profiles().unwrap() {\n        for motor_id in MATDOG_MOTOR_IDS {\n            for tick in 0..=protocol::MAX_ANGLE_STEP {\n                let matched = startup_pose_is_profile_prerequisite(&profile, motor_id, tick);\n                let expected = profile.prerequisites.iter().find(|target| {\n                    target.motor_id == motor_id\n                        && circular_distance(tick, target.target_tick) <= STATIC_TOLERANCE_TICKS\n                }).map(|target| target.target_tick);\n                assert_eq!(matched, expected);\n            }\n        }\n    }\n}\n\n#[test]\nfn restart_trace_v8_v10_keeps_only_valid_m42_residue() {\n    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();\n    for (motor_id, tick) in [(11,2077),(22,2069),(23,2034),(32,2029),(33,2074),(43,2072)] {\n        assert_eq!(startup_pose_is_profile_prerequisite(&profile,motor_id,tick),None);\n        let d=circular_distance(tick,HOME_TICK);\n        assert!(d>STATIC_TOLERANCE_TICKS && d<=STARTUP_HOME_RECOVERY_LIMIT_TICKS);\n    }\n    assert_eq!(startup_pose_is_profile_prerequisite(&profile,42,2386),Some(2389));\n}\n'''
if 'restart_v10_retains_m42_at_2386_for_lf_upper_max' not in t: t+=tests
srcp.write_text(s); tstp.write_text(t)
PY

RUST_FILES=(software/drivers/st3215/src/auto_calibrate/matdog.rs software/drivers/st3215/src/auto_calibrate/matdog_test.rs software/drivers/st3215/src/port.rs)
(
 cd "$WORKTREE"
 rustfmt --edition 2021 --config skip_children=true "${RUST_FILES[@]}"
 rustfmt --edition 2021 --check --config skip_children=true "${RUST_FILES[@]}"
)

section "AUDIT STATICO"
python3 - "$WORKTREE" <<'PY'
from pathlib import Path
import sys
r=Path(sys.argv[1]); s=(r/'software/drivers/st3215/src/auto_calibrate/matdog.rs').read_text(); t=(r/'software/drivers/st3215/src/auto_calibrate/matdog_test.rs').read_text()
run=s.find('    async fn run(&mut self)'); apply=s.find('self.apply_prerequisites().await?;',run)
checks={
 'helper policy': 'fn startup_pose_is_profile_prerequisite(' in s,
 'V10 M42=2386 test': 'restart_v10_retains_m42_at_2386_for_lf_upper_max' in t,
 '24 profiles x 12 motors x 4096 ticks': 'restart_policy_is_exhaustive_for_all_profiles_motors_and_ticks' in t,
 'V8+V10 trace': 'restart_trace_v8_v10_keeps_only_valid_m42_residue' in t,
 'no strict all-home before prerequisites': s.find('self.verify_all_near_home().await?;',run,apply)<0,
 'V9 probing regression retained': 'dynamic_probe_phases_ignore_probe_and_disable_it_before_restore' in t,
 'RAM-only source': all(x not in s for x in ('EepromRegister','RamRegister::Lock','reset_calibration: Some','freeze_calibration: Some','reg_write: Some','action: Some')),
}
for k,v in checks.items(): print(('PASS' if v else 'FAIL')+': '+k)
if not all(checks.values()): raise SystemExit(1)
PY

section "TEST OFFLINE"
export CARGO_TARGET_DIR="$TARGET"
(
 cd "$WORKTREE"
 cargo test --offline --package st3215 restart_v10_retains_m42_at_2386_for_lf_upper_max
 cargo test --offline --package st3215 restart_does_not_accept_foreign_or_wrong_profile_prerequisites
 cargo test --offline --package st3215 restart_policy_is_exhaustive_for_all_profiles_motors_and_ticks
 cargo test --offline --package st3215 restart_trace_v8_v10_keeps_only_valid_m42_residue
 cargo test --offline --package st3215 dynamic_probe_phases_ignore_probe_and_disable_it_before_restore
 cargo test --offline --package st3215
)

section "COMMIT E BUILD RELEASE"
(
 cd "$WORKTREE"
 git add -- "${RUST_FILES[@]}"
 git commit -m "fix(matdog): retain valid prerequisite poses on restart"
)
PATCH_COMMIT="$(git -C "$WORKTREE" rev-parse HEAD)"
[[ "$PATCH_COMMIT" != "$V9_COMMIT" ]] || die "commit non creato"
[[ -z "$(git -C "$WORKTREE" status --porcelain)" ]] || die "worktree non pulito"

UI_SRC="$HOME/norma-core/software/station/clients/station-viewer/dist"
UI_DST="$WORKTREE/software/station/clients/station-viewer/dist"
[[ -f "$UI_SRC/index.html" ]] || die "asset UI assenti"
rm -rf "$UI_DST"; mkdir -p "$(dirname "$UI_DST")"; cp -a "$UI_SRC" "$UI_DST"
diff -qr "$UI_SRC" "$UI_DST" >/dev/null || die "asset UI non identici"
(
 cd "$WORKTREE"
 cargo clean --release --package station
 cargo build --release --package station --offline
)
BIN="$TARGET/release/station"
[[ -x "$BIN" ]] || die "binario assente"
VERSION="$($BIN --version)"; SHORT="${PATCH_COMMIT:0:7}"
[[ "$VERSION" == *"($SHORT)"* ]] || die "metadata binario inatteso: $VERSION"
BIN_SHA="$(sha256sum "$BIN"|awk '{print $1}')"

cat > "$MARKER" <<EOF
result=PASS
hardware_started=false
serial_opened=false
profile=$PROFILE
base_commit=$BASE_COMMIT
v9_commit=$V9_COMMIT
restart_safe_commit=$PATCH_COMMIT
station_version=$VERSION
station_sha256=$BIN_SHA
worktree=$WORKTREE
binary=$BIN
config=$CONFIG
config_sha256=$CONFIG_SHA
EOF
MARKER_SHA="$(sha256sum "$MARKER"|awk '{print $1}')"

section "GENERA RUNNER HARDWARE SEPARATO"
cat > "$HARDWARE_RUNNER" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
PROFILE="$PROFILE"; MARKER="$MARKER"; MARKER_SHA="$MARKER_SHA"; SERIAL_LINK="$SERIAL_LINK"; BUS="$BUS"
section(){ printf '\\n============================================================\\n%s\\n============================================================\\n' "\$1"; }
die(){ printf 'HARD BLOCK: %s\\n' "\$*" >&2; exit 1; }
section "HARDWARE V12 — BINARIO RESTART-SAFE VALIDATO OFFLINE"
[[ -f "\$MARKER" ]] || die "marker assente"
[[ "\$(sha256sum "\$MARKER"|awk '{print \$1}')" == "\$MARKER_SHA" ]] || die "marker modificato"
source "\$MARKER"
[[ "\$result" == PASS && "\$hardware_started" == false && "\$serial_opened" == false ]] || die "marker non valido"
[[ "\$(git -C "\$worktree" rev-parse HEAD)" == "\$restart_safe_commit" ]] || die "commit diverso"
[[ -z "\$(git -C "\$worktree" status --porcelain)" ]] || die "worktree non pulito"
[[ "\$(sha256sum "\$binary"|awk '{print \$1}')" == "\$station_sha256" ]] || die "binario modificato"
[[ "\$(sha256sum "\$config"|awk '{print \$1}')" == "\$config_sha256" ]] || die "config modificata"
pgrep -af '(^|/)station( |$)' >/dev/null && die "Station già attiva"
[[ -e "\$SERIAL_LINK" ]] || die "seriale assente"
command -v lsof >/dev/null && lsof "\$SERIAL_LINK" 2>/dev/null | grep -q . && die "seriale occupata"
for port in 8888 8889; do command -v ss >/dev/null && ss -ltn | awk '{print \$4}' | grep -Eq "[:.]\$port\$" && die "porta \$port occupata"; done
read -r -p "Digita esattamente \$PROFILE per autorizzare la prova hardware: " CONFIRM
[[ "\$CONFIRM" == "\$PROFILE" ]] || die "conferma non valida"
RUN="\$(dirname "\$MARKER")/hardware_v12_\$(date -u +%Y%m%dT%H%M%SZ)"; mkdir -p "\$RUN/data"; LOG="\$RUN/station.log"
section "AVVIO STATION"
printf '%s\\n' 'Robot sostenuto, quattro zampe libere, master disconnect raggiungibile.' 'UI: bus 5B14114953, soltanto Auto Calibrate, mai Save/Reset.'
MATDOG_NATIVE_CALIBRATOR_ARM="\$PROFILE" RUST_LOG=info \
 "\$binary" --config "\$config" --tcp 127.0.0.1:8888 --web 127.0.0.1:8889 --normfs-base-folder "\$RUN/data" \
 > >(tee -a "\$LOG") 2>&1 &
PID=\$!; trap 'kill -INT "\$PID" 2>/dev/null || true; wait "\$PID" 2>/dev/null || true' EXIT INT TERM
printf 'station_pid=%s\\n' "\$PID"
for _ in {1..60}; do kill -0 "\$PID" 2>/dev/null || { tail -n 120 "\$LOG"; die "Station terminata in avvio"; }; grep -Eq '8889|web server|Web server' "\$LOG" && break; sleep 1; done
command -v xdg-open >/dev/null && xdg-open http://127.0.0.1:8889 >/dev/null 2>&1 || true
while kill -0 "\$PID" 2>/dev/null; do
 if grep -q "MATDOG \$PROFILE complete:" "\$LOG"; then RESULT=PASS; break; fi
 if grep -q "MATDOG native profile failed:" "\$LOG"; then RESULT=FAIL; break; fi
 sleep 1
done
RESULT=\${RESULT:-FAIL}; kill -INT "\$PID" 2>/dev/null || true; wait "\$PID" 2>/dev/null || true; trap - EXIT INT TERM
section "RISULTATO"; printf 'result=%s\\nlog=%s\\n' "\$RESULT" "\$LOG"
grep -E 'restart-safe startup|startup home recovery| contact:|MATDOG .* complete:|MATDOG native profile failed:' "\$LOG" || true
[[ "\$RESULT" == PASS ]]
EOF
chmod +x "$HARDWARE_RUNNER"

section "RISULTATO V12"
printf 'result=PASS\nhardware_started=false\nserial_opened=false\nrestart_safe_commit=%s\nstation_version=%s\nstation_sha256=%s\nmarker=%s\nmarker_sha256=%s\nhardware_runner=%s\n' "$PATCH_COMMIT" "$VERSION" "$BIN_SHA" "$MARKER" "$MARKER_SHA" "$HARDWARE_RUNNER"
printf '\nNessun hardware è stato avviato. Per la prova supervisionata:\n  bash %q\n' "$HARDWARE_RUNNER"
