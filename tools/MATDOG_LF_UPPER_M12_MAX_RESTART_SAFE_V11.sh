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
ARTIFACT="$ARCHIVE/MATDOG_LF_UPPER_M12_MAX_RESTART_SAFE_V11_$STAMP"
WORKTREE="$ARTIFACT/norma-core-worktree"
TARGET="$ARTIFACT/cargo-target"
LOG="$ARTIFACT/offline_validation.log"
MARKER="$ARTIFACT/OFFLINE_VALIDATION_PASS.env"
HARDWARE_RUNNER="$HOME/Downloads/MATDOG_LF_UPPER_M12_MAX_HARDWARE_FROM_V11.sh"

section() { printf '\n============================================================\n%s\n============================================================\n' "$1"; }
die() { printf 'HARD BLOCK: %s\n' "$*" >&2; exit 1; }

mkdir -p "$ARTIFACT"
exec > >(tee -a "$LOG") 2>&1

section "MATDOG M12 MAX V11 — ARCHITETTURA RESTART-SAFE"
printf '%s\n' \
  "Questa esecuzione è esclusivamente offline." \
  "Non avvia Station, non apre la seriale e non invia comandi ai servo." \
  "Corregge il caso reale V10: M42=2386 viene riconosciuto come prerequisite valida (target 2389), non come outlier." \
  "Alla fine genera un runner hardware separato, ma NON lo esegue."
read -r -p "Digita esattamente $PROFILE per continuare: " CONFIRM
[[ "$CONFIRM" == "$PROFILE" ]] || die "conferma non valida"

section "PREFLIGHT IMMUTABILE"
command -v git >/dev/null || die "git assente"
command -v cargo >/dev/null || die "cargo assente"
command -v rustfmt >/dev/null || die "rustfmt assente"
command -v python3 >/dev/null || die "python3 assente"
[[ -d "$HOME/norma-core/.git" ]] || die "clone principale norma-core assente"
[[ -d "$ROOT/robot-dog/.git" ]] || die "robot-dog assente"
[[ -f "$CONFIG" ]] || die "config Station assente"
[[ "$(sha256sum "$CONFIG" | awk '{print $1}')" == "$CONFIG_SHA" ]] || die "hash config inatteso"
[[ "$(git -C "$ROOT/robot-dog" rev-parse HEAD)" == "$ROBOT_COMMIT" ]] || die "robot-dog HEAD inatteso"
[[ -z "$(git -C "$ROOT/robot-dog" status --porcelain)" ]] || die "robot-dog non pulito"
[[ -z "$(git -C "$HOME/norma-core" status --porcelain)" ]] || die "norma-core principale non pulito"
[[ "$(git -C "$HOME/norma-core" rev-parse HEAD)" == "$BASE_COMMIT" ]] || die "norma-core main locale inatteso"
[[ "$(git -C "$HOME/norma-core" rev-parse matt/main)" == "$BASE_COMMIT" ]] || die "norma-core matt/main inatteso"

section "RECUPERO DEL COMMIT V9 GIÀ VALIDATO"
V9_WORKTREE=""
while IFS= read -r candidate; do
  [[ -d "$candidate/.git" || -f "$candidate/.git" ]] || continue
  if [[ "$(git -C "$candidate" rev-parse HEAD 2>/dev/null || true)" == "$V9_COMMIT" ]]; then
    V9_WORKTREE="$candidate"
    break
  fi
done < <(find "$ARCHIVE" -type d -path '*MATDOG_LF_UPPER_M12_MAX_OFFLINE_VALIDATION_V9_*/norma-core-worktree' | sort -r)
[[ -n "$V9_WORKTREE" ]] || die "worktree V9 al commit $V9_COMMIT non trovato"
[[ -z "$(git -C "$V9_WORKTREE" status --porcelain)" ]] || die "worktree V9 non pulito"
printf 'v9_worktree=%s\n' "$V9_WORKTREE"

BRANCH="matdog/local-m12-max-restart-safe-v11-$STAMP"
git -C "$HOME/norma-core" worktree add -b "$BRANCH" "$WORKTREE" "$V9_COMMIT"
[[ "$(git -C "$WORKTREE" rev-parse HEAD)" == "$V9_COMMIT" ]] || die "worktree V11 non parte da V9"

section "PATCH ARCHITETTURALE RESTART-SAFE"
python3 - "$WORKTREE" <<'PY'
from pathlib import Path
import re, sys

root = Path(sys.argv[1])
source_path = root / "software/drivers/st3215/src/auto_calibrate/matdog.rs"
test_path = root / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"
source = source_path.read_text()
tests = test_path.read_text()

needle = "fn is_allowed_matdog_ram_register(register: RamRegister) -> bool {"
if needle not in source:
    raise SystemExit("anchor helper non trovato")
helper = r'''
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StartupPoseClass {
    Home,
    RetainedPrerequisite { target_tick: u16 },
    RecoverToHome,
    Outlier,
}

fn classify_startup_pose(
    profile: &ContactProfile,
    motor_id: u8,
    present_tick: u16,
) -> StartupPoseClass {
    let home_distance = circular_distance(present_tick, HOME_TICK);
    if home_distance <= STATIC_TOLERANCE_TICKS {
        return StartupPoseClass::Home;
    }
    if let Some(target) = profile
        .prerequisites
        .iter()
        .find(|target| target.motor_id == motor_id)
    {
        if circular_distance(present_tick, target.target_tick) <= STATIC_TOLERANCE_TICKS {
            return StartupPoseClass::RetainedPrerequisite {
                target_tick: target.target_tick,
            };
        }
    }
    if home_distance <= STARTUP_HOME_RECOVERY_LIMIT_TICKS {
        StartupPoseClass::RecoverToHome
    } else {
        StartupPoseClass::Outlier
    }
}

'''
if "enum StartupPoseClass" not in source:
    source = source.replace(needle, helper + needle, 1)

error_token = "startup home recovery refused before motion"
pos = source.find(error_token)
if pos < 0:
    raise SystemExit("funzione startup V9 non trovata")
fn_start = source.rfind("    async fn ", 0, pos)
if fn_start < 0:
    raise SystemExit("inizio funzione startup non trovato")
brace = source.find("{", fn_start)
depth = 0
fn_end = None
for i in range(brace, len(source)):
    if source[i] == "{": depth += 1
    elif source[i] == "}":
        depth -= 1
        if depth == 0:
            fn_end = i + 1
            break
if fn_end is None:
    raise SystemExit("fine funzione startup non trovata")
old_fn = source[fn_start:fn_end]
name_m = re.search(r"async fn\s+(\w+)\s*\(", old_fn)
if not name_m:
    raise SystemExit("nome funzione startup non trovato")
name = name_m.group(1)
sig_end = old_fn.find("{")
signature = old_fn[:sig_end].rstrip()
new_body = r''' {
        let mut recover_to_home = Vec::new();
        let mut retained_prerequisites = Vec::new();
        let mut outliers = Vec::new();

        for motor_id in MATDOG_MOTOR_IDS {
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;
            if observation.torque_enabled {
                return Err(format!(
                    "M{motor_id} unexpectedly torque-enabled during startup classification"
                )
                .into());
            }
            if observation.has_driver_error || observation.status != 0 {
                return Err(format!(
                    "M{motor_id} unhealthy during startup classification: status=0x{:02X}, driver_error={}",
                    observation.status, observation.has_driver_error
                )
                .into());
            }

            match classify_startup_pose(&self.profile, motor_id, observation.position) {
                StartupPoseClass::Home => {}
                StartupPoseClass::RetainedPrerequisite { target_tick } => {
                    info!(
                        "MATDOG {} restart-safe startup: retain prerequisite M{} present={} target={} error={}",
                        self.profile.label,
                        motor_id,
                        observation.position,
                        target_tick,
                        circular_distance(observation.position, target_tick)
                    );
                    retained_prerequisites.push((motor_id, target_tick));
                }
                StartupPoseClass::RecoverToHome => recover_to_home.push((
                    motor_id,
                    observation.position,
                    circular_distance(observation.position, HOME_TICK),
                )),
                StartupPoseClass::Outlier => outliers.push((
                    motor_id,
                    observation.position,
                    circular_distance(observation.position, HOME_TICK),
                )),
            }
        }

        if !outliers.is_empty() {
            let details = outliers
                .iter()
                .map(|(motor_id, present, distance)| {
                    format!("M{motor_id}:present={present},distance={distance}")
                })
                .collect::<Vec<_>>()
                .join(",");
            return Err(format!(
                "startup classification refused before motion; home_limit={}; outliers=[{}]",
                STARTUP_HOME_RECOVERY_LIMIT_TICKS, details
            )
            .into());
        }

        for (motor_id, present, distance) in recover_to_home {
            info!(
                "MATDOG {} startup home recovery: M{} present={} target={} distance={}",
                self.profile.label, motor_id, present, HOME_TICK, distance
            );
            self.prepare_motor_startup_recovery(motor_id).await?;
            self.move_motor_to_startup_home(motor_id).await?;
            self.set_motor_torque_startup_verified(motor_id, false).await?;
        }

        for (motor_id, target_tick) in retained_prerequisites {
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;
            if observation.torque_enabled
                || observation.has_driver_error
                || observation.status != 0
                || circular_distance(observation.position, target_tick) > STATIC_TOLERANCE_TICKS
            {
                return Err(format!(
                    "retained prerequisite M{motor_id} changed during startup: present={}, target={}, torque={}, status=0x{:02X}",
                    observation.position, target_tick, observation.torque_enabled, observation.status
                )
                .into());
            }
        }
        Ok(())
    }'''
new_fn = signature + new_body
source = source[:fn_start] + new_fn + source[fn_end:]

strict = '''        self.next_phase("Verify all joints near digital home")?;\n        self.verify_all_near_home().await?;\n\n'''
if strict in source:
    source = source.replace(strict, "", 1)
else:
    # V9 may use a renamed phase; remove the unique verifier call only from run().
    run_pos = source.find("    async fn run(&mut self)")
    apply_pos = source.find("self.apply_prerequisites().await?;", run_pos)
    verify_pos = source.rfind("self.verify_all_near_home().await?;", run_pos, apply_pos)
    if verify_pos >= 0:
        line_start = source.rfind("\n", 0, verify_pos) + 1
        line_end = source.find("\n", verify_pos) + 1
        prev_start = source.rfind("\n", 0, line_start - 1) + 1
        if "next_phase" in source[prev_start:line_start]:
            line_start = prev_start
        source = source[:line_start] + source[line_end:]

required = [
    "classify_startup_pose(&self.profile, motor_id, observation.position)",
    "retain prerequisite M{}",
    "prepare_motor_startup_recovery(motor_id).await?",
    "self.apply_prerequisites().await?;",
]
missing = [x for x in required if x not in source]
if missing:
    raise SystemExit(f"patch incompleta: {missing}")

append = r'''

#[test]
fn restart_safe_classifier_accepts_v10_m42_prerequisite_residue() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(
        classify_startup_pose(&profile, 42, 2386),
        StartupPoseClass::RetainedPrerequisite { target_tick: 2389 }
    );
    assert_eq!(classify_startup_pose(&profile, 11, 2077), StartupPoseClass::RecoverToHome);
    assert_eq!(classify_startup_pose(&profile, 22, 2069), StartupPoseClass::RecoverToHome);
}

#[test]
fn restart_safe_classifier_rejects_foreign_and_unbounded_residues() {
    let lf = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(classify_startup_pose(&lf, 32, 1707), StartupPoseClass::Outlier);
    assert_eq!(classify_startup_pose(&lf, 42, 2500), StartupPoseClass::Outlier);
    let rf = profile_for_arm_value("RF_UPPER_M22_MAX").unwrap();
    assert_eq!(classify_startup_pose(&rf, 32, 1709), StartupPoseClass::RetainedPrerequisite { target_tick: 1707 });
    assert_eq!(classify_startup_pose(&rf, 42, 2389), StartupPoseClass::Outlier);
}

#[test]
fn restart_safe_state_machine_replays_clean_v8_and_v10_entries() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    let v8 = [(11, 2077), (22, 2069), (23, 2034), (32, 2029), (33, 2074), (43, 2072)];
    for (motor_id, tick) in v8 {
        assert_eq!(classify_startup_pose(&profile, motor_id, tick), StartupPoseClass::RecoverToHome);
    }
    assert_eq!(classify_startup_pose(&profile, 42, 2386), StartupPoseClass::RetainedPrerequisite { target_tick: 2389 });
    for motor_id in MATDOG_MOTOR_IDS {
        if motor_id != 42 {
            assert_eq!(classify_startup_pose(&profile, motor_id, HOME_TICK), StartupPoseClass::Home);
        }
    }
}

#[test]
fn restart_safe_classifier_is_exhaustive_for_all_profiles_and_ticks() {
    for profile in all_profiles().unwrap() {
        for motor_id in MATDOG_MOTOR_IDS {
            for tick in 0..=protocol::MAX_ANGLE_STEP {
                match classify_startup_pose(&profile, motor_id, tick) {
                    StartupPoseClass::Home => assert!(circular_distance(tick, HOME_TICK) <= STATIC_TOLERANCE_TICKS),
                    StartupPoseClass::RetainedPrerequisite { target_tick } => {
                        assert!(profile.prerequisites.iter().any(|p| p.motor_id == motor_id && p.target_tick == target_tick));
                        assert!(circular_distance(tick, target_tick) <= STATIC_TOLERANCE_TICKS);
                    }
                    StartupPoseClass::RecoverToHome => {
                        let d = circular_distance(tick, HOME_TICK);
                        assert!(d > STATIC_TOLERANCE_TICKS && d <= STARTUP_HOME_RECOVERY_LIMIT_TICKS);
                    }
                    StartupPoseClass::Outlier => {
                        assert!(circular_distance(tick, HOME_TICK) > STARTUP_HOME_RECOVERY_LIMIT_TICKS);
                        assert!(!profile.prerequisites.iter().any(|p| p.motor_id == motor_id && circular_distance(tick, p.target_tick) <= STATIC_TOLERANCE_TICKS));
                    }
                }
            }
        }
    }
}
'''
if "restart_safe_classifier_accepts_v10_m42_prerequisite_residue" not in tests:
    tests += append
source_path.write_text(source)
test_path.write_text(tests)
print(f"startup_function={name}")
PY

RUST_FILES=(
  software/drivers/st3215/src/auto_calibrate/matdog.rs
  software/drivers/st3215/src/auto_calibrate/matdog_test.rs
  software/drivers/st3215/src/port.rs
)
(
  cd "$WORKTREE"
  rustfmt --edition 2021 --config skip_children=true "${RUST_FILES[@]}"
  rustfmt --edition 2021 --check --config skip_children=true "${RUST_FILES[@]}"
)

section "AUDIT STATICO E MUTATION GATES"
python3 - "$WORKTREE" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1])
s=(root/'software/drivers/st3215/src/auto_calibrate/matdog.rs').read_text()
t=(root/'software/drivers/st3215/src/auto_calibrate/matdog_test.rs').read_text()
checks={
 'classifier': 'enum StartupPoseClass' in s,
 'v10_m42': 'classify_startup_pose(&profile, 42, 2386)' in t,
 'all_24_all_ticks': 'restart_safe_classifier_is_exhaustive_for_all_profiles_and_ticks' in t,
 'no_strict_home_before_prereq': s.find('self.verify_all_near_home().await?;') == -1 or s.find('self.verify_all_near_home().await?;') > s.find('self.apply_prerequisites().await?;'),
 'probe_fix_v9': 'dynamic_probe_phases_ignore_probe_and_disable_it_before_restore' in t,
 'ram_only': all(x not in s for x in ('EepromRegister','RamRegister::Lock','reset_calibration: Some','freeze_calibration: Some','reg_write: Some','action: Some')),
}
for k,v in checks.items():
 print(('PASS' if v else 'FAIL')+': '+k)
if not all(checks.values()): raise SystemExit(1)
PY

section "TEST MIRATI E SUITE COMPLETA — OFFLINE"
export CARGO_TARGET_DIR="$TARGET"
(
  cd "$WORKTREE"
  cargo test --offline --package st3215 restart_safe_classifier_accepts_v10_m42_prerequisite_residue -- --exact
  cargo test --offline --package st3215 restart_safe_classifier_rejects_foreign_and_unbounded_residues -- --exact
  cargo test --offline --package st3215 restart_safe_state_machine_replays_clean_v8_and_v10_entries -- --exact
  cargo test --offline --package st3215 restart_safe_classifier_is_exhaustive_for_all_profiles_and_ticks -- --exact
  cargo test --offline --package st3215 dynamic_probe_phases_ignore_probe_and_disable_it_before_restore -- --exact
  cargo test --offline --package st3215
)

section "COMMIT LOCALE E BUILD RELEASE"
(
  cd "$WORKTREE"
  git add -- "${RUST_FILES[@]}"
  git commit -m "fix(matdog): make profile startup restart-safe"
)
PATCH_COMMIT="$(git -C "$WORKTREE" rev-parse HEAD)"
[[ "$PATCH_COMMIT" != "$V9_COMMIT" ]] || die "nessun nuovo commit"
[[ -z "$(git -C "$WORKTREE" status --porcelain)" ]] || die "worktree non pulito dopo commit"

UI_SRC="$HOME/norma-core/software/station/clients/station-viewer/dist"
UI_DST="$WORKTREE/software/station/clients/station-viewer/dist"
[[ -f "$UI_SRC/index.html" ]] || die "asset UI assenti nel clone principale"
rm -rf "$UI_DST"
mkdir -p "$(dirname "$UI_DST")"
cp -a "$UI_SRC" "$UI_DST"
diff -qr "$UI_SRC" "$UI_DST" >/dev/null || die "copia UI non identica"

(
  cd "$WORKTREE"
  cargo clean --release --package station
  cargo build --release --package station --offline
)
BIN="$TARGET/release/station"
[[ -x "$BIN" ]] || die "binario Station assente"
VERSION="$($BIN --version)"
SHORT="${PATCH_COMMIT:0:7}"
[[ "$VERSION" == *"($SHORT)"* ]] || die "metadata binario inatteso: $VERSION"
BIN_SHA="$(sha256sum "$BIN" | awk '{print $1}')"

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
MARKER_SHA="$(sha256sum "$MARKER" | awk '{print $1}')"

section "GENERA RUNNER HARDWARE SEPARATO — NON ESEGUITO"
cat > "$HARDWARE_RUNNER" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
PROFILE="$PROFILE"
MARKER="$MARKER"
MARKER_SHA="$MARKER_SHA"
SERIAL_LINK="$SERIAL_LINK"
BUS="$BUS"
section(){ printf '\\n============================================================\\n%s\\n============================================================\\n' "\$1"; }
die(){ printf 'HARD BLOCK: %s\\n' "\$*" >&2; exit 1; }
section "HARDWARE — BINARIO V11 RESTART-SAFE GIÀ VALIDATO OFFLINE"
[[ -f "\$MARKER" ]] || die "marker assente"
[[ "\$(sha256sum "\$MARKER"|awk '{print \$1}')" == "\$MARKER_SHA" ]] || die "marker modificato"
source "\$MARKER"
[[ "\$result" == PASS && "\$hardware_started" == false && "\$serial_opened" == false ]] || die "marker non valido"
[[ "\$(git -C "\$worktree" rev-parse HEAD)" == "\$restart_safe_commit" ]] || die "commit worktree diverso"
[[ -z "\$(git -C "\$worktree" status --porcelain)" ]] || die "worktree non pulito"
[[ "\$(sha256sum "\$binary"|awk '{print \$1}')" == "\$station_sha256" ]] || die "binario modificato"
[[ "\$(sha256sum "\$config"|awk '{print \$1}')" == "\$config_sha256" ]] || die "config modificata"
pgrep -af '(^|/)station( |$)' >/dev/null && die "Station già attiva"
[[ -e "\$SERIAL_LINK" ]] || die "seriale assente"
command -v lsof >/dev/null && lsof "\$SERIAL_LINK" 2>/dev/null | grep -q . && die "seriale occupata"
for port in 8888 8889; do command -v ss >/dev/null && ss -ltn | awk '{print \$4}' | grep -Eq "[:.]\$port\$" && die "porta \$port occupata"; done
read -r -p "Digita esattamente \$PROFILE per autorizzare la prova hardware: " CONFIRM
[[ "\$CONFIRM" == "\$PROFILE" ]] || die "conferma non valida"
RUN="\$(dirname "\$MARKER")/hardware_v11_\$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "\$RUN/data"
LOG="\$RUN/station.log"
section "AVVIO STATION"
printf '%s\\n' 'Robot completamente sostenuto, quattro zampe libere, master disconnect raggiungibile.' 'Nella UI: seleziona 5B14114953, premi soltanto Auto Calibrate, mai Save/Reset.'
MATDOG_NATIVE_CALIBRATOR_ARM="\$PROFILE" RUST_LOG=info \
  "\$binary" --config "\$config" --web-address 127.0.0.1:8889 --tcp-address 127.0.0.1:8888 --data-dir "\$RUN/data" \
  > >(tee -a "\$LOG") 2>&1 &
PID=\$!
trap 'kill -INT "\$PID" 2>/dev/null || true; wait "\$PID" 2>/dev/null || true' EXIT INT TERM
printf 'station_pid=%s\\n' "\$PID"
for _ in {1..60}; do
  kill -0 "\$PID" 2>/dev/null || { tail -n 100 "\$LOG"; die "Station terminata in avvio"; }
  grep -Eq 'web server|Web server|8889|Station started|Starting web' "\$LOG" && break
  sleep 1
done
command -v xdg-open >/dev/null && xdg-open http://127.0.0.1:8889 >/dev/null 2>&1 || true
while kill -0 "\$PID" 2>/dev/null; do
  if grep -q "MATDOG \$PROFILE complete:" "\$LOG"; then RESULT=PASS; break; fi
  if grep -q "MATDOG native profile failed:" "\$LOG"; then RESULT=FAIL; break; fi
  sleep 1
done
RESULT=\${RESULT:-FAIL}
kill -INT "\$PID" 2>/dev/null || true
wait "\$PID" 2>/dev/null || true
trap - EXIT INT TERM
section "RISULTATO"
printf 'result=%s\\nlog=%s\\n' "\$RESULT" "\$LOG"
grep -E 'restart-safe startup|startup home recovery| contact:|MATDOG .* complete:|MATDOG native profile failed:' "\$LOG" || true
[[ "\$RESULT" == PASS ]]
EOF
chmod +x "$HARDWARE_RUNNER"

section "RISULTATO OFFLINE V11"
printf 'result=PASS\nhardware_started=false\nserial_opened=false\nrestart_safe_commit=%s\nstation_version=%s\nstation_sha256=%s\nmarker=%s\nmarker_sha256=%s\nhardware_runner=%s\n' \
  "$PATCH_COMMIT" "$VERSION" "$BIN_SHA" "$MARKER" "$MARKER_SHA" "$HARDWARE_RUNNER"
printf '\nVALIDAZIONE COMPLETATA. Nessun hardware è stato avviato.\nPer la prova supervisionata esegui poi:\n  bash %q\n' "$HARDWARE_RUNNER"
