# MATDOG calibrator — canonical LF V25 state

## Canonical calibration architecture update — 2026-08-07

Before any further RF/RH/LH implementation, read:

```text
tools/matdog/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
```

That document is the current cross-repository development contract and supersedes the 2026-08-05 RF development prescriptions wherever they conflict.

The immediate development order is now:

```text
1. robot-dog: offline Geometry Compiler / 24 mesh-predicted contacts and safe paths
2. norma-core: refactor to one generic V25-derived full-leg engine
3. hardware: RF -> RH -> LH
```

Do not continue runtime wiring of the current duplicated local `RfSessionStateMachine` before the geometry/profile review.

The corrected q=0 contract is also explicit there: manual/visual home is a seed only; the final model q=0 is derived from model geometry plus repeatable hardware contacts, staged/verified in RAM/software, and only then may a separately authorized transactional EEPROM freeze be considered.

The current local RF worktree/checkpoint must be preserved as evidence:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
checkpoint base: b2f7dac2eab7147917fccdfde702360da82ab7de
```

## Validation status

The only mechanically hardware-validated MATDOG calibration flow is **LF V25**.

```text
measurement sequence: 58/58 DONE
runner result: PASS
global torque OFF: verified
persistent stage: LF_STAGED
EEPROM transaction: LF_FROZEN
persistent profile: LF_FROZEN
```

```text
RF: not yet mechanically hardware validated
RH: not yet mechanically hardware validated
LH: not yet mechanically hardware validated
complete all-leg persistent profile: not yet validated
```

Older V28–V42 and “all legs” branches were development experiments. They must not be used, merged or revived as current calibration programs.

## What LF V25 does

The digital-home commissioning program remains separate. LF V25:

1. measures all six physical LF endpoints through Station;
2. checks repeatability and supervised hardware evidence;
3. compares the measured span with the URDF;
4. derives an affine joint model and q0;
5. stages q0 in RAM;
6. stops Station and verifies serial-adapter release;
7. performs an explicit transactional EEPROM freeze;
8. writes the persistent LF and partial global profiles;
9. verifies global torque OFF.

## Final LF evidence

| Joint | Motor | Final MIN contact | Final MAX contact | Affine q0 before EEPROM |
|---|---:|---:|---:|---:|
| LF hip | M13 | 2535 | 1600 | 2067 |
| LF upper | M12 | 1439 | 3443 | 2040 |
| LF lower | M11 | 3093 | 1658 | 2074 |

Measured mechanical spans versus the URDF:

| Joint | URDF span | Measured span | Difference |
|---|---:|---:|---:|
| M13 hip | 90.00 deg | 82.18 deg | -7.82 deg |
| M12 upper | 174.99 deg | 176.13 deg | +1.14 deg |
| M11 lower | 129.55 deg | 126.12 deg | -3.43 deg |

The affine endpoint residuals were below 0.1 degree for all six contacts.

## EEPROM freeze result

| Motor | Previous Position Offset | Frozen Position Offset | Final displayed q0 |
|---|---:|---:|---:|
| M11 lower | 101 | 127 | 2046 |
| M12 upper | 859 | 851 | 2051 |
| M13 hip | -505 | -486 | 2048 |

All three motors were read back with EEPROM lock enabled and torque disabled.

## Canonical source refs

```text
main
→ current development base
→ future RF/RH/LH generalization starts here

release/matdog-lf-calibrator-v25
→ immutable exact reviewed release
→ reviewed source head: f87dd1fbc7e8100d275c74f9af448642f3429680
→ implementation PR: #11
→ post-merge CI cleanup PR: #12
```

The release branch must never be rewritten. LF V25 must not be rerun unless LF mechanics, servo, mounting, URDF or calibration state changes.

## Principal implementation files

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
tools/matdog/matdog_headless_auto_calibrate.py
tools/matdog/matdog_lf_profile.py
tools/matdog/matdog_native_observer_contract.py
```

The exact release also contains an internally named pinned-Station launcher created during development. Its historical filename is part of the byte-identical release artifact; it does **not** identify a newer validated calibration version. The validated release name remains LF V25.

## Permanent safety and architecture rules

- Station remains the sole owner of the ST3215 serial adapter during motion.
- `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- Digital-home commissioning remains separate from mechanical endpoint calibration.
- Static joints are validated by real position drift and state integrity, not by one isolated raw-speed sample.
- A bounded friction/chamfer plateau may be crossed only when a deeper coarse scout already proved that travel.
- Every contact must pass repeatability, affine/URDF consistency and supervised hardware evidence.
- EEPROM access starts only after complete measurement PASS, Station shutdown and serial release.
- EEPROM provisioning must back up, unlock, write, trigger Action, read back, relock and roll back on failure.
- No later leg may introduce per-motor exceptions to hide a general detector problem.

## CI and workflow evidence

The reviewed V25 source completed four successful release checks:

```text
30881806113  MATDOG Native Calibrator Offline Check
30881806178  MATDOG Native Observer Check
30881806060  pinned Station release artifact
30881806080  LF measurement and freeze release artifact
```

The post-merge main cleanup completed:

```text
30882331333  MATDOG Native Calibrator Offline Check
```

On `main`, only durable reusable checks remain:

```text
.github/workflows/matdog-native-calibrator-check.yml
.github/workflows/matdog-native-observer-check.yml
```

Release-only workflow definitions remain solely on `release/matdog-lf-calibrator-v25` with the exact validated source. Failed, cancelled, incomplete, duplicate and superseded runs are removed from the canonical Actions history.

## Next development milestone

The previous direct-RF milestone below is historical and is superseded by the geometry-first handoff above. Preserve it for audit context only:

```text
branch from current main
→ add data-driven RF geometry/directions/prerequisites
→ preserve LF V25 tests and evidence unchanged
→ supervised RF six-contact calibration
→ RF affine gate
→ transactional RF freeze
→ repeat individually for RH and LH
→ validate the complete twelve-joint persistent profile
```

Only one clearly named active next-milestone branch should exist at a time. Version-numbered preparation branches and copied per-leg workflows are prohibited after this cleanup.

## Historical RF handoff to Claude — 2026-08-05

The unsuccessful RF development cycle has been archived as documentation rather than merged into the active codebase.

Historical files:

```text
tools/matdog/MATDOG_RF_CALIBRATOR_CLAUDE_HANDOFF_2026-08-05.md
tools/matdog/CLAUDE_PROMPT_RF_CALIBRATOR_2026-08-05.md
tools/matdog/MATDOG_RF_CALIBRATOR_HANDOFF_STATE_2026-08-05.json
```

The archived experimental RF head is:

```text
9482086baba6fac0c266c3dc509352b6547d0365
historical PR #18
```

It is not hardware validated and must not be merged wholesale. It remains useful only for inspecting failed approaches, real-hardware regressions and the unfinished relative-span witness concept.

All RF hardware packages produced before that handoff are revoked. Future development follows `MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md`, preserves the immutable LF V25 release, and keeps all hardware blocked until the required offline gates and explicit human authorization are complete.
