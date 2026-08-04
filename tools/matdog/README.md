# MATDOG LF calibrator — validated V25

## Status

The LF calibration flow has completed a supervised hardware run successfully.

```text
measurement sequence: 58/58 DONE
runner result: PASS
global torque OFF: verified
persistent stage: LF_STAGED
EEPROM transaction: LF_FROZEN
persistent profile: LF_FROZEN
```

The digital-home commissioning program remains separate. This calibrator measures the physical LF endpoints, evaluates them against the URDF, derives an affine joint model, stages the resulting q0 positions in RAM, stops Station and releases the serial adapter, and only then performs the explicit transactional EEPROM freeze.

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

## Canonical safety and architecture rules

- Station remains the sole owner of the ST3215 serial adapter during motion.
- `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- Non-participating and statically held joints are validated by real position drift, not by one isolated raw-speed sample.
- Fine contact search can cross a bounded repeatable friction/chamfer plateau only when the deeper coarse scout already proved that travel.
- A 16-tick global fine tracking floor admits the physically observed M13 directional settle while errors above that threshold remain fail-closed.
- Every contact must remain within the uniform supervised LF hardware-witness band.
- Affine URDF consistency and the hardware witness are hard gates; fixed encoder-scale disagreement remains diagnostic.
- EEPROM access starts only after complete measurement PASS, verified Station shutdown and serial release.
- The EEPROM provisioner backs up, unlocks, writes, triggers Action, reads back, relocks and rolls back already-modified LF offsets on transaction failure.

## Frozen source

```text
release branch: release/matdog-lf-calibrator-v25
reviewed source head: f87dd1fbc7e8100d275c74f9af448642f3429680
merged main commit: ad9fdc1e13e8eaaa67193b38a99e4d69dd3a9337
pull request: #11
```

The source branch is retained as the immutable review reference. Development for other legs must branch from the merged canonical main revision, not mutate this release branch.

## Principal files

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
tools/matdog/matdog_headless_auto_calibrate.py
tools/matdog/matdog_lf_profile.py
tools/matdog/matdog_native_observer_contract.py
tools/matdog/matdog_v42_pinned_launcher.py
```

## CI gates and post-merge policy

The reviewed V25 source passed four independent workflows:

- MATDOG Native Calibrator Offline Check;
- MATDOG Native Observer Check;
- MATDOG V42 Pinned Station Artifact;
- MATDOG LF Measurement and Freeze Artifact.

The final suite contains 135 ST3215/MATDOG Rust tests plus the runner, observer, launcher, provisioner and persistent-profile Python contracts.

After merge, only the durable source/architecture gate and observer-authority gate remain on `main`. The two release-artifact workflows are preserved on `release/matdog-lf-calibrator-v25` together with the exact reviewed source, but are removed from `main` so ordinary future PRs do not repeat redundant Station/provisioner release builds.

GitHub Actions run numbers are historical execution counters, not active workflow files. Historical runs are retained as evidence; source cleanup is performed by removing temporary workflow files from `main`, not by erasing audit history.
