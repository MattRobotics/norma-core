# NormaCore

### The Unified Toolkit for Physical System Development & Operations

**NormaCore** is a unified toolkit for developing and operating physical systems. It combines a unified API, high-performance data pipelines and visual tooling for robotics, distributed sensors and research systems.

Developer experience sits at the heart of the project. NormaCore includes complete hardware and software systems such as the **ElRobot 7+1 DoF arm**, the parallel-jaw gripper and **Station**, the real-time control and data platform.

## What's inside

| Project | Path | Description |
|---|---|---|
| **ElRobot** | [`hardware/elrobot/`](hardware/elrobot/) | Fully 3D-printed 7+1 DoF robotic arm for imitation learning |
| **Parallel Jaw Gripper** | [`hardware/pgripper/`](hardware/pgripper/) | Modular gripper for the SO-101 arm |
| **Station** | [`software/station/bin/station/`](software/station/bin/station/) | Real-time robotics platform: data collection, inference, control and web UI |
| **ST3215 driver** | [`software/drivers/st3215/`](software/drivers/st3215/) | Bus ownership, telemetry, RAM control and native calibration support |
| **SmolVLA fine-tune** | [`software/ai/smolvla_py/`](software/ai/smolvla_py/) | Train and deploy a SmolVLA policy on SO-101 |
| **Gremlin** | [`shared/gremlin_go/`](shared/gremlin_go/) · [`shared/gremlin_py/`](shared/gremlin_py/) | High-performance Protobuf SDKs |

## MATDOG native calibration integration

The MattRobotics fork contains a draft MATDOG-specific native ST3215 mechanical-end-stop calibrator. It remains isolated from `main` until supervised hardware checkpoints and human review are complete.

Validated foundation through 2026-08-01:

- exact sparse 12-servo topology: `11,12,13,21,22,23,31,32,33,41,42,43`;
- Station as the only serial owner;
- unsigned standard `GoalPosition`;
- RAM-only writes limited to `TorqueEnable`, `Acc`, `GoalPosition`, `GoalSpeed` and `TorqueLimit`;
- persistent position/velocity/current contact detection;
- repeated contact, backoff, restart-safe recovery and verified global torque OFF;
- all six LF physical contacts completed in supervised hardware sessions;
- LF HIP combined MIN → HOME → MAX → HOME completed `Done 20/20` with spread zero;
- both-endpoint URDF model-zero solver verified independently;
- one-click LF V38 and LH V39 programs verified offline;
- complete 24-contact, 12-joint V40 orchestration verified offline.

## Current staged programs

### LF V38

```text
branch: matdog/full-calibration-v38
arm token: LF_LEG_FULL_V38
CI: 100/100 ST3215 tests PASS
```

One Auto Calibrate performs LF UPPER/LOWER/HIP MIN+MAX, validates both URDF endpoints, derives three software q=0 targets and places LF at calibrated HOME only when every gate passes.

### LH V39

```text
branch: matdog/lh-full-calibration-v39
arm token: LH_LEG_FULL_V39
CI: 104/104 ST3215 tests PASS
```

One Auto Calibrate performs the same full cycle for LH. The canonical exact-mesh checkpoint proves that active rear legs require no additional front-leg parking. LH HIP uses M42 at 3072 and M41 at 3038.

### Complete V40

```text
branch: matdog/all-legs-full-calibration-v40
arm token: MATDOG_ALL_LEGS_FULL_V40
CI: 108/108 ST3215 tests PASS
```

A single Auto Calibrate action executes:

```text
LF → RF → RH → LH
24 repeated mechanical contacts
four per-leg model-zero gates
12-joint calibrated software HOME
verified global torque OFF
```

A leg must pass before the next leg starts. During acquisition, every stage returns to the historical digital HOME. The 12 new q=0 targets are applied only after all four legs pass, in the order all HIP joints → all UPPER joints → all LOWER joints.

## Model-zero contract

Encoder scale and direction remain fixed. Each joint derives two candidates:

```text
zero_from_MIN = measured_MIN - direction × URDF_MIN_delta
zero_from_MAX = measured_MAX - direction × URDF_MAX_delta
```

Acceptance requires:

```text
endpoint disagreement <= 24 ticks
estimated q=0 shift from 2048 <= 96 ticks
```

A failed gate reports `MODEL_ZERO_INCONSISTENT`, records the contacts, does not apply calibrated HOME and performs verified global torque OFF.

## Motion and safety envelope

```text
TorqueLimit = 500
GoalSpeed   = 160
Acc         = 8
coarse step = 64 ticks
fine step   = 8 ticks
hard-current abort = 200
```

Mechanical guards, contact corridors, status checks and driver-error aborts remain active. The faster envelope addresses under-seating observed during the first LF HIP MAX pilot without increasing permitted travel.

The calibrated HOME is software-level. The native calibrator does **not** write EEPROM, Position Offset, LOCK, reset, ResetCalibration, RegWrite, Action, Save or Freeze.

## Offline verification

V40 workflow:

```text
MATDOG All Legs Full Calibration V40
run 30696228454
result: PASS
108/108 ST3215 tests
Station viewer build: PASS
Station release build: PASS
hardware_started=false
serial_opened=false
```

V40 artifact:

```text
id: 8817381318
digest: sha256:e6868d89cdd2473485b4469fc804f1091cf96587b365aa7c84228953db595ede
matdog.rs: a955f7de9a1c3405cf4d4e705d545e499162ba3cb378261bb9ca7afcf53999b7
matdog_test.rs: 23d60c377ee40b8f71c8969989b70e8098b0ae7126392d81af4e631f347ee696
Station: a5d2bd00ad90ad3c4fc3268f52a847dce390a040042d51a7633de89b7b70ff9c
```

Hardware execution of V40 remains gated by successful supervised LF V38 and LH V39 model-zero checkpoints.

## Project links

**Website:** [normacore.dev](https://normacore.dev)

- [X / Twitter](https://x.com/norma_core_dev)
- [YouTube](https://www.youtube.com/@normacoredev)
- [LinkedIn](https://www.linkedin.com/company/normacore/)
- [Reddit](https://www.reddit.com/r/NormaCore/)
- [Discord](https://discord.gg/Z4Ytw3QfHP)
- [Upstream GitHub](https://github.com/norma-core/norma-core)
