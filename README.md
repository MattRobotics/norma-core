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

The MattRobotics fork contains a draft MATDOG-specific ST3215 mechanical-end-stop calibrator. It is intentionally isolated from `main` until hardware evidence and human review are complete.

Validated foundation through 2026-08-01:

- exact sparse 12-servo topology: `11,12,13,21,22,23,31,32,33,41,42,43`;
- Station as the only serial owner;
- unsigned standard `GoalPosition`;
- RAM-only writes limited to `TorqueEnable`, `Acc`, `GoalPosition`, `GoalSpeed` and `TorqueLimit`;
- persistent position/velocity/current contact detection;
- repeated contact, backoff, restart-safe recovery and verified global torque OFF;
- all six LF physical contacts completed in supervised hardware sessions;
- LF HIP combined MIN → HOME → MAX → HOME completed with repeatability spread zero.

### V38 development objective

Branch:

```text
matdog/full-calibration-v38
```

Explicit arm token:

```text
LF_LEG_FULL_V38
```

A single Station **Auto Calibrate** action executes:

```text
LF UPPER MIN → MAX
LF LOWER MIN → MAX
LF HIP MIN → MAX
URDF endpoint consistency check
software model-zero calculation
LF placement at the accepted calibrated q=0
verified global torque OFF
```

V38 increases the post-pilot bounded motion envelope to:

```text
TorqueLimit = 500
GoalSpeed   = 160
Acc         = 8
coarse step = 64 ticks
fine step   = 8 ticks
```

Mechanical guards, contact corridors, status checks and the hard-current abort remain unchanged.

The calibrated HOME is **not** written to EEPROM. Both endpoint-derived zero candidates must agree within 24 ticks and the resulting zero must remain within 96 ticks of the existing digital home. Otherwise the run records the contacts, reports `MODEL_ZERO_INCONSISTENT`, performs verified global torque OFF and does not apply a replacement HOME.

The V38 workflow materializes and publishes the complete Rust source for review, runs the full ST3215 test suite, builds the Station viewer and release binary, and never opens serial hardware.

## Project links

**Website:** [normacore.dev](https://normacore.dev)

- [X / Twitter](https://x.com/norma_core_dev)
- [YouTube](https://www.youtube.com/@normacoredev)
- [LinkedIn](https://www.linkedin.com/company/normacore/)
- [Reddit](https://www.reddit.com/r/NormaCore/)
- [Discord](https://discord.gg/Z4Ytw3QfHP)
- [Upstream GitHub](https://github.com/norma-core/norma-core)
