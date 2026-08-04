# NormaCore

### The Unified Toolkit for Physical System Development & Operations

**NormaCore** is a unified toolkit designed to facilitate the development and deployment of physical systems. From complex robotics to distributed sensor networks and hobby projects, the system provides a solid foundation to manage them all. To achieve this goal, the platform combines a unified API, high-performance data pipelines, and visual tooling to help you build and manage your entire ecosystem as one.

**Developer experience sits at the heart of our design philosophy.**

To fully realize the potential of this approach, we had to build a lot from scratch, rethinking traditional solutions from a practical perspective. This includes not just software, but complete hardware systems like our **7+1 DoF robotic arm** with a **parallel jaw gripper** - tools designed to open up a whole new dimension of applications for home and research robotics without significant cost or investment.

## MattRobotics fork — MATDOG status

This fork contains the Station/ST3215 integration used by the custom MATDOG quadruped.

### Current validation boundary

```text
LF V25: hardware validated, affine profile saved and EEPROM frozen
RF: not yet mechanically hardware validated
RH: not yet mechanically hardware validated
LH: not yet mechanically hardware validated
complete all-leg persistent calibration: not yet validated
```

The only mechanically hardware-validated calibrator is **MATDOG LF V25**. Earlier V28–V42 and “all legs” branches were development experiments and are not valid programs or development bases.

### Canonical refs

```text
main
→ only active development base

release/matdog-lf-calibrator-v25
→ immutable exact LF V25 hardware-validated source
→ reviewed source head: f87dd1fbc7e8100d275c74f9af448642f3429680
```

Future RF/RH/LH development must branch from current `main`, preserve the LF V25 tests and evidence, and generalize through data-driven leg profiles. The release branch must never be rewritten.

Operational and verification details are documented in [`tools/matdog/README.md`](tools/matdog/README.md).

## What's inside

| Project | Path | Description |
|---|---|---|
| **ElRobot** | [`hardware/elrobot/`](hardware/elrobot/) | Fully 3D-printed 7+1 DoF robotic arm for imitation learning |
| **Parallel Jaw Gripper** | [`hardware/pgripper/`](hardware/pgripper/) | Modular gripper for the SO-101 arm |
| **Station** | [`software/station/bin/station/`](software/station/bin/station/) | Real-time robotics platform — data collection, inference, control. Single binary, web UI |
| **MATDOG LF V25 calibrator** | [`tools/matdog/`](tools/matdog/) | Hardware-validated LF end-stop measurement, affine q0 staging, transactional EEPROM freeze and persistent profile |
| **SmolVLA fine-tune** | [`software/ai/smolvla_py/`](software/ai/smolvla_py/) | Train + deploy a [SmolVLA](https://huggingface.co/docs/lerobot/smolvla) policy on the SO-101 arm |
| **Gremlin** | [`shared/gremlin_go/`](shared/gremlin_go/) · [`shared/gremlin_py/`](shared/gremlin_py/) | High-performance Protobuf SDK for Go and Python — used across the station + drivers stack |

## MATDOG repository policy

- `main` contains the canonical reusable implementation and durable CI only.
- `release/matdog-lf-calibrator-v25` preserves the exact reviewed LF release.
- No V28–V42 or per-joint preparation branch is retained as an operational choice.
- At most one clearly named active branch may exist for the next hardware milestone.
- One-shot/version-numbered workflows must be removed after their successful release closeout.
- Failed, cancelled, incomplete, duplicate and superseded workflow runs are not part of the canonical evidence set.

**Website:** [normacore.dev](https://normacore.dev)

**Follow us:**
- 🐦 [X/Twitter](https://x.com/norma_core_dev)
- 🎥 [YouTube](https://www.youtube.com/@normacoredev)
- 💼 [LinkedIn](https://www.linkedin.com/company/normacore/)
- 📢 [Reddit](https://www.reddit.com/r/NormaCore/)

**Join & Contribute:**
- 💬 [Discord](https://discord.gg/Z4Ytw3QfHP) - Chat with the community
- 🐙 [GitHub](https://github.com/norma-core/norma-core) - Source code & issues
