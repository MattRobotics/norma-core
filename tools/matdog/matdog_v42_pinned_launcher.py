#!/usr/bin/env python3
"""Immutable launcher for the reviewed MATDOG V42 Station artifact.

This module does not create any additional command type.  It verifies the
reviewed headless runner byte-for-byte, installs the reviewed q0-first phase
contract, replaces only its Station executable SHA-256 pin, and delegates to
the existing fail-closed runner.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "tools/matdog/matdog_headless_auto_calibrate.py"

EXPECTED_RUNNER_SHA256 = (
    "85a7a7c993d97c331882a5f8f2e6f63311eb03f310203108fc15e5d1d21206a2"
)
PINNED_STATION_SHA256 = (
    "17b4da6eb46f63711a0ece52a6b71311e49afb4e639733f164f7a8e699baa1be"
)
PINNED_STATION_SOURCE_COMMIT = "b112ae4a7c6866305d3deb3b49efa3105beea528"
PINNED_STATION_ARTIFACT_ID = 8845618884
PINNED_STATION_ARTIFACT_ZIP_SHA256 = (
    "7909a85d9d4f90597cfa6733132ff6f1f3f0a678abfa10b0f8743312fa6bd341"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_reviewed_runner():
    actual = sha256_file(RUNNER_PATH)
    if actual != EXPECTED_RUNNER_SHA256:
        raise RuntimeError(
            "refusing to launch an unreviewed MATDOG runner: "
            f"actual={actual}, expected={EXPECTED_RUNNER_SHA256}"
        )

    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from tools.matdog import matdog_headless_auto_calibrate as runner
    from tools.matdog import matdog_q0_phase_contract

    matdog_q0_phase_contract.install(runner)
    runner.EXPECTED_STATION_SHA256 = PINNED_STATION_SHA256
    return runner


def main() -> int:
    runner = load_reviewed_runner()
    return runner.main()


if __name__ == "__main__":
    raise SystemExit(main())
