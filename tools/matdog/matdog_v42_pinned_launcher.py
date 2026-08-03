#!/usr/bin/env python3
"""Immutable launcher for the reviewed MATDOG V42 Station artifact.

This module creates no register command. It verifies the reviewed headless
runner and native-authority observer byte-for-byte, installs the observer,
replaces only the Station executable SHA-256 pin, and delegates to the
fail-closed runner.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "tools/matdog/matdog_headless_auto_calibrate.py"
OBSERVER_PATH = REPO_ROOT / "tools/matdog/matdog_native_observer_contract.py"

EXPECTED_RUNNER_SHA256 = (
    "85a7a7c993d97c331882a5f8f2e6f63311eb03f310203108fc15e5d1d21206a2"
)
EXPECTED_OBSERVER_SHA256 = (
    "b9521f97ed0a3cf4d7f39d8712c2fb7a060fa56bbbc2a10b8709742d6b0a5167"
)
PINNED_STATION_SHA256 = (
    "e58c7b28c36a42d99fed8e861c2ab0689a5247b93bceb874384665c8a64b5d43"
)
PINNED_STATION_SOURCE_COMMIT = "26e743b1e62714b3d09c0fe3a6472bc9e56380b4"
PINNED_STATION_ARTIFACT_ID = 8855071283
PINNED_STATION_ARTIFACT_ZIP_SHA256 = (
    "413df5a3ad97818a3274cdeea21c6314eddf680c94c70ae0335136d5c22cf290"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_reviewed_file(path: Path, expected_sha256: str, label: str) -> None:
    actual = sha256_file(path)
    if actual != expected_sha256:
        raise RuntimeError(
            f"refusing to launch an unreviewed MATDOG {label}: "
            f"actual={actual}, expected={expected_sha256}"
        )


def load_reviewed_runner():
    require_reviewed_file(RUNNER_PATH, EXPECTED_RUNNER_SHA256, "runner")
    require_reviewed_file(OBSERVER_PATH, EXPECTED_OBSERVER_SHA256, "observer")

    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from tools.matdog import matdog_headless_auto_calibrate as runner
    from tools.matdog import matdog_native_observer_contract

    matdog_native_observer_contract.install(runner)
    runner.EXPECTED_STATION_SHA256 = PINNED_STATION_SHA256
    return runner


def main() -> int:
    runner = load_reviewed_runner()
    return runner.main()


if __name__ == "__main__":
    raise SystemExit(main())
