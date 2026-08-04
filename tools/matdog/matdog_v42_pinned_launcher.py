#!/usr/bin/env python3
"""Pinned MATDOG LF/RF native-calibrator launcher.

The historical filename is retained from LF V25. This module creates no
register command and contains no motion policy. It verifies the reviewed runner
and the byte-identical LF V25 authority observer, installs that observer once,
sets only the exact Station executable SHA-256 pin, and delegates to the
fail-closed runner.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "tools/matdog/matdog_headless_auto_calibrate.py"
OBSERVER_PATH = REPO_ROOT / "tools/matdog/matdog_native_observer_contract.py"

EXPECTED_RUNNER_SHA256 = "ab83fad9b93cf7786834cfe54264023178e30922b332411a32000d6193a5eaa0"
EXPECTED_OBSERVER_SHA256 = "b9521f97ed0a3cf4d7f39d8712c2fb7a060fa56bbbc2a10b8709742d6b0a5167"
PINNED_STATION_SHA256 = "ee2494668a06758480461a6b1f140d7120b2ab2f4ec6dd44a89ba4feefeeb053"
PINNED_STATION_SOURCE_COMMIT = "bf878fb70499029ee433bd4a4eaaae6183c854ff"
PINNED_STATION_PROVENANCE = "LOCAL_ASUS_WARNING_FREE_BUILD:/home/matteo-manicardi/MATDOG/_archive/verification-artifacts/RF_V25_CLEAN_PORT_20260804T200037Z/pinned-station-bf878fb70499029ee433bd4a4eaaae6183c854ff/bin/station"


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
