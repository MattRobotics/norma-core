from __future__ import annotations

import hashlib
from pathlib import Path
import re
import unittest

from tools.matdog import matdog_v42_pinned_launcher as launcher


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PinnedLauncherTests(unittest.TestCase):
    def test_runner_and_observer_are_exactly_pinned(self) -> None:
        self.assertEqual(
            sha256_file(launcher.RUNNER_PATH),
            launcher.EXPECTED_RUNNER_SHA256,
        )
        self.assertEqual(
            sha256_file(launcher.OBSERVER_PATH),
            launcher.EXPECTED_OBSERVER_SHA256,
        )

    def test_station_provenance_is_well_formed(self) -> None:
        self.assertRegex(launcher.PINNED_STATION_SHA256, r"^[0-9a-f]{64}$")
        self.assertRegex(
            launcher.PINNED_STATION_SOURCE_COMMIT,
            r"^[0-9a-f]{40}$",
        )
        self.assertTrue(launcher.PINNED_STATION_PROVENANCE)

    def test_launcher_installs_only_v25_authority_observer(self) -> None:
        runner = launcher.load_reviewed_runner()
        self.assertTrue(runner.FrameContract._matdog_native_authority_observer)
        self.assertEqual(
            runner.EXPECTED_STATION_SHA256,
            launcher.PINNED_STATION_SHA256,
        )
        runner.configure_leg("LF")
        self.assertEqual(runner.EXPECTED_FULL_TOTAL_STEPS, 58)
        runner.configure_leg("RF")
        self.assertEqual(runner.EXPECTED_FULL_TOTAL_STEPS, 58)
        self.assertEqual(runner.EXPECTED_BUS_SERIAL, "5B14114953")
        self.assertEqual(
            runner.EXPECTED_START_BODY_HEX,
            "0a0a354231343131343935338a01020801",
        )
        self.assertEqual(
            runner.EXPECTED_STOP_BODY_HEX,
            "0a0a354231343131343935339201020801",
        )


if __name__ == "__main__":
    unittest.main()
