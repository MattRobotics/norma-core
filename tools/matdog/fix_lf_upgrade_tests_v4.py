#!/usr/bin/env python3
"""Align runner tests with the reviewed latest-only/native-thermal contract.

Run only after apply_lf_freeze_source_upgrade.py has transformed the runner.
The replacements are exact and fail closed.
"""

from pathlib import Path

path = Path(__file__).with_name("test_matdog_headless_auto_calibrate.py")
text = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected one marker, found {count}")
    text = text.replace(old, new, 1)


replace_once(
    "        self.assertEqual(len(telemetry.m11_records), 3_002)",
    '''        # Evidence is deliberately decimated: every source frame still
        # reaches the latest-sample safety contract, while the persisted M11
        # series records phase changes and at most four samples per second.
        self.assertGreaterEqual(len(telemetry.m11_records), 15)
        self.assertLessEqual(len(telemetry.m11_records), 20)''',
    "bounded M11 evidence assertion",
)

replace_once(
    "            self.assertGreaterEqual(run.max_frames_per_head_drain, 1_001)",
    '''            # LatestOnlyQueue discards superseded entries at ingestion,
            # so next_frame consumes exactly the current head, not a backlog.
            self.assertEqual(run.max_frames_per_head_drain, 1)''',
    "latest-only head-drain assertion",
)

old_thermal_test = '''    def test_superseded_thermal_fault_cannot_be_hidden_by_latest_normal(self) -> None:
        async def scenario() -> None:
            run = self.make_run()
            now_ns = runner.station_monotonic_stamp_ns()
            run.entries.put_nowait(
                make_frame(
                    now_ns - 20_000_000,
                    temperatures={12: 73},
                )
            )
            run.entries.put_nowait(make_frame(now_ns - 10_000_000))

            with self.assertRaisesRegex(
                runner.RunnerError,
                r"M12 temperature 73 C",
            ):
                await run.next_frame(
                    1.0, phase="during_calibration"
                )

        asyncio.run(scenario())
'''
new_thermal_test = '''    def test_superseded_bulk_temperature_is_replaced_by_confirmed_latest(self) -> None:
        async def scenario() -> None:
            run = self.make_run()
            now_ns = runner.station_monotonic_stamp_ns()
            run.entries.put_nowait(
                make_frame(
                    now_ns - 20_000_000,
                    temperatures={12: 73},
                )
            )
            latest = make_frame(now_ns - 10_000_000)
            run.entries.put_nowait(latest)

            observed = await run.next_frame(
                1.0, phase="during_calibration"
            )
            self.assertEqual(
                observed.motors[12].temperature_c,
                latest.motors[12].temperature_c,
            )
            self.assertEqual(run.max_frames_per_head_drain, 1)

            # A confirmed current over-temperature is still rejected. The
            # Rust driver produces that confirmed current value from dedicated
            # 0x3F reads; the Python runner validates only the latest frame.
            confirmed = make_frame(
                now_ns - 5_000_000,
                temperatures={12: 73},
            )
            with self.assertRaisesRegex(
                runner.RunnerError,
                r"M12 temperature 73 C",
            ):
                run.contract.validate_payload(confirmed)

        asyncio.run(scenario())
'''
replace_once(
    old_thermal_test,
    new_thermal_test,
    "native thermal authority test",
)

path.write_text(text)
print("MATDOG_LF_UPGRADE_TESTS_V4=PASS")
