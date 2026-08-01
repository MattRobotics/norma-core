#!/usr/bin/env python3
"""Carry the V38 two-fine repeatability correction into V40."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS = ROOT / "software/drivers/st3215/src/auto_calibrate/matdog_test.rs"


def replace_exact(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    tests = TESTS.read_text(encoding="utf-8")

    old_progress = "calibrator.total_steps = 14;"
    count = source.count(old_progress)
    if count != 2:
        raise SystemExit(f"contact-stage progress totals: expected exactly two matches, found {count}")
    source = source.replace(old_progress, "calibrator.total_steps = 16;")

    old = '''        self.next_phase("Coarse approach")?;
        let first_tick = self.approach(COARSE_STEP_TICKS, baseline).await?;

        self.next_phase("Backoff and verify recovery")?;
        self.backoff_and_verify(first_tick, baseline).await?;

        self.next_phase("Fine repeat approach")?;
        let second_tick = self.approach(FINE_STEP_TICKS, baseline).await?;

        self.next_phase("Verify repeatability")?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)?;
'''
    new = '''        self.next_phase("Coarse scouting approach — measurement discarded")?;
        let coarse_scout_tick = self.approach(COARSE_STEP_TICKS, baseline).await?;

        self.next_phase("Backoff after coarse scout")?;
        self.backoff_and_verify(coarse_scout_tick, baseline).await?;

        self.next_phase("First fine metrology approach")?;
        let first_tick = self.approach(FINE_STEP_TICKS, baseline).await?;

        self.next_phase("Backoff between identical fine approaches")?;
        self.backoff_and_verify(first_tick, baseline).await?;

        self.next_phase("Second fine metrology approach")?;
        let second_tick = self.approach(FINE_STEP_TICKS, baseline).await?;

        self.next_phase("Verify fine-to-fine repeatability")?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)?;
'''
    source = replace_exact(source, old, new, "two-fine sequence")

    anchor = '''#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
'''
    injected = r'''#[test]
fn v38_repeatability_compares_two_identical_fine_approaches_not_the_coarse_scout() {
    let source = include_str!("matdog.rs");
    let start = source.find("async fn run(&mut self) -> Result<ContactResult, DynError>").unwrap();
    let end = source[start..].find("async fn inspect_profile_entry").map(|offset| start + offset).unwrap();
    let body = &source[start..end];
    assert!(body.contains("let coarse_scout_tick = self.approach(COARSE_STEP_TICKS"));
    assert_eq!(body.matches("self.approach(FINE_STEP_TICKS, baseline).await?").count(), 2);
    assert!(body.contains("repeatability_spread(first_tick, second_tick)"));
    assert!(!body.contains("repeatability_spread(coarse_scout_tick"));
    assert!(body.contains("backoff_and_verify(first_tick, baseline)"));
}

#[test]
fn v38_2026_08_01_failure_is_coarse_loading_not_mechanical_change() {
    assert_eq!(circular_distance(1416, 1440), 24);
    assert!(repeatability_spread(1416, 1440).is_err());
    assert_eq!(circular_distance(1440, 1443), 3);
    assert!(repeatability_spread(1440, 1443).is_ok());
}

#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
'''
    tests = replace_exact(tests, anchor, injected, "regression tests")

    SOURCE.write_text(source, encoding="utf-8")
    TESTS.write_text(tests, encoding="utf-8")


if __name__ == "__main__":
    main()
