#!/usr/bin/env python3
"""Apply the existing V25 32-tick HOME-side scout band to coarse detection too."""

from pathlib import Path

SOURCE = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TESTS = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")

source = SOURCE.read_text(encoding="utf-8")
tests = TESTS.read_text(encoding="utf-8")

old = '''fn adaptive_contact_acceptance_bounds(
    profile: &ContactProfile,
    coarse_scout_tick: Option<u16>,
) -> (u16, u16) {
    let (mut low, mut high) = contact_acceptance_bounds(profile);
    if let Some(scout) = coarse_scout_tick {
        // The coarse pass is allowed to discover an earlier real stop on the
        // HOME-facing side of the model corridor. Never extend beyond the
        // mechanical guard; extend only away from it by a bounded amount.
        if profile.probe_sign > 0 {
            low = low.min(scout.saturating_sub(ADAPTIVE_FINE_SCOUT_TICKS));
        } else {
            high = high.max(
                scout
                    .saturating_add(ADAPTIVE_FINE_SCOUT_TICKS)
                    .min(protocol::MAX_ANGLE_STEP),
            );
        }
    }
    (low, high)
}
'''

new = '''fn adaptive_contact_acceptance_bounds(
    profile: &ContactProfile,
    coarse_scout_tick: Option<u16>,
) -> (u16, u16) {
    let (mut low, mut high) = contact_acceptance_bounds(profile);
    // V25 permits the coarse scout to discover an earlier real stop on the
    // HOME-facing side of the model corridor by the same bounded 32-tick band
    // later used to anchor the fine passes. With no scout yet, anchor that band
    // at the model corridor's HOME-facing edge. Never extend toward or beyond
    // the mechanical guard.
    let home_facing_anchor = coarse_scout_tick.unwrap_or(if profile.probe_sign > 0 {
        low
    } else {
        high
    });
    if profile.probe_sign > 0 {
        low = low.min(home_facing_anchor.saturating_sub(ADAPTIVE_FINE_SCOUT_TICKS));
    } else {
        high = high.max(
            home_facing_anchor
                .saturating_add(ADAPTIVE_FINE_SCOUT_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        );
    }
    (low, high)
}
'''

if new in source:
    print("V25 coarse HOME-side acceptance already applied")
elif source.count(old) != 1:
    raise SystemExit(f"expected exactly one adaptive acceptance function, found {source.count(old)}")
else:
    source = source.replace(old, new, 1)

marker = "fn coarse_home_side_acceptance_is_v25_symmetric_for_lf_m11_and_rf_m21()"
if marker not in tests:
    tests += r'''

#[test]
fn coarse_home_side_acceptance_is_v25_symmetric_for_lf_m11_and_rf_m21() {
    let lf = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    let rf = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();

    assert_eq!(contact_acceptance_bounds(&lf), (1557, 1685));
    assert_eq!(contact_acceptance_bounds(&rf), (2411, 2539));
    assert_eq!(
        adaptive_contact_acceptance_bounds(&lf, None),
        (1557, 1717)
    );
    assert_eq!(
        adaptive_contact_acceptance_bounds(&rf, None),
        (2379, 2539)
    );

    // Exact RF hardware stop and its LF encoder mirror are both admitted.
    assert!((2379..=2539).contains(&2399));
    assert!((1557..=1717).contains(&1697));

    // The band remains exactly 32 ticks and does not grow toward the guard.
    assert!(!(2379..=2539).contains(&2378));
    assert!(!(1557..=1717).contains(&1718));
    assert_eq!(lf.guard_tick, 1557);
    assert_eq!(rf.guard_tick, 2539);
}

#[test]
fn real_rf_m21_max_coarse_trace_and_lf_mirror_are_contacts_under_v25_policy() {
    let baseline = BaselineStats {
        median_current: 1,
        mad_current: 0,
    };

    let rf = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();
    let mut rf_detector = HybridContactDetector::new_for_profile_with_scout(
        HOME_TICK,
        baseline,
        &rf,
        None,
    );
    let rf_sample = observation(2399, 0, 48, 2439);
    assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::FreeMotion);
    }
    assert_eq!(
        rf_detector.observe(rf_sample, 2439),
        ContactState::ContactSuspected
    );
    assert_eq!(
        rf_detector.observe(rf_sample, 2439),
        ContactState::ContactSuspected
    );
    assert_eq!(
        rf_detector.observe(rf_sample, 2439),
        ContactState::ContactConfirmed
    );

    let lf = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    let mut lf_detector = HybridContactDetector::new_for_profile_with_scout(
        HOME_TICK,
        baseline,
        &lf,
        None,
    );
    let lf_sample = observation(1697, 0, 48, 1657);
    assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::FreeMotion);
    }
    assert_eq!(
        lf_detector.observe(lf_sample, 1657),
        ContactState::ContactSuspected
    );
    assert_eq!(
        lf_detector.observe(lf_sample, 1657),
        ContactState::ContactSuspected
    );
    assert_eq!(
        lf_detector.observe(lf_sample, 1657),
        ContactState::ContactConfirmed
    );
}

#[test]
fn coarse_stall_beyond_the_existing_v25_home_side_band_still_fails_closed() {
    let baseline = BaselineStats {
        median_current: 1,
        mad_current: 0,
    };
    let rf = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();
    let mut detector = HybridContactDetector::new_for_profile_with_scout(
        HOME_TICK,
        baseline,
        &rf,
        None,
    );
    let sample = observation(2378, 0, 48, 2439);
    assert_eq!(detector.observe(sample, 2439), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(detector.observe(sample, 2439), ContactState::FreeMotion);
    }
    assert_eq!(detector.observe(sample, 2439), ContactState::ContactSuspected);
    assert_eq!(detector.observe(sample, 2439), ContactState::ContactSuspected);
    assert_eq!(detector.observe(sample, 2439), ContactState::EarlyStall);
}
'''

SOURCE.write_text(source, encoding="utf-8")
TESTS.write_text(tests, encoding="utf-8")
print("V25 coarse HOME-side acceptance patch applied")
