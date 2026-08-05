#!/usr/bin/env python3
from pathlib import Path
import re

TESTS = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")
tests = TESTS.read_text(encoding="utf-8")


def replace_once(pattern: str, replacement: str, label: str) -> None:
    global tests
    tests, count = re.subn(pattern, replacement, tests, count=1, flags=re.S)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one replacement, got {count}")


replace_once(
    r'''#\[test\]\nfn rf_profile_record_is_ram_only_and_never_authorizes_persistent_freeze\(\) \{.*?\n\}(?=\n\n#\[test\])''',
    '''#[test]
fn rf_profile_record_is_ram_only_and_never_authorizes_persistent_freeze() {
    let (minimum, maximum) = rf_reference_contact_ticks(JointKind::Hip);
    let contacts = DualContactResult {
        minimum: contact_result(minimum, minimum),
        maximum: contact_result(maximum, maximum),
    };
    let evidence = derive_leg_joint_evidence(Leg::Rf, *spec_for(Leg::Rf, JointKind::Hip), contacts);
    assert!(evidence.affine.accepted);
    assert!(evidence.contact_witness_accepted);
    assert!(evidence.accepted);
    let record = leg_machine_profile_record(Leg::Rf, evidence);
    assert!(record.starts_with("MATDOG_RF_PROFILE_V1|"));
    assert!(record.contains("motor_id=23"));
    assert!(record.contains("persistent_freeze_authorized=false"));
    assert!(!record.contains("EEPROM"));
}''',
    "replace obsolete RF profile-record fixture",
)

replace_once(
    r'''#\[test\]\nfn coarse_home_side_acceptance_is_v25_symmetric_for_lf_m11_and_rf_m21\(\) \{.*?\n\}(?=\n\n#\[test\])''',
    '''#[test]
fn rf_mirror_witness_tightens_only_rf_home_facing_entry_and_preserves_lf_v25() {
    let lf = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    let rf = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();

    assert_eq!(contact_acceptance_bounds(&lf), (1557, 1685));
    assert_eq!(contact_acceptance_bounds(&rf), (2411, 2539));
    assert_eq!(adaptive_contact_acceptance_bounds(&lf, None), (1557, 1717));
    assert_eq!(adaptive_contact_acceptance_bounds(&rf, None), (2422, 2539));

    assert!((1557..=1717).contains(&1697));
    assert!(!(1557..=1717).contains(&1718));
    assert!((2422..=2539).contains(&2430));
    assert!(!(2422..=2539).contains(&2399));
    assert!(!(2422..=2539).contains(&2421));
    assert_eq!(lf.guard_tick, 1557);
    assert_eq!(rf.guard_tick, 2539);
}''',
    "replace obsolete RF adaptive-corridor expectation",
)

replace_once(
    r'''#\[test\]\nfn real_rf_m21_max_coarse_trace_and_lf_mirror_are_contacts_under_v25_policy\(\) \{.*?\n\}(?=\n\n#\[test\])''',
    '''#[test]
fn rf_early_lower_plateau_is_rejected_while_lf_v25_policy_stays_unchanged() {
    let baseline = BaselineStats {
        median_current: 1,
        mad_current: 0,
    };

    let rf = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();
    let mut rf_detector =
        HybridContactDetector::new_for_profile_with_scout(HOME_TICK, baseline, &rf, None);
    let rf_sample = observation(2399, 0, 48, 2439);
    assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::FreeMotion);
    }
    assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::ContactSuspected);
    assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::ContactSuspected);
    assert_eq!(rf_detector.observe(rf_sample, 2439), ContactState::EarlyStall);

    let lf = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    let mut lf_detector =
        HybridContactDetector::new_for_profile_with_scout(HOME_TICK, baseline, &lf, None);
    let lf_sample = observation(1697, 0, 48, 1657);
    assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::FreeMotion);
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::FreeMotion);
    }
    assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::ContactSuspected);
    assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::ContactSuspected);
    assert_eq!(lf_detector.observe(lf_sample, 1657), ContactState::ContactConfirmed);
}''',
    "replace obsolete RF false-contact detector expectation",
)

TESTS.write_text(tests, encoding="utf-8")
print("RF mirrored-witness regression expectations updated")
