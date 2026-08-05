from pathlib import Path

SOURCE_PATH = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
TEST_PATH = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")


def fail(message: str) -> None:
    raise SystemExit(message)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        fail(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def remove_test_function(text: str, name: str) -> str:
    needle = f"fn {name}("
    idx = text.find(needle)
    if idx < 0:
        fail(f"test function not found: {name}")
    start = text.rfind("#[test]", 0, idx)
    if start < 0:
        fail(f"#[test] not found before {name}")
    brace = text.find("{", idx)
    if brace < 0:
        fail(f"opening brace not found for {name}")
    depth = 0
    end = None
    for pos in range(brace, len(text)):
        ch = text[pos]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = pos + 1
                break
    if end is None:
        fail(f"closing brace not found for {name}")
    while end < len(text) and text[end] in "\r\n":
        end += 1
    return text[:start] + text[end:]


source = SOURCE_PATH.read_text()
tests = TEST_PATH.read_text()

source = replace_once(
    source,
    '''// RF is the exact mirrored mechanism of the hardware-validated LF leg. Its
// supervised contacts must therefore reproduce the mirrored LF V25 witness,
// not merely fall inside the broad affine scale band.
const RF_MIRROR_WITNESS_TOLERANCE_TICKS: u16 = REPEATABILITY_TOLERANCE_TICKS;
// Do not accept a HOME-facing friction plateau as the RF contact. Continue the
// bounded search until at least one fine step from the exact mirrored LF V25
// endpoint. LF V25 search behavior remains unchanged.
const RF_MIRROR_SEARCH_ENTRY_TOLERANCE_TICKS: u16 = FINE_STEP_TICKS;
''',
    '''// RF is the exact mirrored mechanism of the hardware-validated LF leg.
// Mechanical symmetry constrains the joint travel span, not the absolute
// encoder coordinates. Both endpoints may translate together with the RF
// mechanical-to-encoder zero.
const RF_MIRROR_SPAN_TOLERANCE_TICKS: u16 = REPEATABILITY_TOLERANCE_TICKS;
// After the first RF contact is measured, the second search may bypass a
// HOME-facing friction plateau until it is within one fine step of the endpoint
// predicted from the immutable LF V25 measured travel span.
const RF_RELATIVE_SEARCH_ENTRY_TOLERANCE_TICKS: u16 = FINE_STEP_TICKS;
''',
    "RF constants",
)

source = replace_once(
    source,
    '''    pub(crate) baseline_target_tick: u16,
    pub(crate) allowed_motor_ids: &'static [u8],
''',
    '''    pub(crate) baseline_target_tick: u16,
    // Set only for the second RF contact. This is derived from the first
    // measured RF contact plus the immutable LF V25 travel span.
    relative_contact_entry_tick: Option<u16>,
    pub(crate) allowed_motor_ids: &'static [u8],
''',
    "ContactProfile relative entry field",
)

source = replace_once(
    source,
    '''        guard_tick,
        baseline_target_tick,
        allowed_motor_ids: leg.allowed_motor_ids(),
''',
    '''        guard_tick,
        baseline_target_tick,
        relative_contact_entry_tick: None,
        allowed_motor_ids: leg.allowed_motor_ids(),
''',
    "build_profile relative entry initialization",
)

source = replace_once(
    source,
    '''fn reference_contact_tick_for_profile(profile: &ContactProfile) -> Option<u16> {
    let (minimum, maximum) = reference_contact_ticks_for_leg(profile.leg, profile.joint)?;
    Some(match profile.side {
        ContactSide::Min => minimum,
        ContactSide::Max => maximum,
    })
}

fn rf_mirror_search_entry_tick(profile: &ContactProfile) -> Option<u16> {
    if profile.leg != Leg::Rf {
        return None;
    }
    let reference = reference_contact_tick_for_profile(profile)?;
    Some(if profile.probe_sign > 0 {
        reference.saturating_sub(RF_MIRROR_SEARCH_ENTRY_TOLERANCE_TICKS)
    } else {
        reference
            .saturating_add(RF_MIRROR_SEARCH_ENTRY_TOLERANCE_TICKS)
            .min(protocol::MAX_ANGLE_STEP)
    })
}

fn rf_home_facing_before_mirror_search_entry(profile: &ContactProfile, position: u16) -> bool {
    let Some(entry) = rf_mirror_search_entry_tick(profile) else {
        return false;
    };
    if profile.probe_sign > 0 {
        position < entry
    } else {
        position > entry
    }
}
''',
    '''fn lf_v25_reference_span_ticks(joint: JointKind) -> u16 {
    let (minimum, maximum) = lf_reference_contact_ticks(joint);
    directional_progress(maximum, minimum, spec_for(Leg::Lf, joint).direction)
}

fn rf_relative_second_contact_entry_tick(
    first_contact_tick: u16,
    second_profile: &ContactProfile,
) -> Result<Option<u16>, String> {
    if second_profile.leg != Leg::Rf {
        return Ok(None);
    }
    let reference_span = lf_v25_reference_span_ticks(second_profile.joint);
    let predicted_second = advance_tick(
        first_contact_tick,
        second_profile.probe_sign,
        reference_span,
    )?;
    if passed_guard(
        predicted_second,
        second_profile.guard_tick,
        second_profile.probe_sign,
    ) {
        return Err(format!(
            "{} relative LF V25 span predicts second contact beyond guard: first={}, span={}, predicted={}, guard={}",
            second_profile.label,
            first_contact_tick,
            reference_span,
            predicted_second,
            second_profile.guard_tick
        ));
    }
    let entry = if second_profile.probe_sign > 0 {
        predicted_second.saturating_sub(RF_RELATIVE_SEARCH_ENTRY_TOLERANCE_TICKS)
    } else {
        predicted_second
            .saturating_add(RF_RELATIVE_SEARCH_ENTRY_TOLERANCE_TICKS)
            .min(protocol::MAX_ANGLE_STEP)
    };
    Ok(Some(entry))
}

fn configure_rf_relative_second_contact_entry(
    second_profile: &mut ContactProfile,
    first_contact_tick: u16,
) -> Result<(), String> {
    second_profile.relative_contact_entry_tick =
        rf_relative_second_contact_entry_tick(first_contact_tick, second_profile)?;
    Ok(())
}

fn profile_home_facing_before_relative_search_entry(
    profile: &ContactProfile,
    position: u16,
) -> bool {
    let Some(entry) = profile.relative_contact_entry_tick else {
        return false;
    };
    if profile.probe_sign > 0 {
        position < entry
    } else {
        position > entry
    }
}
''',
    "absolute RF search-entry functions",
)

source = replace_once(
    source,
    '''    // RF is not allowed to certify an early plateau merely because it lies in
    // the broad model/affine corridor. Its mechanism and travel are the exact
    // mirror of LF V25, so the contact detector enters its acceptance region
    // only when the mirrored LF endpoint has been reached to within one fine
    // step. The existing mechanical guard remains the outer bound.
    if let Some(entry) = rf_mirror_search_entry_tick(profile) {
        if profile.probe_sign > 0 {
            low = low.max(entry);
        } else {
            high = high.min(entry);
        }
    }
''',
    '''    // Only the second RF contact receives a relative entry. The first RF
    // contact is the encoder anchor; the second must reproduce the immutable LF
    // V25 measured span. LF and standalone profiles retain their existing
    // acceptance corridors.
    if let Some(entry) = profile.relative_contact_entry_tick {
        if profile.probe_sign > 0 {
            low = low.max(entry);
        } else {
            high = high.min(entry);
        }
    }
''',
    "adaptive relative entry",
)

source = source.replace(
    "rf_home_facing_before_mirror_search_entry",
    "profile_home_facing_before_relative_search_entry",
)
source = source.replace(
    'rf_mirror_search_entry_tick(&self.profile)\n                                    .expect("RF mirror entry exists")',
    'self.profile\n                                    .relative_contact_entry_tick\n                                    .expect("RF relative entry exists")',
)
source = source.replace(
    'rf_mirror_search_entry_tick(&self.profile)\n                            .expect("RF mirror entry exists")',
    'self.profile\n                            .relative_contact_entry_tick\n                            .expect("RF relative entry exists")',
)
source = source.replace("RF mirrored-witness plateau bypass", "RF relative-span plateau bypass")
source = source.replace(
    "RF mirrored-witness tracking plateau bypass",
    "RF relative-span tracking plateau bypass",
)
source = source.replace("mirror_entry=", "relative_entry=")

source = replace_once(
    source,
    '''fn reference_contact_ticks_for_leg(leg: Leg, joint: JointKind) -> Option<(u16, u16)> {
    match leg {
        Leg::Lf => Some(lf_reference_contact_ticks(joint)),
        Leg::Rf => Some(rf_reference_contact_ticks(joint)),
        Leg::Rh | Leg::Lh => None,
    }
}

fn contact_witness_tolerance_for_leg(leg: Leg) -> Option<u16> {
    match leg {
        Leg::Lf => Some(LF_CONTACT_WITNESS_TOLERANCE_TICKS),
        Leg::Rf => Some(RF_MIRROR_WITNESS_TOLERANCE_TICKS),
        Leg::Rh | Leg::Lh => None,
    }
}

fn contact_witness_deviations_for_leg(
    leg: Leg,
    joint: JointKind,
    contacts: DualContactResult,
) -> Option<(u16, u16)> {
    let (minimum, maximum) = reference_contact_ticks_for_leg(leg, joint)?;
    Some((
        circular_distance(contact_result_tick(contacts.minimum), minimum),
        circular_distance(contact_result_tick(contacts.maximum), maximum),
    ))
}

fn lf_contact_witness_deviations(joint: JointKind, contacts: DualContactResult) -> (u16, u16) {
    contact_witness_deviations_for_leg(Leg::Lf, joint, contacts).expect("LF V25 witness exists")
}
''',
    '''fn lf_contact_witness_deviations(joint: JointKind, contacts: DualContactResult) -> (u16, u16) {
    let (minimum, maximum) = lf_reference_contact_ticks(joint);
    (
        circular_distance(contact_result_tick(contacts.minimum), minimum),
        circular_distance(contact_result_tick(contacts.maximum), maximum),
    )
}

fn rf_measured_span_ticks(joint: JointKind, contacts: DualContactResult) -> u16 {
    directional_progress(
        contact_result_tick(contacts.maximum),
        contact_result_tick(contacts.minimum),
        spec_for(Leg::Rf, joint).direction,
    )
}

fn rf_span_witness_deviation(joint: JointKind, contacts: DualContactResult) -> u16 {
    rf_measured_span_ticks(joint, contacts).abs_diff(lf_v25_reference_span_ticks(joint))
}
''',
    "absolute witness helper block",
)

source = source.replace(
    "#[cfg(test)]\nfn lf_contact_witness_accepted",
    "fn lf_contact_witness_accepted",
)

source = replace_once(
    source,
    '''fn contact_witness_accepted_for_leg(
    leg: Leg,
    joint: JointKind,
    contacts: DualContactResult,
) -> bool {
    let Some((minimum, maximum)) = contact_witness_deviations_for_leg(leg, joint, contacts) else {
        return false;
    };
    let Some(tolerance) = contact_witness_tolerance_for_leg(leg) else {
        return false;
    };
    minimum <= tolerance && maximum <= tolerance
}
''',
    '''fn contact_witness_accepted_for_leg(
    leg: Leg,
    joint: JointKind,
    contacts: DualContactResult,
) -> bool {
    match leg {
        Leg::Lf => lf_contact_witness_accepted(joint, contacts),
        Leg::Rf => {
            rf_span_witness_deviation(joint, contacts) <= RF_MIRROR_SPAN_TOLERANCE_TICKS
        }
        Leg::Rh | Leg::Lh => false,
    }
}
''',
    "relative span witness acceptance",
)

source = replace_once(
    source,
    '''            } else {
                let (minimum_deviation, maximum_deviation) = contact_witness_deviations_for_leg(
                    Leg::Rf,
                    evidence.spec.kind,
                    evidence.contacts,
                )
                .expect("RF mirrored witness exists");
                let (reference_minimum, reference_maximum) =
                    rf_reference_contact_ticks(evidence.spec.kind);
                info!(
                    "MATDOG RF MIRRORED LF V25 WITNESS: {} M{} reference_MIN={} reference_MAX={} measured_MIN={}/{} measured_MAX={}/{} deviations={}/{} tolerance={} repeatability={}/{} affine_accepted={} witness_accepted={} persistent_freeze_authorized=false",
                    evidence.spec.kind.label(),
                    evidence.spec.motor_id,
                    reference_minimum,
                    reference_maximum,
                    evidence.contacts.minimum.first_tick,
                    evidence.contacts.minimum.second_tick,
                    evidence.contacts.maximum.first_tick,
                    evidence.contacts.maximum.second_tick,
                    minimum_deviation,
                    maximum_deviation,
                    RF_MIRROR_WITNESS_TOLERANCE_TICKS,
                    evidence.contacts.minimum.spread_ticks,
                    evidence.contacts.maximum.spread_ticks,
                    evidence.affine.accepted,
                    evidence.contact_witness_accepted,
                );
            }
''',
    '''            } else {
                let reference_span = lf_v25_reference_span_ticks(evidence.spec.kind);
                let measured_span =
                    rf_measured_span_ticks(evidence.spec.kind, evidence.contacts);
                let span_deviation =
                    rf_span_witness_deviation(evidence.spec.kind, evidence.contacts);
                info!(
                    "MATDOG RF RELATIVE LF V25 SPAN WITNESS: {} M{} reference_span={} measured_span={} span_deviation={} tolerance={} measured_MIN={}/{} measured_MAX={}/{} q0_affine={} repeatability={}/{} affine_accepted={} witness_accepted={} persistent_freeze_authorized=false",
                    evidence.spec.kind.label(),
                    evidence.spec.motor_id,
                    reference_span,
                    measured_span,
                    span_deviation,
                    RF_MIRROR_SPAN_TOLERANCE_TICKS,
                    evidence.contacts.minimum.first_tick,
                    evidence.contacts.minimum.second_tick,
                    evidence.contacts.maximum.first_tick,
                    evidence.contacts.maximum.second_tick,
                    evidence.affine.estimated_zero_tick,
                    evidence.contacts.minimum.spread_ticks,
                    evidence.contacts.maximum.spread_ticks,
                    evidence.affine.accepted,
                    evidence.contact_witness_accepted,
                );
            }
''',
    "RF diagnostic logging",
)

source = replace_once(
    source,
    '''    async fn measure_v25_hip_pair_efficient(
        &mut self,
        leg: Leg,
        first_profile: ContactProfile,
        second_profile: ContactProfile,
''',
    '''    async fn measure_v25_hip_pair_efficient(
        &mut self,
        leg: Leg,
        first_profile: ContactProfile,
        mut second_profile: ContactProfile,
''',
    "mutable RF HIP second profile",
)

source = replace_once(
    source,
    '''        let first = self.measure_lf_contact_side_efficient(None).await?;

        self.stop_pressure(self.profile.motor_id, first.second_tick)
''',
    '''        let first = self.measure_lf_contact_side_efficient(None).await?;
        configure_rf_relative_second_contact_entry(&mut second_profile, first.second_tick)
            .map_err(|message| -> DynError { message.into() })?;

        self.stop_pressure(self.profile.motor_id, first.second_tick)
''',
    "configure RF HIP relative second entry",
)

source = replace_once(
    source,
    '''    async fn measure_lf_joint_pair_efficient(
        &mut self,
        minimum_profile: ContactProfile,
        maximum_profile: ContactProfile,
''',
    '''    async fn measure_lf_joint_pair_efficient(
        &mut self,
        minimum_profile: ContactProfile,
        mut maximum_profile: ContactProfile,
''',
    "mutable RF joint second profile",
)

source = replace_once(
    source,
    '''        let minimum = self.measure_lf_contact_side_efficient(None).await?;

        self.stop_pressure(self.profile.motor_id, minimum.second_tick)
''',
    '''        let minimum = self.measure_lf_contact_side_efficient(None).await?;
        configure_rf_relative_second_contact_entry(&mut maximum_profile, minimum.second_tick)
            .map_err(|message| -> DynError { message.into() })?;

        self.stop_pressure(self.profile.motor_id, minimum.second_tick)
''',
    "configure RF joint relative second entry",
)

for forbidden in (
    "RF_MIRROR_WITNESS_TOLERANCE_TICKS",
    "RF_MIRROR_SEARCH_ENTRY_TOLERANCE_TICKS",
    "reference_contact_tick_for_profile(",
    "rf_mirror_search_entry_tick(",
    "contact_witness_deviations_for_leg(",
    "contact_witness_tolerance_for_leg(",
):
    if forbidden in source:
        fail(f"forbidden absolute RF witness token remains in source: {forbidden}")

for name in (
    "rf_reference_contacts_are_exact_mirrors_of_lf_v25",
    "rf_search_does_not_accept_the_observed_home_facing_plateaus",
    "rf_affine_only_false_passes_are_rejected_by_the_mirrored_lf_witness",
    "rf_mirror_witness_tightens_only_rf_home_facing_entry_and_preserves_lf_v25",
    "rf_early_lower_plateau_is_rejected_while_lf_v25_policy_stays_unchanged",
):
    tests = remove_test_function(tests, name)

tests += r'''

#[test]
fn rf_witness_uses_lf_v25_travel_span_not_absolute_encoder_coordinates() {
    assert_eq!(lf_v25_reference_span_ticks(JointKind::Upper), 1999);
    assert_eq!(lf_v25_reference_span_ticks(JointKind::Lower), 1427);
    assert_eq!(lf_v25_reference_span_ticks(JointKind::Hip), 918);

    let translated_upper = DualContactResult {
        minimum: contact_result(2680, 2680),
        maximum: contact_result(681, 681),
    };
    assert_eq!(rf_measured_span_ticks(JointKind::Upper, translated_upper), 1999);
    assert_eq!(rf_span_witness_deviation(JointKind::Upper, translated_upper), 0);
    assert!(contact_witness_accepted_for_leg(
        Leg::Rf,
        JointKind::Upper,
        translated_upper
    ));
    let evidence = derive_leg_joint_evidence(
        Leg::Rf,
        *spec_for(Leg::Rf, JointKind::Upper),
        translated_upper,
    );
    assert!(evidence.affine.accepted);
    assert!(evidence.contact_witness_accepted);
    assert!(evidence.accepted);
    assert_ne!(evidence.affine.estimated_zero_tick, HOME_TICK);
}

#[test]
fn rf_second_contact_entry_is_anchored_to_first_measured_contact() {
    let first_contact = 2680;
    let mut second = build_profile(Leg::Rf, JointKind::Upper, ContactSide::Max).unwrap();
    assert_eq!(second.relative_contact_entry_tick, None);

    configure_rf_relative_second_contact_entry(&mut second, first_contact).unwrap();
    assert_eq!(second.relative_contact_entry_tick, Some(689));
    assert!(profile_home_facing_before_relative_search_entry(&second, 700));
    assert!(!profile_home_facing_before_relative_search_entry(&second, 689));

    let (low, high) = adaptive_contact_acceptance_bounds(&second, None);
    assert!(low <= 681);
    assert_eq!(high, 689);
}

#[test]
fn rf_relative_entry_translates_with_encoder_zero_without_changing_span() {
    let mut upper_a = build_profile(Leg::Rf, JointKind::Upper, ContactSide::Max).unwrap();
    let mut upper_b = upper_a.clone();
    configure_rf_relative_second_contact_entry(&mut upper_a, 2653).unwrap();
    configure_rf_relative_second_contact_entry(&mut upper_b, 2680).unwrap();

    assert_eq!(upper_a.relative_contact_entry_tick, Some(662));
    assert_eq!(upper_b.relative_contact_entry_tick, Some(689));
    assert_eq!(
        upper_b.relative_contact_entry_tick.unwrap()
            - upper_a.relative_contact_entry_tick.unwrap(),
        27
    );
}

#[test]
fn rf_observed_hip_plateau_is_bypassed_relative_to_first_contact() {
    let mut second = full_sequence_hip_profile(Leg::Rf, ContactSide::Min).unwrap();
    configure_rf_relative_second_contact_entry(&mut second, 1561).unwrap();
    assert_eq!(second.relative_contact_entry_tick, Some(2471));
    assert!(profile_home_facing_before_relative_search_entry(&second, 2467));
    assert!(!profile_home_facing_before_relative_search_entry(&second, 2471));

    let mut translated = full_sequence_hip_profile(Leg::Rf, ContactSide::Min).unwrap();
    configure_rf_relative_second_contact_entry(&mut translated, 1581).unwrap();
    assert_eq!(translated.relative_contact_entry_tick, Some(2491));
}

#[test]
fn rf_span_witness_rejects_short_travel_even_when_affine_band_is_broad() {
    let short_lower = DualContactResult {
        minimum: contact_result(1003, 1003),
        maximum: contact_result(2400, 2400),
    };
    let affine = derive_affine_joint_calibration(
        *spec_for(Leg::Rf, JointKind::Lower),
        short_lower,
    );
    assert!(affine.accepted);
    assert_eq!(rf_measured_span_ticks(JointKind::Lower, short_lower), 1397);
    assert_eq!(rf_span_witness_deviation(JointKind::Lower, short_lower), 30);
    assert!(!contact_witness_accepted_for_leg(
        Leg::Rf,
        JointKind::Lower,
        short_lower
    ));
}

#[test]
fn lf_v25_profiles_never_receive_rf_relative_entries() {
    for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {
        let mut maximum = build_profile(Leg::Lf, joint, ContactSide::Max).unwrap();
        configure_rf_relative_second_contact_entry(&mut maximum, 2000).unwrap();
        assert_eq!(maximum.relative_contact_entry_tick, None);
    }

    let contacts = DualContactResult {
        minimum: contact_result(1443, 1443),
        maximum: contact_result(3442, 3442),
    };
    assert!(lf_contact_witness_accepted(JointKind::Upper, contacts));
}
'''

for required in (
    "RF_MIRROR_SPAN_TOLERANCE_TICKS",
    "relative_contact_entry_tick: Option<u16>",
    "fn lf_v25_reference_span_ticks(",
    "fn rf_relative_second_contact_entry_tick(",
    "fn rf_measured_span_ticks(",
    "fn rf_span_witness_deviation(",
    "configure_rf_relative_second_contact_entry(&mut maximum_profile",
    "configure_rf_relative_second_contact_entry(&mut second_profile",
    "MATDOG RF RELATIVE LF V25 SPAN WITNESS",
):
    if required not in source:
        fail(f"required source contract missing: {required}")

for required_test in (
    "rf_witness_uses_lf_v25_travel_span_not_absolute_encoder_coordinates",
    "rf_second_contact_entry_is_anchored_to_first_measured_contact",
    "rf_relative_entry_translates_with_encoder_zero_without_changing_span",
    "rf_observed_hip_plateau_is_bypassed_relative_to_first_contact",
    "rf_span_witness_rejects_short_travel_even_when_affine_band_is_broad",
    "lf_v25_profiles_never_receive_rf_relative_entries",
):
    if required_test not in tests:
        fail(f"required test missing: {required_test}")

SOURCE_PATH.write_text(source)
TEST_PATH.write_text(tests)
print("MATDOG RF relative LF V25 span repair applied")
