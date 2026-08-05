from pathlib import Path


def remove_function(text: str, name: str) -> str:
    needle = f"fn {name}("
    start = text.find(needle)
    if start < 0:
        raise SystemExit(f"obsolete function not found: {name}")
    brace = text.find("{", start)
    if brace < 0:
        raise SystemExit(f"opening brace not found: {name}")
    depth = 0
    end = None
    for pos in range(brace, len(text)):
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
            if depth == 0:
                end = pos + 1
                break
    if end is None:
        raise SystemExit(f"closing brace not found: {name}")
    while end < len(text) and text[end] in "\r\n":
        end += 1
    return text[:start] + text[end:]


source_path = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
source = source_path.read_text()
old = '''    let predicted_second = advance_tick(
        first_contact_tick,
        second_profile.probe_sign,
        reference_span,
    )?;
'''
new = '''    let predicted_second = advance_tick(
        first_contact_tick,
        second_profile.probe_sign,
        reference_span,
    )
    .map_err(|error| error.to_string())?;
'''
count = source.count(old)
if count != 1:
    raise SystemExit(f"expected one advance_tick error-conversion site, found {count}")
source = source.replace(old, new, 1)
source = remove_function(source, "mirror_tick_about_digital_home")
source = remove_function(source, "rf_reference_contact_ticks")
for forbidden in ("fn mirror_tick_about_digital_home(", "fn rf_reference_contact_ticks("):
    if forbidden in source:
        raise SystemExit(f"obsolete absolute witness helper remains: {forbidden}")
source_path.write_text(source)

test_path = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")
tests = test_path.read_text()
old_test = '''fn rf_profile_record_is_ram_only_and_never_authorizes_persistent_freeze() {
    let (minimum, maximum) = rf_reference_contact_ticks(JointKind::Hip);
    let contacts = DualContactResult {
        minimum: contact_result(minimum, minimum),
        maximum: contact_result(maximum, maximum),
    };
'''
new_test = '''fn rf_profile_record_is_ram_only_and_never_authorizes_persistent_freeze() {
    // Both RF endpoints are translated +20 ticks from the former digital-home
    // mirror, while the immutable LF V25 physical span remains exactly 918.
    let contacts = DualContactResult {
        minimum: contact_result(2499, 2499),
        maximum: contact_result(1581, 1581),
    };
    assert_eq!(
        rf_measured_span_ticks(JointKind::Hip, contacts),
        lf_v25_reference_span_ticks(JointKind::Hip)
    );
'''
count = tests.count(old_test)
if count != 1:
    raise SystemExit(f"expected one absolute RF profile-record test, found {count}")
tests = tests.replace(old_test, new_test, 1)
if "rf_reference_contact_ticks(" in tests:
    raise SystemExit("test suite still references absolute RF endpoint helper")
test_path.write_text(tests)
print("RF relative-span compile correction, absolute-helper cleanup and test migration applied")
