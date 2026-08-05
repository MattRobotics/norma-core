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


path = Path("software/drivers/st3215/src/auto_calibrate/matdog.rs")
text = path.read_text()
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
count = text.count(old)
if count != 1:
    raise SystemExit(f"expected one advance_tick error-conversion site, found {count}")
text = text.replace(old, new, 1)
text = remove_function(text, "mirror_tick_about_digital_home")
text = remove_function(text, "rf_reference_contact_ticks")
for forbidden in ("fn mirror_tick_about_digital_home(", "fn rf_reference_contact_ticks("):
    if forbidden in text:
        raise SystemExit(f"obsolete absolute witness helper remains: {forbidden}")
path.write_text(text)
print("RF relative-span compile correction and absolute-helper cleanup applied")
