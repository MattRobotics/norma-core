from pathlib import Path

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
path.write_text(text.replace(old, new, 1))
print("RF relative-span error conversion fixed")
