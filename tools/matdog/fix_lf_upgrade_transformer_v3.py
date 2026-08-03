#!/usr/bin/env python3
from pathlib import Path

path = Path(__file__).with_name("apply_lf_freeze_source_upgrade.py")
text = path.read_text()

replacements = {
    "dual_contact(1446, 1441, 3443, 3442)": "DualContactResult {\n                minimum: contact_result(1446, 1441),\n                maximum: contact_result(3443, 3442),\n            }",
    "dual_contact(3132, 3135, 1640, 1643)": "DualContactResult {\n                minimum: contact_result(3132, 3135),\n                maximum: contact_result(1640, 1643),\n            }",
    "dual_contact(2546, 2544, 1545, 1547)": "DualContactResult {\n                minimum: contact_result(2546, 2544),\n                maximum: contact_result(1545, 1547),\n            }",
}

for old, new in replacements.items():
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"historical contact fixture {old}: expected one marker, found {count}")
    text = text.replace(old, new, 1)

path.write_text(text)
print("MATDOG_LF_UPGRADE_TRANSFORMER_V3=PASS")
