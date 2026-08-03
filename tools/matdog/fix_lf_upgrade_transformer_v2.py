#!/usr/bin/env python3
from pathlib import Path

path = Path(__file__).with_name("apply_lf_freeze_source_upgrade.py")
text = path.read_text()
old = '''    old_worker_args = "&mut matdog_thermal_transients,\\n                                &mut matdog_thermal_transient_total,"
    if text.count(old_worker_args) != 1:
        raise RuntimeError("worker thermal arguments: marker mismatch")
    text = text.replace(old_worker_args, "&mut matdog_thermal_state,", 1)
'''
new = '''    worker_pattern = re.compile(
        r"&mut matdog_thermal_transients,\\n\\s*&mut matdog_thermal_transient_total,"
    )
    text, worker_count = worker_pattern.subn("&mut matdog_thermal_state,", text, count=1)
    if worker_count != 1:
        raise RuntimeError(f"worker thermal arguments: expected one marker, found {worker_count}")
'''
if text.count(old) != 1:
    raise SystemExit("transformer v1 marker is not unique")
path.write_text(text.replace(old, new, 1))
print("MATDOG_LF_UPGRADE_TRANSFORMER_V2=PASS")
