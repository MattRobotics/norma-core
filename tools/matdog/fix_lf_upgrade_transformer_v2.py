#!/usr/bin/env python3
from pathlib import Path

path = Path(__file__).with_name("apply_lf_freeze_source_upgrade.py")
text = path.read_text()

old_worker = '''    old_worker_args = "&mut matdog_thermal_transients,\\n                                &mut matdog_thermal_transient_total,"
    if text.count(old_worker_args) != 1:
        raise RuntimeError("worker thermal arguments: marker mismatch")
    text = text.replace(old_worker_args, "&mut matdog_thermal_state,", 1)
'''
new_worker = '''    worker_pattern = re.compile(
        r"&mut matdog_thermal_transients,\\n\\s*&mut matdog_thermal_transient_total,"
    )
    text, worker_count = worker_pattern.subn("&mut matdog_thermal_state,", text, count=1)
    if worker_count != 1:
        raise RuntimeError(f"worker thermal arguments: expected one marker, found {worker_count}")
'''
if text.count(old_worker) != 1:
    raise SystemExit("transformer worker marker is not unique")
text = text.replace(old_worker, new_worker, 1)

old_emission = '''    text = replace_once(
        text,
        '            info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));',
        '            info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));\\n            info!("{}", lf_machine_profile_record(evidence));',
        "machine profile emission",
    )
'''
new_emission = '''    old_profile_emission = ''' + "'''" + '''            match cleanup {
                Ok(()) => {
                    for evidence in outcome.joints {
                        info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));
                    }
''' + "'''" + '''
    new_profile_emission = ''' + "'''" + '''            match cleanup {
                Ok(()) => {
                    for evidence in outcome.joints {
                        info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));
                        info!("{}", lf_machine_profile_record(evidence));
                    }
''' + "'''" + '''
    text = replace_once(
        text,
        old_profile_emission,
        new_profile_emission,
        "machine profile emission after verified cleanup",
    )
'''
if text.count(old_emission) != 1:
    raise SystemExit("transformer profile-emission marker is not unique")
text = text.replace(old_emission, new_emission, 1)

path.write_text(text)
print("MATDOG_LF_UPGRADE_TRANSFORMER_V2=PASS")
