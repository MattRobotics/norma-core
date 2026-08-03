#!/usr/bin/env python3
from pathlib import Path
import re

path = Path(__file__).with_name("apply_lf_freeze_source_upgrade.py")
text = path.read_text()

old_helper = '''def replace_test_function(text: str, name: str, replacement: str) -> str:
    pattern = re.compile(
        rf"(?ms)^    #\\[test\\]\\n    fn {re.escape(name)}\\(\\) \\{{.*?^    \\}}\\n"
    )
    updated, count = pattern.subn(replacement.rstrip() + "\\n", text)
    if count != 1:
        raise RuntimeError(f"test {name}: expected one function, found {count}")
    return updated
'''
new_helper = '''def replace_test_function(text: str, name: str, replacement: str) -> str:
    pattern = re.compile(
        rf"(?ms)^(?P<indent>[ \\t]*)#\\[test\\]\\n(?P=indent)fn {re.escape(name)}\\(\\) \\{{.*?^(?P=indent)\\}}\\n"
    )
    updated, count = pattern.subn(replacement.rstrip() + "\\n", text)
    if count != 1:
        raise RuntimeError(f"test {name}: expected one function, found {count}")
    return updated
'''
if text.count(old_helper) != 1:
    raise SystemExit("transformer test-function helper marker is not unique")
text = text.replace(old_helper, new_helper, 1)

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

old_scan = '        THERMAL_IMPL + "    async fn scan_motors(",'
new_scan = '        THERMAL_IMPL,'
if text.count(old_scan) != 1:
    raise SystemExit("transformer scan-motors replacement marker is not unique")
text = text.replace(old_scan, new_scan, 1)

immediate_pattern = re.compile(
    r"^const MATDOG_IMMEDIATE_THERMAL_ABORT_C: u8 = 85;\n",
    re.MULTILINE,
)
text, immediate_count = immediate_pattern.subn("", text, count=1)
if immediate_count != 1:
    raise SystemExit(
        f"transformer immediate thermal constant: expected one marker, found {immediate_count}"
    )

immediate_classifier = '''    if samples
        .iter()
        .any(|temperature| *temperature >= MATDOG_IMMEDIATE_THERMAL_ABORT_C)
    {
        return MatdogThermalDecision::Confirmed;
    }
'''
if text.count(immediate_classifier) != 1:
    raise SystemExit("transformer immediate thermal classifier marker is not unique")
text = text.replace(immediate_classifier, "", 1)

old_confirmation_condition = (
    "        if first > configured_limit_c && first < MATDOG_IMMEDIATE_THERMAL_ABORT_C {"
)
new_confirmation_condition = "        if first > configured_limit_c {"
if text.count(old_confirmation_condition) != 1:
    raise SystemExit("transformer thermal confirmation condition is not unique")
text = text.replace(old_confirmation_condition, new_confirmation_condition, 1)

old_direct_test = '''        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[85]),
            MatdogThermalDecision::Confirmed
        );
        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[73, 72, 39]),
            MatdogThermalDecision::Confirmed
        );
'''
new_direct_test = '''        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[85]),
            MatdogThermalDecision::Transient
        );
        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[85, 84, 39]),
            MatdogThermalDecision::Confirmed
        );
        assert_eq!(
            classify_matdog_direct_temperature_samples(70, &[73, 72, 39]),
            MatdogThermalDecision::Confirmed
        );
'''
if text.count(old_direct_test) != 1:
    raise SystemExit("transformer direct-temperature test marker is not unique")
text = text.replace(old_direct_test, new_direct_test, 1)

path.write_text(text)
print("MATDOG_LF_UPGRADE_TRANSFORMER_V2=PASS")
