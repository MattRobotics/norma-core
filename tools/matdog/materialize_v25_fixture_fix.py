#!/usr/bin/env python3
"""Align V25 end-to-end fixtures with the supervised LF hardware witness."""

from pathlib import Path

path = Path("software/drivers/st3215/src/auto_calibrate/matdog_test.rs")
text = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one marker, found {count}")
    text = text.replace(old, new, 1)


helper_marker = """fn model_consistent_contacts(joint: JointKind) -> DualContactResult {
"""
helper = """fn supervised_lf_witness_contacts(joint: JointKind) -> DualContactResult {
    let contact = |first_tick: u16, second_tick: u16| ContactResult {
        coarse_scout_tick: first_tick,
        first_tick,
        second_tick,
        spread_ticks: circular_distance(first_tick, second_tick),
        baseline: BaselineStats {
            median_current: 1,
            mad_current: 0,
        },
    };
    match joint {
        JointKind::Upper => DualContactResult {
            minimum: contact(1442, 1444),
            maximum: contact(3441, 3443),
        },
        JointKind::Lower => DualContactResult {
            minimum: contact(3092, 3094),
            maximum: contact(1665, 1667),
        },
        JointKind::Hip => DualContactResult {
            minimum: contact(2534, 2536),
            maximum: contact(1616, 1618),
        },
    }
}

fn model_consistent_contacts(joint: JointKind) -> DualContactResult {
"""
if "fn supervised_lf_witness_contacts(" not in text:
    replace_once(helper_marker, helper, "supervised witness helper")

function_start = text.index("fn lf_state_machine_runs_the_full_simulated_path_with_runtime_roles()")
function_end = text.index("\n#[test]\nfn production_snapshot_verifier", function_start)
section = text[function_start:function_end]
old = "let contacts = model_consistent_contacts(joint);"
if section.count(old) != 1:
    raise SystemExit(f"state-machine contact fixture: expected one marker, found {section.count(old)}")
section = section.replace(old, "let contacts = supervised_lf_witness_contacts(joint);", 1)
# The production code stages affine q0. Keep the state-machine replay aligned.
section = section.replace(".fixed_scale.estimated_zero_tick", ".affine.estimated_zero_tick")
text = text[:function_start] + section + text[function_end:]

historical_start = text.index("fn historical_contacts_use_affine_and_uniform_witness_freeze_gate()")
historical_end = text.index("\n#[test]\nfn degree_diagnostics", historical_start)
historical = text[historical_start:historical_end]
first_statement = historical.index("    let upper =")
assertions = historical.index("    for evidence in")
replacement_body = """    let upper = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Upper),
        supervised_lf_witness_contacts(JointKind::Upper),
    );
    let lower = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Lower),
        supervised_lf_witness_contacts(JointKind::Lower),
    );
    let hip = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Hip),
        supervised_lf_witness_contacts(JointKind::Hip),
    );
"""
historical = historical[:first_statement] + replacement_body + historical[assertions:]
text = text[:historical_start] + historical + text[historical_end:]

for token in (
    "supervised_lf_witness_contacts(joint)",
    "supervised_lf_witness_contacts(JointKind::Upper)",
    "evidences[0].affine.estimated_zero_tick",
):
    if token not in text:
        raise SystemExit(f"required V25 fixture token missing: {token}")

path.write_text(text)
