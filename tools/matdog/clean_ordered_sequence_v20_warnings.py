#!/usr/bin/env python3
"""Remove the exact V20-only dead code reported by Rust after redesign."""

from pathlib import Path
import sys


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: count={count}, expected=1")
    return text.replace(old, new, 1)


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: clean_ordered_sequence_v20_warnings.py MATDOG_RS")
    path = Path(sys.argv[1])
    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "const UPPER_50_DELTA: i16 = 569;\n",
        "",
        "remove obsolete UPPER_50_DELTA",
    )
    text = replace_once(
        text,
        "fn position_inside_contact_acceptance(\n",
        "#[cfg(test)]\nfn position_inside_contact_acceptance(\n",
        "scope acceptance helper to tests",
    )
    text = replace_once(
        text,
        "    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {\n",
        "    #[cfg(test)]\n"
        "    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {\n",
        "scope legacy detector constructor to tests",
    )

    forbidden = (
        "const UPPER_50_DELTA: i16 = 569;",
        "fn position_inside_contact_acceptance(\n",
        "    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {",
    )
    if text.count("#[cfg(test)]\nfn position_inside_contact_acceptance(") != 1:
        raise SystemExit("acceptance helper test scope postcondition failed")
    if text.count(
        "#[cfg(test)]\n    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {"
    ) != 1:
        raise SystemExit("legacy constructor test scope postcondition failed")
    if forbidden[0] in text:
        raise SystemExit("obsolete UPPER_50_DELTA remained")

    path.write_text(text, encoding="utf-8")
    print("V20 dead-code cleanup: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
