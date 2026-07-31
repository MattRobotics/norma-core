#!/usr/bin/env python3
"""Normalize the one V20 Rust string literal escaped by the text transformer."""

from pathlib import Path
import sys


BAD = (
    'const HIP_HARDWARE_BLOCK_REASON: &str = \\\"HIP hardware is blocked until '
    'ordered UPPER MIN/MAX and LOWER MIN/MAX phase proof is verified\\\";'
)
GOOD = (
    'const HIP_HARDWARE_BLOCK_REASON: &str = "HIP hardware is blocked until '
    'ordered UPPER MIN/MAX and LOWER MIN/MAX phase proof is verified";'
)


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: fix_ordered_sequence_v20_quote.py MATDOG_RS")
    path = Path(sys.argv[1])
    text = path.read_text(encoding="utf-8")
    bad_count = text.count(BAD)
    good_count = text.count(GOOD)
    if bad_count != 1 or good_count != 0:
        raise SystemExit(
            f"unexpected V20 quote state: bad_count={bad_count}, good_count={good_count}"
        )
    text = text.replace(BAD, GOOD, 1)
    if text.count(BAD) != 0 or text.count(GOOD) != 1:
        raise SystemExit("V20 quote normalization postcondition failed")
    path.write_text(text, encoding="utf-8")
    print("V20 Rust quote normalization: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
