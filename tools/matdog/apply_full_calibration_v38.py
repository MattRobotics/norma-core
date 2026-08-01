#!/usr/bin/env python3
"""Verified loader for the reviewable V38 materialization artifact.

The compressed Base64 payload is split into small ordered parts so GitHub's
contents API cannot truncate a large single-line file. The loader requires the
exact six parts, concatenates them, verifies the SHA-256 of the decompressed
Python transformation source and only then executes it. CI publishes the full
materialized Rust sources as a normal review artifact.
"""

from __future__ import annotations

import base64
import gzip
import hashlib
from pathlib import Path

EXPECTED_SOURCE_SHA256 = "d668315a6bba3dd1904726542239967cae339f963f49123145e7dd5ac45f2f2e"
EXPECTED_PART_COUNT = 6
PART_GLOB = Path(__file__).name + ".gz.b64.part*"


def load_encoded_payload() -> str:
    parts = sorted(Path(__file__).parent.glob(PART_GLOB))
    if len(parts) != EXPECTED_PART_COUNT:
        raise SystemExit(
            f"V38 payload part count mismatch: expected={EXPECTED_PART_COUNT} actual={len(parts)}"
        )
    expected_names = [
        f"{Path(__file__).name}.gz.b64.part{index:02d}"
        for index in range(EXPECTED_PART_COUNT)
    ]
    actual_names = [part.name for part in parts]
    if actual_names != expected_names:
        raise SystemExit(
            f"V38 payload part names mismatch: expected={expected_names} actual={actual_names}"
        )
    return "".join(part.read_text(encoding="ascii").strip() for part in parts)


def main() -> None:
    encoded = load_encoded_payload()
    source = gzip.decompress(base64.b64decode(encoded, validate=True))
    actual = hashlib.sha256(source).hexdigest()
    if actual != EXPECTED_SOURCE_SHA256:
        raise SystemExit(
            f"V38 transformation source hash mismatch: expected={EXPECTED_SOURCE_SHA256} actual={actual}"
        )
    code = compile(source, str(Path(__file__).resolve()), "exec")
    namespace = {
        "__name__": "__main__",
        "__file__": str(Path(__file__).resolve()),
        "__package__": None,
    }
    exec(code, namespace, namespace)


if __name__ == "__main__":
    main()
