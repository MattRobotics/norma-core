#!/usr/bin/env python3
"""Verified loader for the reviewable V38 materialization artifact.

The companion payload is gzip-compressed only to keep the GitHub contents API
commit compact. CI expands it, verifies the exact SHA-256 of the Python source,
then executes it with this file path so repository-root resolution remains
stable. The workflow publishes the complete materialized Rust sources as a
normal review artifact.
"""

from __future__ import annotations

import base64
import gzip
import hashlib
from pathlib import Path

EXPECTED_SOURCE_SHA256 = "d668315a6bba3dd1904726542239967cae339f963f49123145e7dd5ac45f2f2e"
PAYLOAD = Path(__file__).with_suffix(".py.gz.b64")


def main() -> None:
    encoded = PAYLOAD.read_text(encoding="ascii").strip()
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
