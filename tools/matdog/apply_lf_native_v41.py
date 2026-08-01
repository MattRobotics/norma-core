#!/usr/bin/env python3
"""Materialize the V41 persistent-session LF native calibrator."""

from pathlib import Path
import base64
import gzip
import hashlib

HERE = Path(__file__).resolve().parent
PARTS = [HERE / f"apply_lf_native_v41.py.gz.b64.part{i:02d}" for i in range(2)]
EXPECTED_SHA256 = "c0135527997ba5a9499013f2bf14b7a485b5de4874bdd8b7487d0eba7a9eb172"

encoded = "".join(part.read_text(encoding="ascii").strip() for part in PARTS)
payload = gzip.decompress(base64.b64decode(encoded, validate=True))
actual = hashlib.sha256(payload).hexdigest()
if actual != EXPECTED_SHA256:
    raise SystemExit(
        f"V41 payload SHA-256 mismatch: expected={EXPECTED_SHA256}, actual={actual}"
    )

code = compile(payload, str(Path(__file__)), "exec")
exec(code, {"__name__": "__main__", "__file__": str(Path(__file__))})
