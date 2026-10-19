#!/usr/bin/env python3
"""Verify immutable original EXP4 Loaded Experiment data across 11 runs."""
import hashlib
from pathlib import Path
import sys

base = Path(__file__).resolve().parents[1]
list_path = base / "docs/mq_reports/20261019_exp4_loaded_batch/checksums.sha256"
mismatches = []
count = 0
for line in list_path.read_text(encoding="utf-8").splitlines():
    if not line.strip():
        continue
    digest, relative = line.split("  ", 1)
    path = base / relative
    actual = None
    if path.is_file():
        hasher = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b""):
                hasher.update(chunk)
        actual = hasher.hexdigest()
    if actual != digest:
        mismatches.append(relative)
    count += 1
print(f"EXP4 original-file SHA256 verification: {count-len(mismatches)}/{count} passed")
if mismatches:
    for path in mismatches:
        print("MISMATCH OR MISSING:", path)
    sys.exit(2)
if count != 77:
    print("Unexpected file count; expected 77 original files")
    sys.exit(2)
print("PASS — 11 runs / 77 original evidence files intact")
