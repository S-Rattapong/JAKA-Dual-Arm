#!/usr/bin/env python3
"""Validate SHA256 hashes for the archived 20-run Campaign A evidence."""
from pathlib import Path
import hashlib
import sys
base=Path(__file__).resolve().parents[1]
path=base/'docs/mq_reports/20261019_campaign_A_batch/checksums.sha256'
failed=[]
count=0
for line in path.read_text(encoding='utf-8').splitlines():
    if not line.strip(): continue
    digest,relative=line.split('  ',1)
    file=base/relative
    observed=hashlib.sha256(file.read_bytes()).hexdigest() if file.is_file() else None
    if observed!=digest: failed.append(relative)
    count+=1
print(f'Campaign A file verification: {count-len(failed)}/{count} passed')
if failed:
    print('Hash mismatches or missing files:',*failed,sep='\n - ')
    sys.exit(2)
if count!=140:
    print('Unexpected evidence file count: expected 140')
    sys.exit(2)
print('PASS — 20 runs / 140 evidence files intact')
