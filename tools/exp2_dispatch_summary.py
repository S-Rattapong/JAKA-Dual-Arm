"""Print per-run terminal host-driver diagnostics from saved raw JSON."""
import argparse
import json
from pathlib import Path


def summarize(raw):
    summary = raw.get('endpoint_evidence', {}).get('driver_dispatch_summary')
    if summary is None:
        summary = {'arms': {side: {'status': 'UNAVAILABLE'} for side in ('left', 'right')}}
    return {'run_id': raw.get('run_id'), **summary}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw', type=Path)
    args = parser.parse_args()
    with args.raw.open(encoding='utf-8') as stream:
        print(json.dumps(summarize(json.load(stream)), indent=2))
