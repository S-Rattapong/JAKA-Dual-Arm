"""Replay local JSONL recorder events into a new experiment directory."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from dual_arm_app.backend.experimental_infrastructure import ExperimentRecorder, ExperimentRunStore


def replay(events, output):
    now = 0
    wall = 0
    recorder = ExperimentRecorder(ExperimentRunStore(output), monotonic_ns=lambda: now,
                                  wall_time_ms=lambda: wall, manual_endpoint_clock=True)
    for event in events:
        if type(event['host_ns']) is not int or event['host_ns'] < now:
            raise ValueError('Replay events require ordered host_ns')
        now, wall = event['host_ns'], event['wall_ms']
        kind = event['event']
        data = event.get('data', {})
        if kind == 'arm':
            recorder.arm(**(data | {'artifact': SimpleNamespace(**data['artifact'])}))
        elif kind == 'bind':
            recorder.bind_execution(data)
        elif kind == 'packet':
            recorder.ingest_actual_packet(data)
        elif kind == 'observe':
            recorder.observe(data, {}, {})
        elif kind != 'tick':
            raise ValueError(f'Unknown replay event {kind}')
        # Advance the tail on event time, including the last event of a replay.
        recorder.finalize_endpoint()
    return recorder.state()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    with args.events.open() as stream:
        state = replay((json.loads(line) for line in stream if line.strip()), args.output)
    print(json.dumps(state))
    raise SystemExit(0 if state['state'] == 'CLOSED' and state['ok'] else 1)
