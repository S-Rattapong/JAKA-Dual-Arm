"""Sample Linux host CPU, memory and optional process I/O counters to JSONL."""
import argparse
import json
import os
from pathlib import Path
import time


def counters(path):
    return {parts[0].rstrip(':'): int(parts[1]) for line in path.read_text().splitlines()
            if len(parts := line.split()) >= 2 and parts[1].isdigit()}


def sample(proc, pid):
    start = time.monotonic_ns()
    cpu = (proc/'stat').read_text().splitlines()[0].split()[1:9]
    mem = counters(proc/'meminfo')
    row = dict(schema_version='exp2-linux-proc/v1', sample_begin_ns=start,
               host_wall_ms=time.time_ns()//1_000_000, cpu_ticks=list(map(int, cpu)),
               cpu_tick_hz=os.sysconf('SC_CLK_TCK'),
               mem_total_kib=mem['MemTotal'], mem_available_kib=mem.get('MemAvailable'),
               io_pid=pid)
    try:
        row['disk_sectors'] = {
            fields[2]: {'read': int(fields[5]), 'written': int(fields[9])}
            for line in (proc/'diskstats').read_text().splitlines() if len(fields := line.split()) >= 14
        }
    except OSError as err:
        row.update(disk_sectors=None, disk_error=str(err))
    try:
        row['process_io'] = counters(proc/str(pid)/'io')
    except OSError as err:
        row.update(process_io=None, io_error=str(err))
    row['sample_end_ns'] = time.monotonic_ns()
    return row


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pid', type=int, default=os.getpid())
    parser.add_argument('--samples', type=int, default=60)
    parser.add_argument('--interval-s', type=float, default=1)
    args = parser.parse_args()
    if args.samples < 1 or not 0 < args.interval_s < float('inf'):
        parser.error('samples and interval must be positive and finite')
    with args.output.open('x') as output:
        for index in range(args.samples):
            if index:
                time.sleep(args.interval_s)
            print(json.dumps(sample(Path('/proc'), args.pid)), file=output, flush=True)
