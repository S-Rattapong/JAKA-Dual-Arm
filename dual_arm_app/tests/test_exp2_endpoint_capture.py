"""Synthetic host clocks and in-process recorder. No transport or driver imports."""
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

from dual_arm_app.backend import experimental_infrastructure as infra
from dual_arm_app.tests.test_exp0_experimental_infrastructure import artifact, report
from tools.exp2_host_endpoints import analyze, classify

SIDES = ('left', 'right')
BIND = dict(trajectory_id='test', start_time_unix_ns=2_000_000_000, duration_s=.2,
            artifact_fingerprint='artifact-exp0', artifact_generation=7)
TERMINAL = dict(execution={'trajectory_id': 'test'}, driver_feedback={
    'trajectory_id': 'test', 'authoritative': True, 'terminal': True, 'combined_state': 'COMPLETED'})


def arm_args(enabled=True):
    return dict(artifact=artifact(), phase4_report=report(), phase4_generation=7,
                artifact_generation=7, metadata={'endpoint_evidence': {}} if enabled else {})


def simulate(root, *, enabled=True, right_delay=30, right_period=10, poll=20, noisy=False, terminal=TERMINAL):
    ms = 0
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(root), monotonic_ns=lambda: ms*10**6,
                                   wall_time_ms=lambda: ms)
    rec.arm(**arm_args(enabled))
    for ms in range(3251):
        if ms == 2000:
            rec.bind_execution(BIND)
        if ms == 2200:
            rec.observe(terminal, {side: dict(valid=True, joint=[1]*6, received_at_ms=ms)
                                   for side in SIDES}, {})
            if enabled:
                assert rec.state()['state'] == 'FINALIZING'
                assert rec.state()['terminal_status'] == 'COMPLETED'
        for side, period in zip(SIDES, (5, right_period)):
            if ms % period:
                continue
            start = 2000 + (right_delay if side == 'right' else 0)
            q = min(1, max(0, (ms-start)/(2180-start))) * (2 if side == 'left' else -2)
            if noisy and ms == 1500:
                q = .02
            rec.ingest_actual_packet(dict(side=side, received_at_ms=ms, received_monotonic_ns=ms*10**6,
                                          packet_sequence_index=ms//period, actual_joints_rad=[q]*6,
                                          controller_joint_position_rad=[q+.01]*6))
        if ms >= 2200 and ms % poll == 0:
            rec.observe({}, {}, {})
    rec.finalize_endpoint()
    assert rec.state()['state'] == 'CLOSED'
    return rec.store.root / rec.state()['run_id']


@pytest.mark.parametrize('delay,period,name', [(30,10,'RIGHT_LATER'), (0,5,'INCLUDES_ZERO')])
def test_complete_and_poll_independence(tmp_path, delay, period, name):
    outputs = []
    for poll in (20, 43):
        run = simulate(tmp_path/str(poll), right_delay=delay, right_period=period, poll=poll)
        raw = json.loads((run/'raw.json').read_text())
        stream = raw['endpoint_evidence']
        assert stream['outcome'] == 'SETTLED'
        assert stream['packets']['left'][0]['received_monotonic_ns'] == 10**9
        assert stream['packets']['left'][-1]['received_monotonic_ns'] >= 3_200_000_000
        result = analyze(raw)
        assert result['start_classification'] == name
        assert result['stop_classification'] == 'INCLUDES_ZERO'
        lo, hi = result['start_R_minus_L_host_ms']
        assert lo <= delay <= hi
        outputs.append((lo, hi, result['stop_R_minus_L_host_ms']))
    assert outputs[0] == outputs[1]


def test_legacy_products_identical(tmp_path, monkeypatch):
    monkeypatch.setattr(infra, '_utc_iso', lambda *args: '2026-01-01T00:00:00Z')
    monkeypatch.setattr(infra.ExperimentRunStore, 'generate_run_id',
                        lambda self: 'exp-20260101T000000000Z-12345678')
    drivers = {}
    for count, side in enumerate(SIDES, 4):
        drivers[side] = dict(valid=True, available=True, trajectory_id='test',
            dispatch_timing=dict(sample_count=count, mean_abs_lateness_ms=.2, p95_abs_lateness_ms=.5,
                                 max_abs_lateness_ms=.6, missed_cycle_count=1),
            servo_j_call_duration=dict(sample_count=count, overrun_count=1, mean_ms=2, p95_ms=3, max_ms=4),
            servo_stream=dict(command_period_ms=24, first_dispatch_unix_ns=1900000000000000001,
                              last_dispatch_unix_ns=1900000000096000001, first_servo_return_unix_ns=1900000000002000001,
                              last_servo_return_unix_ns=1900000000098000001))
    terminal = TERMINAL | {'driver_feedback': TERMINAL['driver_feedback'] | {'drivers': drivers}}
    old, new = [simulate(tmp_path/str(flag), enabled=flag, terminal=terminal) for flag in (False, True)]
    a, b = [json.loads((root/'raw.json').read_bytes()) for root in (old, new)]
    summary = b.pop('endpoint_evidence')['driver_dispatch_summary']
    for side in SIDES:
        assert summary['arms'][side] == {'status': 'AVAILABLE', **{
            k: drivers[side][k] for k in ('dispatch_timing', 'servo_j_call_duration', 'servo_stream')}}
    for run in (old, new):
        result = json.loads(subprocess.check_output([sys.executable, '-m', 'tools.exp2_dispatch_summary',
                                                      run/'raw.json']))
        assert result['run_id'] == a['run_id']
        assert result['arms'] == (summary['arms'] if run == new else
                                  {s: {'status': 'UNAVAILABLE'} for s in SIDES})
    assert a == b
    for name in ('manifest.json','aligned.json','observed.csv','planned.csv','exp1_analysis.json'):
        assert (old/name).read_bytes() == (new/name).read_bytes(), name


    before, after = [json.loads((root/'exp2_analysis.json').read_bytes()) for root in (old, new)]
    assert before['input_provenance'].pop('raw_sha256') != after['input_provenance'].pop('raw_sha256')
    assert before == after
    assert json.loads((old/'exp1_analysis.json').read_bytes())['arms']['left']['joints'][0]['rmse_rad'] > 0
    assert before['same_packet_tracking']['sides']['left']['overall_rad']['sample_count'] > 0


def test_bad_evidence(tmp_path):
    run = simulate(tmp_path, noisy=True)
    raw = json.loads((run/'raw.json').read_bytes())
    stream = raw['endpoint_evidence']
    for side in SIDES:
        assert 'BASELINE_OUTSIDE_THRESHOLD' in analyze(raw)['arms'][side]['reasons']
    rows = stream['packets']['right']
    for row in rows:
        if row['received_monotonic_ns'] == 1_500_000_000:
            row['actual_joints_rad'] = [0]*6
    rows[1]['received_monotonic_ns'] = rows[0]['received_monotonic_ns']
    rows[1]['packet_sequence_index'] = 0
    assert {'LOCAL_SEQUENCE_DISCONTINUITY','DUPLICATE_OR_REVERSED_CLOCK'} <= set(analyze(raw)['arms']['right']['reasons'])
    stream['packets']['right'] = [p for p in rows[2:] if p['received_monotonic_ns'] < 2_600_000_000]
    assert analyze(raw)['stop_classification'] == 'INCOMPLETE'
    del raw['endpoint_evidence']
    assert analyze(raw)['stop_classification'] == 'INCOMPLETE'
    assert classify([-35,-25]) == 'RIGHT_EARLIER'


def test_timeout_and_no_callbacks_during_persistence(tmp_path, monkeypatch):
    now = 0
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path), monotonic_ns=lambda: now)
    rec.arm(**arm_args())
    rec.bind_execution(BIND)
    rec.observe(TERMINAL, {}, {})
    with pytest.raises(infra.ExperimentError, match='already armed'):
        rec.arm(**arm_args())
    now = 4_999_999_999
    rec.finalize_endpoint()
    assert rec.state()['state'] == 'FINALIZING'
    now += 1
    entered, release = threading.Event(), threading.Event()
    original = infra.align_planned_to_actual

    def slow_align(raw):
        assert (json.loads((tmp_path/raw['run_id']/'raw.json').read_bytes())['endpoint_evidence']
                ['outcome'] == 'TIMEOUT')
        entered.set()
        assert release.wait(3)
        return original(raw)

    monkeypatch.setattr(infra, 'align_planned_to_actual', slow_align)
    thread = threading.Thread(target=rec.finalize_endpoint)
    thread.start()
    try:
        assert entered.wait(3)
        rec.ingest_actual_packet(dict(side='left', received_at_ms=0, received_monotonic_ns=now, joints_rad=[0]*6))
    finally:
        release.set()
        thread.join(3)
    assert not thread.is_alive()
    rec.finalize_endpoint()
    assert rec.state()['terminal_status'] == 'COMPLETED'
    assert 'timed out' in rec.state()['error']
    assert rec.store.load(rec.state()['run_id'])['raw']['endpoint_evidence']['packets'] == {s:[] for s in SIDES}


def test_replay_and_exports(tmp_path):
    args = arm_args()
    args['artifact'] = vars(args['artifact'])
    events = [('arm',args,0), ('bind',BIND,2000), ('observe',TERMINAL,2200),
              ('packet',dict(side='right',received_monotonic_ns=8_000_000_000,received_at_ms=100,
                             joints_rad=[0]*6),8000), ('tick',{},9000)]
    source = tmp_path/'events.jsonl'
    source.write_text(''.join(json.dumps(dict(event=k,data=d,host_ns=t*10**6,wall_ms=t))+'\n'
                              for k,d,t in events))
    def cli(module, *args):
        return subprocess.run([sys.executable, '-m', 'tools.'+module, *args],
                              capture_output=True, text=True)
    r = cli('exp2_endpoint_replay',source,tmp_path/'runs')
    assert r.returncode == 1, r.stderr
    state = json.loads(r.stdout)
    assert state['state'] == 'CLOSED' and 'timed out' in state['error']
    path = tmp_path/'runs'/state['run_id']/'raw.json'
    assert json.loads(path.read_bytes())['endpoint_evidence']['packets']['right'] == []
    csv = tmp_path/'samples.csv'
    r = cli('exp2_host_endpoints', path, '--samples-csv', csv)
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout)['start_classification'] == 'INCOMPLETE'
    assert 'actual_joint_6_rad' in csv.read_text()
    host = tmp_path/'perf.jsonl'
    r = cli('exp2_host_performance',host,'--samples','1')
    assert r.returncode == 0, r.stderr
    perf = json.loads(host.read_bytes())
    assert perf['mem_total_kib'] > 0 and perf['sample_begin_ns'] <= perf['sample_end_ns']


def test_tail_extends_then_freezes_through_write_retry(tmp_path, monkeypatch):
    now = 0
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path), monotonic_ns=lambda: now)
    rec.arm(**arm_args())
    rec.bind_execution(BIND)
    rec.observe(TERMINAL, {}, {})
    def packet(side, q):
        rec.ingest_actual_packet(dict(side=side, joints_rad=[q]*6, received_at_ms=0,
                                      received_monotonic_ns=now))
    for ms in range(0, 1350, 50):
        now = ms*10**6
        packet('left', 2)
        packet('right', -2 if ms >= 1200 else -1)
        rec.finalize_endpoint()
        assert rec.state()['state'] == 'FINALIZING'
    now = 1350*10**6
    packet('left', 2)
    with monkeypatch.context() as m:
        def fail(*args):
            raise OSError('injected disk error')
        m.setattr(rec.store, 'update', fail)
        worker = rec._endpoint_worker
        packet('right', -2)
        worker.join(3)
        assert not worker.is_alive()
    assert 'injected disk error' in rec.state()['error']
    packet('right', 0)
    rec.finalize_endpoint()
    raw = rec.store.load(rec.state()['run_id'])['raw']
    assert raw['endpoint_evidence']['outcome'] == 'SETTLED'
    assert raw['endpoint_evidence']['packets']['right'][-1]['actual_joints_rad'] == [-2]*6
    assert rec.state()['ok']


def test_gap_without_local_sequence_and_reversed_sign(tmp_path):
    run = simulate(tmp_path)
    raw = json.loads((run/'raw.json').read_bytes())
    stream = raw['endpoint_evidence']
    rows = stream['packets']['left']
    for row in rows:
        row['packet_sequence_index'] = None
    stream['packets']['left'] = [row for row in rows if not 1.8e9 < row['received_monotonic_ns'] < 2.1e9]
    result = analyze(raw)
    assert 'RECEIVE_GAP' in result['arms']['left']['reasons']
    assert result['start_classification'] == result['stop_classification'] == 'INCOMPLETE'
    stream['packets']['left'] = stream['packets']['right']
    stream['packets']['right'] = rows
    for sample in raw['planned']['samples']:
        sample['left'], sample['right'] = sample['right'], sample['left']
    assert analyze(raw)['start_classification'] == 'RIGHT_EARLIER'


def test_stationary_or_disarmed_never_qualifies_motion(tmp_path):
    run = simulate(tmp_path)
    raw = json.loads((run/'raw.json').read_bytes())
    for rows in raw['endpoint_evidence']['packets'].values():
        for row in rows:
            row['actual_joints_rad'] = [0]*6
    assert analyze(raw)['start_classification'] == analyze(raw)['stop_classification'] == 'INCOMPLETE'
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path))
    rec.arm(**arm_args())
    state = rec.disarm()
    assert rec.store.load(state['run_id'])['raw']['endpoint_evidence']['outcome'] == 'DISARMED'


def test_authoritative_terminal_wall_step_and_sparse_finalization(tmp_path):
    now = 100
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path), monotonic_ns=lambda: now,
                                   wall_time_ms=lambda: 500)
    opts = arm_args()
    opts['metadata']['endpoint_evidence']['post_ms'] = 1500
    rec.arm(**opts)
    rec.bind_execution(BIND)
    rec.observe(TERMINAL | {'driver_feedback': TERMINAL['driver_feedback'] | {'authoritative': False}}, {}, {})
    assert rec.state()['state'] == 'RECORDING'
    terminal = TERMINAL | {'driver_feedback': TERMINAL['driver_feedback'] | {'combined_state': 'ABORTED'}}
    rec.observe(terminal, {}, {})
    for i in range(20):
        now = 100 + i*100_000_000
        for side, q in zip(SIDES,(2,-2)):
            rec.ingest_actual_packet(dict(side=side, joints_rad=[q]*6, received_at_ms=1000-i*10,
                                          received_monotonic_ns=now))
    now += 7_000_000_000
    rec.finalize_endpoint()
    raw = rec.store.load(rec.state()['run_id'])['raw']
    assert raw['result']['status'] == 'ABORTED'
    assert raw['actual_packets'] == {s: [] for s in SIDES}
    evidence = raw['endpoint_evidence']
    assert evidence['outcome'] == 'SETTLED'
    assert evidence['settled_observed_ns'] == 100 + 1_500_000_000
    assert evidence['receive_gaps_since_arm']['left']['count'] == 15


@pytest.mark.parametrize('options', [True, {'post_ms':999}, {'pre_ms':0}, {'timeout_ms':900},
                                      {'hold_ms':float('nan')}, {'unknown':10}])
def test_invalid_policy(tmp_path, options):
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path))
    args = arm_args()
    args['metadata']['endpoint_evidence'] = options
    with pytest.raises(infra.ExperimentError):
        rec.arm(**args)
    assert rec.state()['state'] == 'IDLE'


def test_browser_arm_to_request_model_contract():
    # Inspect the actual entry point without importing ROS or serving HTTP.
    import ast
    root = Path(__file__).resolve().parents[2]
    backend = (root/'dual_arm_app/backend/dual_jaka_web_backend.py').read_text()
    request = next(node for node in ast.parse(backend).body
                   if isinstance(node, ast.ClassDef) and node.name == 'ExperimentArmRequest')
    field = next(node for node in request.body if isinstance(node, ast.AnnAssign)
                 and node.target.id == 'endpoint_evidence')
    assert ast.literal_eval(field.value) is False
    assert 'metadata=request_payload' in backend
    js = (root/'dual_arm_app/web/digital_twin_experiment.js').read_text()
    assert 'element("experimentEndpointEvidence")?.checked ? { endpoint_evidence: {} }' in js
    assert 'id="experimentEndpointEvidence" type="checkbox" />' in (root/'dual_arm_app/web/index.html').read_text()


def test_cli_successful_replay_and_late_false_motion(tmp_path):
    args = arm_args()
    args['artifact'] = vars(args['artifact'])
    events = []

    def event(t, kind, data):
        events.append(dict(host_ns=t*10**6, wall_ms=t, event=kind, data=data))

    event(0, 'arm', args)
    for t in range(0, 3250, 50):
        if t == 2000:
            event(t, 'bind', BIND)
        if t == 2200:
            event(t, 'observe', TERMINAL)
        for side in SIDES:
            # The only motion starts AFTER the authoritative terminal observation.
            q = (2 if side == 'left' else -2) if t >= 2300 else 0
            event(t, 'packet', dict(side=side, joints_rad=[q]*6, received_at_ms=t,
                                    received_monotonic_ns=t*10**6))
    path = tmp_path/'events.jsonl'
    path.write_text('\n'.join(map(json.dumps, events)))
    state = json.loads(subprocess.check_output([sys.executable, '-m', 'tools.exp2_endpoint_replay',
                                                path, tmp_path/'runs']))
    assert state['state'] == 'CLOSED' and state['ok']
    raw = json.loads((tmp_path/'runs'/state['run_id']/'raw.json').read_bytes())
    result = analyze(raw)
    assert result['start_classification'] == result['stop_classification'] == 'INCOMPLETE'
    assert 'START_AFTER_TERMINAL' in result['arms']['left']['reasons']


@pytest.mark.parametrize('settles', [True, False])
def test_no_poll_real_clock(tmp_path, monkeypatch, settles):
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path))
    args = arm_args()
    args['metadata']['endpoint_evidence']['timeout_ms'] = 5000 if settles else 1000
    state = rec.arm(**args)
    assert rec._endpoint_worker is None

    def packet(side, q):
        rec.ingest_actual_packet(dict(side=side, joints_rad=[q]*6, received_at_ms=2000,
                                      received_monotonic_ns=time.monotonic_ns()))

    for side in SIDES:
        packet(side, 0)
    rec.bind_execution(BIND)
    assert rec._endpoint_worker is None
    writing, release = threading.Event(), threading.Event()
    update = rec.store.update

    def slow_write(*args):
        writing.set()
        assert release.wait(3)
        return update(*args)

    monkeypatch.setattr(rec.store, 'update', slow_write)
    rec.observe(TERMINAL, {}, {})
    worker = rec._endpoint_worker
    try:
        if settles:
            until = time.monotonic() + 2.5
            while not writing.is_set() and time.monotonic() < until:
                for side, goal in zip(SIDES, (2, -2)):
                    packet(side, goal)
                writing.wait(.04)
        assert writing.wait(2)
        packet('left', 99)  # Must return while store.update is waiting for release.
    finally:
        release.set()
        worker.join(3)
    assert not worker.is_alive()
    assert rec.state()['state'] == 'CLOSED'
    ev = rec.store.load(state['run_id'])['raw']['endpoint_evidence']
    assert ev['outcome'] == ('SETTLED' if settles else 'TIMEOUT')
    assert ev['driver_dispatch_summary']['arms'] == {s: {'status': 'UNAVAILABLE'} for s in SIDES}
    for side in SIDES:
        rows = ev['packets'][side]
        assert rows[0]['received_monotonic_ns'] <= ev['bind_observed_ns']
        if settles:
            assert rows[-1]['received_monotonic_ns'] >= ev['terminal_observed_ns'] + 10**9
        else:
            assert len(rows) == 1
        assert all(row['joints_rad'][0] != 99 for row in rows)


def test_new_arm_waits_for_previous_workers_write(tmp_path, monkeypatch):
    now = 0
    rec = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path), monotonic_ns=lambda: now)
    first = rec.arm(**arm_args())
    capture = rec._endpoint
    rec.bind_execution(BIND)
    rec.observe(TERMINAL, {}, {})
    worker = rec._endpoint_worker
    writing, release, arming = (threading.Event() for _ in range(3))
    original = rec.store.update

    def slow_update(*args):
        writing.set()
        assert release.wait(3)
        return original(*args)

    monkeypatch.setattr(rec.store, 'update', slow_update)
    now = capture.deadline
    capture.wake_finalizer.set()
    second = {}

    def arm_again():
        arming.set()
        second.update(rec.arm(**arm_args()))

    assert writing.wait(3)
    contender = threading.Thread(target=arm_again)
    contender.start()
    try:
        assert arming.wait(3)
        assert rec._endpoint is capture
    finally:
        release.set()
        worker.join(3)
        contender.join(3)
    assert second['state'] == 'ARMED'
    assert second['run_id'] != first['run_id']
    assert rec._endpoint_worker is None
    assert rec.store.load(first['run_id'])['raw']['endpoint_evidence']['outcome'] == 'TIMEOUT'
    assert 'endpoint_evidence' not in rec.store.load(second['run_id'])['raw']
