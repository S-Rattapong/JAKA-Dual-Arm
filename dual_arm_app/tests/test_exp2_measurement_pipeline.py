"""Offline packet acquisition, timing-policy and provenance regressions."""
from copy import deepcopy
import json
import math
import threading
from unittest.mock import patch

import pytest

from dual_arm_app.backend import experimental_infrastructure as infra
from dual_arm_app.backend.experimental_rigid_grasp import analyze_rigid_grasp
from dual_arm_app.backend.port10000_actual_feedback import Port10000ActualFeedback, Port10000StreamParser
from dual_arm_app.tests.test_exp0_experimental_infrastructure import artifact, report
from dual_arm_app.tests.test_exp2_rigid_grasp import evidence, FIXTURE_URDF


def recorder(tmp_path):
    result = infra.ExperimentRecorder(infra.ExperimentRunStore(tmp_path / 'runs'))
    result.arm(artifact=artifact(), artifact_generation=7, phase4_report=report(), phase4_generation=7, metadata={})
    return result


def bind(rec):
    rec.bind_execution(dict(trajectory_id='test', start_time_unix_ns=1_000_000_000, duration_s=2,
                            artifact_fingerprint='artifact-exp0', artifact_generation=7))


def phase(terminal=False):
    return {'execution': {'trajectory_id': 'test'}, 'driver_feedback': {
        'trajectory_id': 'test', 'authoritative': True, 'terminal': terminal,
        'combined_state': 'COMPLETED' if terminal else 'RUNNING'}}


def frame(value):
    packet = {'len': 0, 'joint_actual_position': [value] * 6}
    while True:
        data = json.dumps(packet, separators=(',', ':')).encode()
        if len(data) == packet['len']:
            return data
        packet['len'] = len(data)


def test_all_parsed_packets_without_browser_and_cache_still_latest(tmp_path):
    rec = recorder(tmp_path)
    feedback = Port10000ActualFeedback({'left': ('offline', 1), 'right': ('offline', 2)})
    feedback.subscribe_packets(rec.ingest_actual_packet)
    feedback.subscribe_packets(rec.ingest_actual_packet)  # idempotent
    # ARMED packets are kept then filtered to the accepted execution interval.
    with patch('dual_arm_app.backend.port10000_actual_feedback.wall_clock_ms', return_value=900):
        feedback._record_packet('left', {'joint_actual_position': [0] * 6})
    bind(rec)
    with patch.object(rec.store, 'update', side_effect=AssertionError('hot path disk write')):
        for side in ('left', 'right'):
            parser = Port10000StreamParser()
            packets = parser.feed(b''.join(frame(i) for i in range(20)))
            for i, packet in enumerate(packets):
                with patch('dual_arm_app.backend.port10000_actual_feedback.wall_clock_ms', return_value=1000 + 100*i):
                    feedback._record_packet(side, packet)
                if i % 7 == 0:
                    rec.observe(phase(), {}, {})
        assert rec.state()['sample_counts']['actual_packets_left'] == 20
        assert rec.state()['sample_counts']['actual_packets_right'] == 20
    snapshot = feedback.snapshot()
    assert snapshot['left']['joint'] == pytest.approx([math.radians(19)]*6)
    snapshot['left']['joint'][0] = 999
    assert feedback.snapshot()['left']['joint'][0] != 999
    rec.observe(phase(True), {}, {})
    raw = rec.store.load(rec.state()['run_id'])['raw']
    assert len(raw['actual_packets']['left']) == 20
    assert raw['actual_packets']['left'][0]['source'] == 'jaka_port10000_actual_feedback'
    assert len(raw['observations']) < 20


def test_acquisition_does_not_wait_for_status_or_store_lock(tmp_path):
    rec = recorder(tmp_path)
    bind(rec)
    done = threading.Event()
    def ingest():
        rec.ingest_actual_packet({'side': 'left', 'received_at_ms': 1100, 'joints_rad': [0]*6})
        done.set()
    with rec._lock, rec.store._lock:
        worker = threading.Thread(target=ingest)
        worker.start()
        assert done.wait(1), 'acquisition blocked by status/persistence lock'
    worker.join()


def direct_evidence():
    raw, manifest = evidence()
    raw['actual_packets'] = {side: [deepcopy(o['actual'][side]) for o in raw['observations']] for side in ('left', 'right')}
    raw['actual_packet_schema'] = 'port10000-packets/v1'
    raw['actual_packet_policy'] = {'expected_period_ms': 100, 'max_interpolation_span_ms': 250}
    raw['observations'] = []
    return raw, manifest


def test_direct_sync_and_legacy_quality_and_no_silent_fallback():
    result = analyze_rigid_grasp(*direct_evidence(), FIXTURE_URDF)
    assert result['status'] == 'AVAILABLE'
    assert result['relative_translation']['magnitude_m']['max'] == 0
    assert result['timing_quality'] == 'DIRECT_HOST_RECEIVE'
    assert 'not controller acquisition' in result['timestamp_uncertainty']
    legacy = analyze_rigid_grasp(*evidence(), FIXTURE_URDF)
    assert legacy['status'] == 'AVAILABLE'
    assert legacy['timing_quality'] == 'LEGACY_DEGRADED_STATUS_SAMPLED'
    raw, manifest = evidence()
    raw['actual_packets'] = {'left': [], 'right': []}
    assert analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)['status'] == 'UNAVAILABLE'


def test_large_gap_rejected_and_counted():
    raw, manifest = direct_evidence()
    raw['execution']['duration_s'] = 1
    start = raw['execution']['start_time_unix_ns']/1e6
    for side, times in [('left', [0, 100, 900, 1000]), ('right', list(range(0, 1001, 100)))]:
        raw['actual_packets'][side] = [{'received_at_ms': start+t, 'joints_rad': [0]*6} for t in times]
    result = analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)
    assert result['status'] == 'PARTIAL'
    assert result['coverage']['skipped_status_counts']['INTERPOLATION_GAP_REJECTED'] == 7
    assert result['coverage']['usable_synchronized_samples'] == 4
    assert result['coverage']['source_gaps_over_policy']['left'] == 1


def test_perfect_moving_tracking_async_10hz_near_model_baseline(tmp_path):
    # A translating rigid pair, with a known 0.9 mm model offset. Both arms
    # follow the same smooth planned position exactly, sampled 37 ms apart.
    urdf = tmp_path / 'moving.urdf'
    urdf.write_text('''<robot name="fixture"><link name="world"/>
      <link name="left_J6"/><link name="right_J6"/>
      <joint name="left_joint_1" type="prismatic"><parent link="world"/><child link="left_J6"/><axis xyz="1 0 0"/></joint>
      <joint name="right_joint_1" type="prismatic"><parent link="world"/><child link="right_J6"/><origin xyz="1.0009 0 0"/><axis xyz="1 0 0"/></joint></robot>''')
    raw, manifest = direct_evidence()
    raw['execution']['duration_s'] = 5
    start = raw['execution']['start_time_unix_ns']/1e6
    def samples(phase_ms):
        return [{'received_at_ms': start+t, 'joints_rad': [0.04*math.sin(t/1000), 0, 0, 0, 0, 0]}
                for t in range(phase_ms, 5001, 100) if t // 100 not in (17, 33)]
    raw['actual_packets'] = {'left': samples(0), 'right': samples(37)}
    async_result = analyze_rigid_grasp(raw, manifest, urdf)
    peak = async_result['relative_translation']['magnitude_m']['max']
    assert 0.0009 <= peak < 0.0012
    assert peak < 0.002
    raw['actual_packets']['right'] = deepcopy(raw['actual_packets']['left'])
    sync = analyze_rigid_grasp(raw, manifest, urdf)
    assert sync['relative_translation']['magnitude_m']['max'] == pytest.approx(0.0009)


def test_provenance_stale_load_and_race_rejects_overwrite(tmp_path, monkeypatch):
    store = infra.ExperimentRunStore(tmp_path / 'runs')
    raw, manifest = direct_evidence()
    run_id = store.generate_run_id()
    raw['run_id'] = manifest['run_id'] = run_id
    store.create(manifest, raw)
    monkeypatch.setattr(infra, 'EXP2_URDF_PATH', FIXTURE_URDF)
    analysis = store.analyze_exp2(run_id)
    assert store.load(run_id)['exp2_analysis_freshness']['status'] == 'CURRENT'
    path = store.root / run_id / 'exp2_analysis.json'
    before = path.read_bytes()
    raw['actual_packets']['left'][0]['joints_rad'][0] = .01
    store.update(run_id, manifest, raw)
    assert store.load(run_id)['exp2_analysis_freshness']['status'] == 'STALE'
    with pytest.raises(infra.ExperimentError, match='Stale'):
        store.write_exp2_analysis(run_id, analysis)
    assert path.read_bytes() == before
    original = infra.analyze_rigid_grasp
    def race(raw_snapshot, manifest_snapshot, urdf):
        result = original(raw_snapshot, manifest_snapshot, urdf)
        raw['revision'] = 2
        store.update(run_id, manifest, raw)
        return result
    monkeypatch.setattr(infra, 'analyze_rigid_grasp', race)
    with pytest.raises(infra.ExperimentError, match='Stale'):
        store.analyze_exp2(run_id)
    assert path.read_bytes() == before


def test_urdf_and_policy_and_missing_provenance_stale(tmp_path, monkeypatch):
    model = tmp_path / 'model.urdf'
    model.write_bytes(FIXTURE_URDF.read_bytes())
    monkeypatch.setattr(infra, 'EXP2_URDF_PATH', model)
    store = infra.ExperimentRunStore(tmp_path / 'runs')
    raw, manifest = direct_evidence()
    run_id = store.generate_run_id()
    raw['run_id'] = manifest['run_id'] = run_id
    store.create(manifest, raw)
    analysis = store.analyze_exp2(run_id)
    model.write_bytes(model.read_bytes() + b'\n')
    assert store.load(run_id)['exp2_analysis_freshness']['status'] == 'STALE'
    with pytest.raises(infra.ExperimentError, match='Stale'):
        store.write_exp2_analysis(run_id, analysis)
    analysis = store.analyze_exp2(run_id)
    raw['actual_packet_policy']['max_interpolation_span_ms'] = 120
    store.update(run_id, manifest, raw)
    assert store.load(run_id)['exp2_analysis_freshness']['status'] == 'STALE'
    analysis.pop('input_provenance')
    # Legacy derived files still load, explicitly stale; new unprovenanced writes fail.
    (store.root / run_id / 'exp2_analysis.json').write_text(json.dumps(analysis))
    assert store.load(run_id)['exp2_analysis_freshness']['status'] == 'STALE'
    with pytest.raises(infra.ExperimentError, match='Stale'):
        store.write_exp2_analysis(run_id, analysis)


def test_receiver_loop_delivers_coalesced_and_fragmented_packets(tmp_path):
    rec = recorder(tmp_path)
    bind(rec)
    feedback = Port10000ActualFeedback({'left': ('offline', 1), 'right': ('offline', 2)})
    feedback.subscribe_packets(rec.ingest_actual_packet)
    wire = b''.join(frame(i) for i in range(12))
    chunks = iter([wire[:31], wire[31:170], wire[170:]])
    class FakeSocket:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def settimeout(self, value):
            pass
        def recv(self, count):
            try:
                return next(chunks)
            except StopIteration:
                feedback.stop()
                return b''
    with patch('dual_arm_app.backend.port10000_actual_feedback.socket.create_connection', return_value=FakeSocket()), \
         patch('dual_arm_app.backend.port10000_actual_feedback.wall_clock_ms', return_value=1500):
        feedback._receive_loop('left')
    assert rec.state()['sample_counts']['actual_packets_left'] == 12
    assert [p['joints_rad'][0] for p in rec._raw['actual_packets']['left']] == pytest.approx([math.radians(i) for i in range(12)])
    assert feedback.snapshot()['left']['joint'][0] == pytest.approx(math.radians(11))


def test_timing_binding_and_terminal_cutoff(tmp_path):
    rec = recorder(tmp_path)
    def send(t):
        rec.ingest_actual_packet({'side': 'left', 'received_at_ms': t, 'joints_rad': [0]*6})
    for t in (900, 1000, 1500, 3100):
        send(t)
    bind(rec)
    for t in (999, 2000, 3001):
        send(t)
    rec.observe(phase(True), {}, {})
    send(2500)
    raw = rec.store.load(rec.state()['run_id'])['raw']
    assert [p['received_at_ms'] for p in raw['actual_packets']['left']] == [1000, 1500, 2000]
    assert raw['result']['sample_counts']['actual_packets_left'] == 3


def test_all_interpolation_targets_rejected_is_unavailable():
    raw, manifest = direct_evidence()
    raw['execution']['duration_s'] = 1.1
    start = raw['execution']['start_time_unix_ns']/1e6
    for side, times in [('left', (0, 1000)), ('right', (100, 1100))]:
        raw['actual_packets'][side] = [{'received_at_ms': start+t, 'joints_rad': [0]*6} for t in times]
    result = analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)
    assert result['status'] == 'UNAVAILABLE'
    assert result['coverage']['usable_fraction'] == 0
    assert result['coverage']['skipped_synchronized_samples'] == 4
    assert result['coverage']['skipped_status_counts']['INTERPOLATION_GAP_REJECTED'] == 2


def test_shared_blind_gap_is_partial_even_when_endpoints_match():
    raw, manifest = direct_evidence()
    raw['execution']['duration_s'] = 1
    for side in ('left', 'right'):
        raw['actual_packets'][side][1]['received_at_ms'] += 900
    result = analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)
    assert result['status'] == 'PARTIAL'
    assert result['coverage']['source_gaps_over_policy'] == {'left': 1, 'right': 1}


def rich_evidence():
    raw, manifest = direct_evidence()
    raw['actual_packet_schema'] = 'port10000-packets/v2'
    raw['execution']['duration_s'] = 4
    times = [i/10 for i in range(41)]
    # Closed, non-retraced joint-space loop, represented exactly as a polyline.
    vectors = [[.08*math.sin(t*math.pi/2), .08*(1-math.cos(t*math.pi/2)), 0, 0, 0, 0] for t in times]
    raw['planned'] = {'common_timestamps_s': times, 'samples': [
        {side: {'joints_rad': q[:]} for side in ('left', 'right')} for q in vectors]}
    start = raw['execution']['start_time_unix_ns']/1e6
    for side in ('left', 'right'):
        raw['actual_packets'][side] = [dict(
            actual_joints_rad=q[:], joints_rad=q[:], controller_joint_position_rad=q[:],
            packet_sequence_index=i+1, received_monotonic_ns=10_000_000_000+i*100_000_000,
            received_at_ms=start+1000*(t if side == 'left' else min(4, t+.45*(1-t/4))))
            for i, (t, q) in enumerate(zip(times, vectors))
            if side == 'left' or i not in (12, 23)]
    return raw, manifest


def moving_model(tmp_path):
    path = tmp_path / 'moving.urdf'
    path.write_text('''<robot name="fixture"><link name="world"/>
    <link name="left_J6"/><link name="right_J6"/>
    <joint name="left_joint_1" type="prismatic"><parent link="world"/><child link="left_J6"/><axis xyz="1 0 0"/></joint>
    <joint name="right_joint_1" type="prismatic"><parent link="world"/><child link="right_J6"/><origin xyz="1.0009 0 0"/><axis xyz="1 0 0"/></joint></robot>''')
    return path


def test_phase_falsifies_host_jitter_and_detects_actual_deviation(tmp_path):
    raw, manifest = rich_evidence()
    model = moving_model(tmp_path)
    before = deepcopy(raw)
    result = analyze_rigid_grasp(raw, manifest, model)
    phase_result = result['phase_synchronized']
    assert phase_result['status'] == 'AVAILABLE'
    assert result['relative_translation']['magnitude_m']['max'] > .02
    assert phase_result['relative_translation']['magnitude_m']['max'] < .0015
    assert phase_result['center_consistency']['translation_magnitude_m']['max'] < .0015
    assert phase_result['relative_orientation']['angle']['max'] == 0
    assert raw == before
    assert phase_result['samples'][0]['phase_time_s'] == 0
    assert phase_result['samples'][-1]['phase_time_s'] == pytest.approx(4)
    assert phase_result['phase_match_quality']['left']['packets'][1]['phase_time_s'] == pytest.approx(.1)
    assert result['same_packet_tracking']['sides']['right']['overall_rad']['max_abs'] == 0
    packet = raw['actual_packets']['right'][18]
    packet['actual_joints_rad'][0] += .03
    packet['joints_rad'][0] += .03
    changed = analyze_rigid_grasp(raw, manifest, model)
    assert changed['phase_synchronized']['relative_translation']['magnitude_m']['max'] > .0308
    tracking = changed['same_packet_tracking']['sides']['right']
    assert tracking['overall_rad']['max_abs'] == pytest.approx(.03)
    assert tracking['overall_rad']['max_abs_location']['packet_sequence_index'] == packet['packet_sequence_index']
    assert tracking['per_joint_rad'][0]['mean'] == pytest.approx(.03/tracking['sample_count'])
    assert tracking['per_joint_rad'][0]['rmse'] == pytest.approx(.03/math.sqrt(tracking['sample_count']))
    assert tracking['per_joint_deg'][0]['max_abs'] == pytest.approx(math.degrees(.03))


def test_phase_continuous_segment_projection_and_rejections(tmp_path):
    raw, manifest = rich_evidence()
    packet = raw['actual_packets']['left'][5]
    q0, q1 = [raw['planned']['samples'][i]['left']['joints_rad'] for i in (4, 5)]
    q = [(a+b)/2 for a, b in zip(q0, q1)]
    packet.update(actual_joints_rad=q[:], joints_rad=q[:], controller_joint_position_rad=q[:])
    raw['actual_packets']['left'][8]['controller_joint_position_rad'] = [10]*6
    raw['actual_packets']['left'][10]['packet_sequence_index'] = 1
    result = analyze_rigid_grasp(raw, manifest, moving_model(tmp_path))['phase_synchronized']
    quality = result['phase_match_quality']['left']
    assert quality['packets'][5]['phase_time_s'] == pytest.approx(.45)
    assert quality['status_counts']['POOR_MATCH'] == 1
    assert quality['status_counts']['NONMONOTONIC_SEQUENCE'] == 1
    assert result['status'] == 'PARTIAL'


@pytest.mark.parametrize('schema', ['port10000-packets/v1', 'port10000-packets/v2'])
def test_missing_same_frame_fields_explicitly_unavailable(schema):
    raw, manifest = direct_evidence()
    raw['actual_packet_schema'] = schema
    result = analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)
    assert result['status'] == 'AVAILABLE'
    for key in ('same_packet_tracking', 'phase_synchronized'):
        assert result[key]['status'] == 'UNAVAILABLE'
        assert result[key]['reason']


def test_rich_fragmented_coalesced_capture_persisted_memory_only(tmp_path):
    rec = recorder(tmp_path)
    bind(rec)
    feedback = Port10000ActualFeedback({'left': ('offline', 1), 'right': ('offline', 2)})
    seen = []
    feedback.subscribe_packets(seen.append)
    feedback.subscribe_packets(rec.ingest_actual_packet)
    wire = b''
    for i in range(3):
        packet = {'len': 0, 'joint_actual_position': [i+1]*6 + ['ignored'],
                  'joint_position': [i]*6 + ['ignored'], 'position': [100, 2.5], 'actual_position': [99, 3]}
        while True:
            encoded = json.dumps(packet, separators=(',', ':')).encode()
            if len(encoded) == packet['len']:
                break
            packet['len'] = len(encoded)
        wire += encoded
    parser = Port10000StreamParser()
    with patch('dual_arm_app.backend.port10000_actual_feedback.wall_clock_ms', return_value=1500), \
         patch.object(rec.store, 'update', side_effect=AssertionError('disk I/O')):
        for chunk in (wire[:23], wire[23:150], wire[150:]):
            for packet in parser.feed(chunk):
                feedback._record_packet('left', packet)
    assert len(seen) == 3
    for i, p in enumerate(seen):
        assert p['actual_joints_rad'] == p['joints_rad'] == pytest.approx([math.radians(i+1)]*6)
        assert p['controller_joint_position_rad'] == pytest.approx([math.radians(i)]*6)
        assert p['controller_position_raw'] == [100, 2.5]
        assert p['controller_actual_position_raw'] == [99, 3]
        assert p['packet_length_bytes'] > 0
        assert p['packet_sequence_index'] == i+1
        assert p['received_monotonic_ns'] > 0
        assert all(v == 'VALID' for v in p['field_validation'].values())
    seen[0]['controller_position_raw'][0] = 900
    rec.observe(phase(True), {}, {})
    raw = rec.store.load(rec.state()['run_id'])['raw']
    assert raw['actual_packet_schema'] == 'port10000-packets/v2'
    assert raw['actual_packets']['left'][0]['controller_position_raw'] == [100, 2.5]
    assert raw['actual_packets']['left'][0]['received_monotonic_ns'] == seen[0]['received_monotonic_ns']


@pytest.mark.parametrize('optional', [{}, {'joint_position': [False]*6, 'position': [float('nan')], 'actual_position': 'bad', 'len': True}])
def test_optional_controller_fields_never_break_actual_cache(optional):
    feedback = Port10000ActualFeedback({'left': ('offline', 1), 'right': ('offline', 2)})
    seen = []
    feedback.subscribe_packets(seen.append)
    feedback._record_packet('left', {'joint_actual_position': [90]*6, **optional})
    assert feedback.snapshot()['left']['joint'] == pytest.approx([math.pi/2]*6)
    p = seen[0]
    assert p['controller_joint_position_rad'] is None
    assert p['controller_position_raw'] is None
    assert p['controller_actual_position_raw'] is None
    assert p['field_validation']['joint_position'] == ('INVALID' if optional else 'MISSING')
    feedback._record_packet('right', {'joint_actual_position': [0]*6})
    assert seen[-1]['packet_sequence_index'] == 1


def test_phase_dwell_ambiguity_and_large_phase_gap(tmp_path):
    raw, manifest = rich_evidence()
    # A phase-space gap may have identical usable endpoints: still PARTIAL.
    raw['actual_packets']['left'] = [p for i, p in enumerate(raw['actual_packets']['left']) if i not in (11, 12, 13)]
    result = analyze_rigid_grasp(raw, manifest, moving_model(tmp_path))['phase_synchronized']
    assert result['status'] == 'PARTIAL'
    assert result['coverage']['source_gaps_over_policy']['left'] == 1
    assert result['coverage']['skipped_status_counts']['INTERPOLATION_GAP_REJECTED'] >= 2
    # All poses identical: no encoder information establishes phase within dwell.
    for sample in raw['planned']['samples']:
        for side in ('left', 'right'):
            sample[side]['joints_rad'] = [0]*6
    for side in ('left', 'right'):
        for p in raw['actual_packets'][side]:
            p['controller_joint_position_rad'] = p['actual_joints_rad'] = [0]*6
    result = analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)['phase_synchronized']
    assert result['status'] == 'UNAVAILABLE'
    assert result['phase_match_quality']['left']['status_counts']['AMBIGUOUS_MATCH'] > 0


def test_legacy_new_sections_and_same_frame_hash_guard(tmp_path, monkeypatch):
    legacy = analyze_rigid_grasp(*evidence(), FIXTURE_URDF)
    assert legacy['status'] == 'AVAILABLE'
    assert legacy['same_packet_tracking']['status'] == 'UNAVAILABLE'
    assert legacy['phase_synchronized']['status'] == 'UNAVAILABLE'
    store = infra.ExperimentRunStore(tmp_path / 'runs')
    raw, manifest = rich_evidence()
    run_id = store.generate_run_id()
    raw['run_id'] = manifest['run_id'] = run_id
    store.create(manifest, raw)
    monkeypatch.setattr(infra, 'EXP2_URDF_PATH', FIXTURE_URDF)
    analysis = store.analyze_exp2(run_id)
    raw['actual_packets']['left'][0]['controller_joint_position_rad'][0] += .01
    store.update(run_id, manifest, raw)
    assert store.load(run_id)['exp2_analysis_freshness']['status'] == 'STALE'
    with pytest.raises(infra.ExperimentError, match='Stale'):
        store.write_exp2_analysis(run_id, analysis)


def test_phase_first_packet_projects_into_early_window_without_end_pose_jump(tmp_path):
    raw, manifest = rich_evidence()
    # Drop the first two packets: the first available controller-reported pose is
    # phase 0.2 s.  The plan closes back onto its start pose at 4 s, so this also
    # proves initial matching does not jump to the repeated final pose.
    for side in ('left', 'right'):
        raw['actual_packets'][side] = raw['actual_packets'][side][2:]
    result = analyze_rigid_grasp(raw, manifest, moving_model(tmp_path))['phase_synchronized']
    assert result['status'] == 'AVAILABLE'
    for side in ('left', 'right'):
        first = next(item for item in result['phase_match_quality'][side]['packets']
                     if item['status'] == 'MATCHED')
        assert first['phase_time_s'] == pytest.approx(0.2)
        assert first['phase_time_s'] < 0.5
