"""Synthetic fail-closed MQ regression tests; never connect to a robot."""
import hashlib
import json
from real_robot_scripts import mq_qualification as mq


def fixture(tmp_path, monkeypatch):
    model=tmp_path/"model.urdf"; model.write_text("<robot/>")
    monkeypatch.setattr(mq,"MODEL",model)
    p=tmp_path/"exp-20261019T000000000Z-abcdef12";p.mkdir()
    rows=[dict(received_at_ms=1000+100*i,received_monotonic_ns=1_000_000_000+100_000_000*i,
               packet_sequence_index=i,actual_joints_rad=[0.]*6,controller_joint_position_rad=[0.]*6)
          for i in range(65)]
    raw=dict(run_id=p.name,actual_packet_schema="port10000-packets/v2",
        actual_packet_policy={"max_interpolation_span_ms":250},
        execution={"duration_s":6.},actual_packets={"left":rows,"right":[dict(x) for x in rows]})
    manifest=dict(run_id=p.name,state="CLOSED",result={"combined_state":"COMPLETED"})
    analysis=dict(run_id=p.name,phase_synchronized=dict(status="AVAILABLE",
        coverage=dict(usable_fraction=.98,synchronized_candidate_samples=130,
                      usable_synchronized_samples=127)),
        same_packet_tracking=dict(status="AVAILABLE",sides={
            arm:{"overall_rad":{"sample_count":390}} for arm in ("left","right")}),
        input_provenance=dict(raw_sha256=mq.canonical_hash(raw),
           manifest_sha256=mq.canonical_hash(manifest),
           urdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
           analyzer_version="exp2-same-frame-phase/v3"))
    return p,raw,manifest,analysis


def failures(p,raw,manifest,analysis):
    for name,obj in [("raw.json",raw),("manifest.json",manifest),
                     ("exp2_analysis.json",analysis)]:
        (p/name).write_text(json.dumps(obj))
    return [x["name"] for x in mq.saved_run_gate(p) if not x["passed"]]


def test_valid_synthetic_fixture(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    assert failures(p,r,m,a)==[]


def test_tampered_raw_detected(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    r["actual_packets"]["left"][0]["actual_joints_rad"][0]=.001
    assert "provenance_raw_sha256" in failures(p,r,m,a)


def test_timestamp_reversal_detected(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    r["actual_packets"]["right"][12]["received_at_ms"]=500
    a["input_provenance"]["raw_sha256"]=mq.canonical_hash(r)
    found=failures(p,r,m,a)
    assert "right_timestamps_monotonic" in found


def test_missing_same_packet_field_detected(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    del r["actual_packets"]["left"][3]["controller_joint_position_rad"]
    a["input_provenance"]["raw_sha256"]=mq.canonical_hash(r)
    assert "left_joint_field_coverage" in failures(p,r,m,a)


def test_low_coverage_detected(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    a["phase_synchronized"]["coverage"]["usable_fraction"]=.84
    assert "phase_coverage" in failures(p,r,m,a)


def test_missing_run_files_fail_closed(tmp_path):
    assert [x["name"] for x in mq.saved_run_gate(tmp_path)]==["read_saved_evidence"]


def test_changed_urdf_detected(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    mq.MODEL.write_text("<robot changed='yes'/>")
    assert "provenance_urdf_sha256" in failures(p,r,m,a)


def test_duplicate_wall_millisecond_is_not_false_timestamp_reversal(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    r["actual_packets"]["left"][12]["received_at_ms"] = r["actual_packets"]["left"][11]["received_at_ms"]
    a["input_provenance"]["raw_sha256"] = mq.canonical_hash(r)
    assert "left_timestamps_monotonic" not in failures(p,r,m,a)


def test_monotonic_nanosecond_reversal_is_rejected(tmp_path,monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    r["actual_packets"]["left"][12]["received_monotonic_ns"] = r["actual_packets"]["left"][11]["received_monotonic_ns"]
    a["input_provenance"]["raw_sha256"] = mq.canonical_hash(r)
    assert "left_timestamps_monotonic" in failures(p,r,m,a)


def test_large_real_monotonic_gap_rejected(tmp_path, monkeypatch):
    p,r,m,a=fixture(tmp_path,monkeypatch)
    for packet in r["actual_packets"]["right"][12:]:
        packet["received_monotonic_ns"] += 200_000_000
    a["input_provenance"]["raw_sha256"] = mq.canonical_hash(r)
    assert "right_receive_gap_bounded" in failures(p,r,m,a)
