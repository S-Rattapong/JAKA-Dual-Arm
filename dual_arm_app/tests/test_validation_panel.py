"""Synthetic geometry, standalone ASGI routes and temporary stores only."""
import asyncio
import copy
import json
import math
import time

from fastapi import FastAPI
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dual_arm_app.backend.validation_api import validation_router
from dual_arm_app.backend.validation_engine import Geometry, SIDES, analyze, crosscheck, gap, shift, snapshot
from dual_arm_app.backend.validation_store import PassiveTap, ValidationSessionStore

ZERO = [0]*6
POSE = {"translation_m":[.04,-.01,.12],"rotation_rpy_rad":[.2,-.4,.3]}


def geometry():
    return Geometry(snapshot())


def mapping(g, side="left"):
    return {"mapping":{side:{"verified":True,"source":"synthetic verified TCP fixture","tool_id":"fixture",
                             "field":"controller_actual_position_raw","frame_id":"base 0","frame":"base",
                             "angle_unit":"deg","length_unit":"mm","orientation":"RzRyRx","j6_from_tcp":POSE,
                             "model_revision":g.cal["revision"]}}}


def packet(g,side):
    from dual_arm_app.backend.validation_engine import transform
    q=[.3,-.4,.7,.1,-.1,.25]
    p=np.linalg.inv(g.base[side]) @ g.pose(side,q) @ transform(POSE)
    return {"actual_joints_rad":q,"mapping":"port10000_joint_actual_position_deg_to_rad","controller_actual_position_raw":[*(p[:3,3]*1000),*Rotation.from_matrix(p[:3,:3]).as_euler("xyz",degrees=True)],
            "received_monotonic_ns":1,"field_validation":{"actual_position":"VALID","joint_actual_position":"VALID"}}


def test_v1_same_packet_transform_once_and_block():
    g=geometry()
    p=packet(g,"left")
    c=mapping(g)
    result=crosscheck(g,"left",p,c)
    assert result["error_mm"]<1e-9 and result["orientation_deg"]<1e-9
    for key,value in [("frame","unknown"),("verified",False),("length_unit",""),("tool_id",""),("model_revision","old")]:
        bad=copy.deepcopy(c)
        bad["mapping"]["left"][key]=value
        with pytest.raises(ValueError):
            crosscheck(g,"left",p,bad)
    p["field_validation"]["actual_position"]="INVALID"
    with pytest.raises(ValueError):
        crosscheck(g,"left",p,c)


def test_gap_rotated_surface_and_interval():
    poses={s:np.eye(4) for s in SIDES}
    for p in poses.values():
        p[:3,:3]=Rotation.from_euler("y",90,degrees=True).as_matrix()
    poses["right"][:3,3]=[.012,.3,0]
    c={"parallel_deg":1,"gauge":{"brand":"test","type":"leaf","uncertainty_mm":.02,"resolution_mm":.01},
       "surfaces":{s:{"tool_frame":"J6","normal":[0,0,1],"point_m":[0,0,.002 if s=="left" else 0],"source":"fixture"} for s in SIDES}}
    t={"planned_mm":10,"measured_mm":11}
    r=gap(poses,c,t)
    assert r["fk_gap_mm"]==pytest.approx(10) and r["error_mm"]==pytest.approx(1)
    r=gap(poses,c,dict(t,measured_mm=None,interval_mm=[10,12]))
    assert r["error_interval_mm"]==pytest.approx([0,2]) and "error_mm" not in r
    c["surfaces"]["right"]["normal"]=[1,0,0]
    with pytest.raises(ValueError,match="Planes"):
        gap(poses,c,t)


def sample(t, q):
    return {"key":str(t),"trial":t,"packets":{t["side"]:[{"actual_joints_rad":q}]}}


def test_v4_incomplete_and_dispersion(tmp_path):
    store=ValidationSessionStore(tmp_path)
    c={"repetitions":2,"side":"left","jigs":{"j":{"source":"CMM","distance_mm":100,"uncertainty_mm":.1}},
       "probes":{"left":{"source":"test","tool_frame":"J6","point_m":[.01,0,0]}}}
    s=store.create("V4",c)
    q=[.2,.1,0,-.3,.5,0]
    g=Geometry(s["snapshot"])
    p=np.r_[.01,0,0,1]
    dist=np.linalg.norm(((g.pose("left",q)-g.pose("left",ZERO))@p)[:3])*1000
    t={"side":"left","socket":"A","jig":"j","repeat":1}
    s["samples"]=[sample(t,ZERO)]
    assert analyze(s)["acceptance"]=="CANNOT_EVALUATE"
    for i in (1,2):
        if i==2:s["samples"].append(sample(dict(t,repeat=i),ZERO))
        s["samples"].append(sample(dict(t,repeat=i,socket="B"),q))
    r=analyze(s)
    assert r["acceptance"]=="MEASURED_NO_CRITERION"
    assert r["rows"][0]["error_mm"]==pytest.approx(dist-100)
    assert all(x["radial_rms_mm"]==0 for x in r["repeat_dispersion"])


def test_v4_missing_reference_uncertainty_is_characterization_only(tmp_path):
    store = ValidationSessionStore(tmp_path)
    config = {
        "repetitions": 1,
        "side": "left",
        "jigs": {"test_jig": {"source": "caliper, resolution known", "distance_mm": 100,
                               "uncertainty_mm": None}},
        "probes": {"left": {"source": "estimated from tool measurements",
                            "tool_frame": "J6",
                            "point_m": [.03982, -.026, .27006]}},
    }
    s = store.create("V4", config)
    s["samples"] = [
        sample({"side": "left", "socket": name, "jig": "test_jig", "repeat": 1},
               [0, 0, 0, 0, 0, 0] if name == "A" else [.1, .1, .1, 0, 0, 0])
        for name in ("A", "B")
    ]
    result = analyze(s)
    assert len(result["rows"]) == 1
    assert result["issues"] == []
    assert result["acceptance"] == "MEASURED_NO_CRITERION"
    assert result["reference_uncertainty"]["status"] == "UNKNOWN"
    assert "UNKNOWN" in result["limitation"]
    s["config"]["criterion"] = {
        "value": 1.0,
        "unit": "mm",
        "source": "independent design requirement",
        "uncertainty": "not yet known",
    }
    with_limit = analyze(s)
    assert len(with_limit["rows"]) == 1
    assert with_limit["acceptance"] == "CANNOT_EVALUATE"
    assert any("Unknown reference Jig" in x for x in with_limit["criterion_issues"])


@pytest.mark.parametrize("offset",[[0,0,0],[1,-2,3]])
def test_v5_signed_world_offset(tmp_path,offset):
    g=geometry()
    world=np.array([.1,.2,.8,1])
    c={"repetitions":1,"sockets":["P"],"probes":{}}
    for side in SIDES:
        target=world.copy()
        if side=="left":target[:3]+=np.array(offset)/1000
        c["probes"][side]={"source":"test","tool_frame":"J6","point_m":(np.linalg.inv(g.pose(side,ZERO))@target)[:3].tolist()}
    s=ValidationSessionStore(tmp_path).create("V5",c)
    s["samples"]=[sample({"role":"validation","side":side,"repeat":1,"socket":"P"},ZERO) for side in SIDES]
    r=analyze(s)
    assert r["rows"][0]["xyz_mm"]==pytest.approx(offset,abs=1e-9)
    assert r["rows"][0]["error_mm"]==pytest.approx(np.linalg.norm(offset),abs=1e-9)
    assert r["acceptance"]=="CANNOT_EVALUATE"  # spatial coverage still insufficient


def timing(delay):
    t=np.arange(201)/100
    q=lambda x:[math.sin(x*(j+1)) for j in range(6)]
    plan={"t":t.tolist(),"left":[q(x) for x in t]}
    packets=[{"received_at_ms":i*10,"actual_joints_rad":q(x-delay)} for i,x in enumerate(t) if not .8<x<1.2]
    return plan,packets,{"tau_min_ms":-100,"tau_max_ms":100,"tau_step_ms":10}


def test_shift_equal_support_and_outages():
    plan,ps,c=timing(.06)
    r=shift(plan,ps,"left",0,c)
    assert r["tau_ms"]==60 and r["after_rad"]<1e-12 and r["before_rad"]>.01
    assert r["common_packets"]==sum(100<=p["received_at_ms"]<=1900 for p in ps)
    assert r["packet_gaps"]==1 and not r["warnings"]
    for p in ps:p["actual_joints_rad"]=ZERO
    plan["left"]=[ZERO]*len(plan["t"])
    assert "FLAT_CURVE" in shift(plan,ps,"left",0,c)["warnings"]
    plan,ps,c=timing(.1)
    assert "BOUNDARY_MINIMUM" in shift(plan,ps,"left",0,c)["warnings"]


def idle():
    return {s:{"feedback_status":"LIVE","state":{"motion_state":0}} for s in SIDES}


def fill(tap, now, moving=False):
    for i in range(12):
        tap.observe({"side":"left","received_at_ms":i*100,"received_monotonic_ns":now-(11-i)*100_000_000,
                     "packet_sequence_index":i,"actual_joints_rad":[i*.01 if moving else 0]*6})


def test_capture_valid_reject_stale_motion_duplicates(monkeypatch):
    now=12_000_000_000
    monkeypatch.setattr(time,"monotonic_ns",lambda:now)
    for age,moving,error in [(0,False,None),(500_000_000,False,"stale"),(0,True,"motion")]:
        tap=PassiveTap()
        fill(tap,now-age,moving)
        if error:
            with pytest.raises(ValueError,match=error):tap.capture(["left"],idle)
        else:
            assert tap.capture(["left"],idle)["quality"]["left"]["unique_packets"]==11
            tap.motion("right")
            with pytest.raises(ValueError,match="motion"):tap.capture(["left"],idle)
    tap=PassiveTap()
    for _ in range(15):tap.observe({"side":"left","packet_sequence_index":1,"received_monotonic_ns":now})
    with pytest.raises(ValueError,match="1 unique"):tap.capture(["left"],idle)


def client(app):
    # Use TestClient when its optional transport is installed. This host also
    # supports direct ASGI dispatch, which opens no socket.
    try:
        from fastapi.testclient import TestClient
    except (ImportError,RuntimeError):
        async def call(method,path,b):
            data=json.dumps(b).encode()
            scope={"type":"http","http_version":"1.1","scheme":"http","method":method,"path":path,
                   "raw_path":path.encode(),"query_string":b"","headers":[],"server":("offline",80),"client":("fixture",1)}
            messages=[]
            async def receive():return {"type":"http.request","body":data,"more_body":False}
            async def send(m):messages.append(m)
            # Managed offline sandboxes may suppress asyncio self-pipe wakeups.
            task = asyncio.create_task(app(scope,receive,send))
            deadline = time.monotonic()+10
            while not task.done():
                if time.monotonic()>deadline:
                    task.cancel()
                    raise TimeoutError(path)
                await asyncio.sleep(.005)
            await task
            return messages[0]["status"],b"".join(m.get("body",b"") for m in messages)
        return lambda m,p,b=None:asyncio.run(call(m,p,b))
    c=TestClient(app)
    def request(m,p,b=None):
        r=c.request(m,p,json=b)
        return r.status_code,r.content
    return request


def test_router_session_lifecycle_and_replay(tmp_path):
    g=geometry()
    store=ValidationSessionStore(tmp_path/"sessions",tmp_path/"runs")
    tap=PassiveTap()
    app=FastAPI()
    app.include_router(validation_router(tap,idle,store))
    req=client(app)
    def request(method,path,body=None):
        status,data=req(method,"/api/validation"+path,body)
        assert status==200,data
        return json.loads(data)
    assert len(request("GET","/catalog")["methods"])==5
    raw={"run_id":"exp-20261019T063733587Z-c16d24cb","actual_packets":{"left":[packet(g,"left")]},
         "plan_snapshot":{"calibration":{"model_revision":g.cal["revision"]}}}
    folder=store.runs/raw["run_id"]
    folder.mkdir(parents=True)
    (folder/"raw.json").write_text(json.dumps(raw))
    assert request("GET","/runs")==[raw["run_id"]]
    initial=request("POST","/sessions",{"method":"V1","config":mapping(g)})
    path="/sessions/"+initial["id"]
    data={"key":"trial1","run_id":raw["run_id"]}
    for _ in range(2):assert len(request("POST",path+"/run",data)["samples"])==1
    assert request("POST",path+"/analysis",{})["analysis"]["absolute_error"]["max"]<1e-9
    for fmt,magic in [("csv",b"record"),("png",b"\x89PNG")]:
        code,body=req("GET","/api/validation"+path+"/export/"+fmt)
        assert code==200 and body.startswith(magic)
    exported=request("GET",path+"/export/json")
    assert analyze(exported)==exported["analysis"]
    assert store.read(initial["id"])==exported
    request("POST",path+"/exclude",{"key":"trial1","reason":"test"})
    assert request("POST",path+"/analysis",{})["analysis"]["acceptance"]=="CANNOT_EVALUATE"
    assert request("POST",path+"/unexclude",{"key":"trial1"})["samples"][0]["excluded"] is False
    assert request("GET",path)["samples"][0]["history"]
    assert req("POST","/api/validation"+path+"/run",{"key":"bad","run_id":"../raw"})[0]==422
    assert request("GET","/sessions")==[initial["id"]]
    fill(tap,time.monotonic_ns())
    body={"key":"cached","trial":{"side":"left"}}
    assert len(request("POST",path+"/capture",body)["samples"])==2
    assert len(request("POST",path+"/capture",body)["samples"])==2
    assert req("POST","/api/validation"+path+"/capture",dict(body,key="dup"))[0]==422


def test_v3_interval_acceptance_and_mounted_tool(tmp_path):
    g=geometry()
    surfaces={}
    for side in SIDES:
        # Invert a non-identity tool mount to define known world planes.
        from dual_arm_app.backend.validation_engine import transform
        tool=g.pose(side,ZERO)@transform(POSE)
        world=np.array([.01 if side=="right" else 0,.2 if side=="right" else 0,0,1])
        surfaces[side]={"tool_frame":"mounted","j6_from_tool":POSE,"tool_id":"gauge fixture",
                        "point_m":(np.linalg.inv(tool)@world)[:3].tolist(),
                        "normal":(tool[:3,:3].T@[1,0,0]).tolist(),"source":"fixture"}
    c={"surfaces":surfaces,"parallel_deg":.1,"repetitions":1,
       "gauge":{"type":"leaf","brand":"fixture","resolution_mm":.1,"uncertainty_mm":.2},
       "criterion":{"value":2,"unit":"mm","source":"synthetic protocol","uncertainty":"0.2 mm"}}
    s=ValidationSessionStore(tmp_path).create("V3",c)
    t={"pose":"P1","direction":"+X","repeat":1,"planned_mm":10}
    s["samples"]=[{"trial":t,"key":"static","packets":{side:[{"actual_joints_rad":ZERO}] for side in SIDES}}]
    for interval,status in [([10,11],"PASS"),([11,13],"CANNOT_EVALUATE"),([14,16],"FAIL")]:
        t["interval_mm"]=interval
        r=analyze(s)
        assert r["enough_data"] and r["acceptance"]==status
        assert r["rows"][0]["fk_gap_mm"]==pytest.approx(10)
        assert "error_mm" not in r["rows"][0]
    s["snapshot"]["urdf"]+=" "
    assert analyze(s)["data_status"]=="CANNOT_EVALUATE"


def test_v5_coverage_and_training_isolation(tmp_path):
    c={"sockets":["1","2","3","4"],"training_sockets":["T"],"repetitions":2,
       "probes":{side:{"source":"fixture","tool_frame":"J6","point_m":[.02,0,0]} for side in SIDES}}
    s=ValidationSessionStore(tmp_path).create("V5",c)
    contacts=[ZERO,[.6,0,0,0,0,0],[0,.6,0,0,0,0],[0,0,.8,0,0,0]]
    for socket,q in zip(c["sockets"],contacts):
        for repeat in (1,2):
            for side in SIDES:
                s["samples"].append(sample(dict(side=side,socket=socket,repeat=repeat,role="validation"),q))
    s["samples"].append(sample(dict(side="left",socket="T",repeat=1,role="training"),ZERO))
    a=analyze(s)
    assert a["enough_data"] and len(a["rows"])==8 and len(a["by_point_mm"])==4
    assert len(a["training"]["issues"])==1 and not a["issues"]


def test_v2_photo_finish_and_historical_criterion(tmp_path):
    from dual_arm_app.backend.validation_store import compact_run
    plan,left,c=timing(.06)
    _,right,_=timing(.03)
    c["criterion"]={"value":40,"unit":"ms","source":"study protocol","uncertainty":"10 ms grid"}
    raw={"actual_packets":{"left":left,"right":right},"execution":{"start_time_unix_ns":0},
         "planned":{"samples":[{"time_from_start_s":t,**{s:{"joints_rad":q} for s in SIDES}}
                               for t,q in zip(plan["t"],plan["left"])]}}
    s=ValidationSessionStore(tmp_path).create("V2",c)
    s["samples"]=[dict(compact_run(raw,"V2"),trial={},key="1",raw_sha256="fixture")]
    a=analyze(s)
    assert a["rows"][0]["delta_tau_ms"]==-30 and a["criterion_issues"]
    assert a["enough_data"] and a["acceptance"]=="CANNOT_EVALUATE"
    s["config"]["criterion"]["declared_at_ms"]=-1
    assert analyze(s)["acceptance"]=="PASS"


def test_concurrent_retry_single_capture(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    store=ValidationSessionStore(tmp_path)
    s=store.create("V1",{})
    calls=[]
    def produce(_):
        calls.append(1)
        return {"packets":{}}
    def capture(_):
        return store.add(s["id"],"retry",{"side":"left"},produce)
    with ThreadPoolExecutor(2) as pool:
        snapshots=list(pool.map(capture,range(2)))
    assert len(calls)==1 and all(len(item["samples"])==1 for item in snapshots)
    with pytest.raises(ValueError,match="another trial"):
        store.add(s["id"],"retry",{"side":"right"},produce)


def test_saved_model_survives_changed_files_and_blocks_mixed_capture(tmp_path,monkeypatch):
    from dual_arm_app.backend import validation_api
    store=ValidationSessionStore(tmp_path)
    g=geometry()
    s=store.create("V1",mapping(g))
    store.add(s["id"],"record",{},lambda _:{"packets":{"left":[packet(g,"left")]},
                                            "model_revision":g.cal["revision"]})
    before=analyze(store.read(s["id"]))
    changed=snapshot()
    changed["urdf_sha256"]="another-model"
    monkeypatch.setattr(validation_api,"snapshot",lambda:changed)
    app=FastAPI()
    app.include_router(validation_router(PassiveTap(),idle,store))
    call=client(app)
    path="/api/validation/sessions/"+s["id"]
    code,body=call("POST",path+"/capture",{"key":"changed","trial":{"side":"left"}})
    assert code==422 and b"changed since configuration" in body
    code,body=call("POST",path+"/analysis",{})
    assert code==200 and json.loads(body)["analysis"]["rows"]==before["rows"]
    assert len(store.read(s["id"])["samples"])==1



def test_validation_overview_is_derived_from_atomic_session_state(tmp_path):
    store = ValidationSessionStore(tmp_path)
    initial = store.overview()
    assert len(initial)==5 and all(v["state"]=="NO_SESSION" for v in initial.values())
    s = store.create("V4", {"repetitions": 1,"side": "left",
             "jigs": {"J":{"source":"independent tape","distance_mm":100,"uncertainty_mm":1}},
             "probes":{"left":{"source":"fixture","point_m":[0,0,0]}}})
    first = store.overview()["V4"]
    assert first["state"]=="NOT_STARTED" and first["session_id"]==s["id"]
    store.add(s["id"],"capture1",{"side":"left","socket":"A","jig":"J","repeat":1},
              lambda _ : {"packets":{"left":[{"actual_joints_rad":[0]*6}]}})
    second=store.overview()["V4"]
    assert second["active_samples"]==1 and second["state"]=="CAPTURED_NOT_ANALYZED"
    assert store.read(s["id"])["samples"][0]["key"]=="capture1"
    assert store.overview()["V1"]["state"]=="NO_SESSION"


def test_stationary_policy_is_adjustable_bounded_and_saved(tmp_path, monkeypatch):
    from dual_arm_app.backend.validation_store import stationary_policy
    p = stationary_policy({"min_packets": 10, "max_drift_rad": .003,
                           "max_gap_s": .2})
    assert p["min_packets"] == 10 and p["max_drift_rad"] == .003
    assert p["window_s"] == 1 and p["max_gap_s"] == .2
    with pytest.raises(ValueError, match="range"):
        stationary_policy({"max_drift_rad": 100})
    with pytest.raises(ValueError, match="integer"):
        stationary_policy({"min_packets": 6.5})
    with pytest.raises(ValueError, match="Unsupported"):
        stationary_policy({"jaka_servo_period_ms": 10})
    now = 20_000_000_000
    monkeypatch.setattr(time, "monotonic_ns", lambda: now)
    tap = PassiveTap()
    fill(tap, now)
    data = tap.capture(["left"], idle, {"min_packets": 10, "window_s": 1})
    assert data["quality"]["left"]["unique_packets"] >= 10
    assert data["policy"]["min_packets"] == 10
    with pytest.raises(ValueError, match="insufficient"):
        tap.capture(["left"], idle, {"min_packets": 15})
    config = {"repetitions": 1, "side": "left",
              "stationary_policy": {"min_packets": 10, "max_gap_s": .2},
              "jigs": {"J": {"source": "measured reference","distance_mm":100, "uncertainty_mm":1}},
              "probes": {"left":{"source":"fixture","point_m":[0,0,0]}}}
    session = ValidationSessionStore(tmp_path).create("V4", config)
    assert session["config"]["stationary_policy"]["min_packets"] == 10
    assert session["config"]["stationary_policy"]["window_s"] == 1
