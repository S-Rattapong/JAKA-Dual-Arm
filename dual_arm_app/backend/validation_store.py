"""Local immutable evidence and transactional session updates."""
from collections import deque
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time
import uuid

import numpy as np

try:
    from .validation_engine import APP, METHODS, METRICS, SIDES, number, require, snapshot, vec
except ImportError:
    from validation_engine import APP, METHODS, METRICS, SIDES, number, require, snapshot, vec

RUN = re.compile(r"exp-[0-9]{8}T[0-9]{9}Z-[0-9a-f]{8}")
KEY = re.compile(r"[a-zA-Z0-9_-]{1,80}")
MAX_RUN = 64*1024**2
MAX_SESSION = 32*1024**2
POLICY = {"window_s": 1, "min_packets": 6, "max_age_s": .25, "max_gap_s": .25,
          "max_drift_rad": .002, "max_speed_rad_s": .02}
POLICY_BOUNDS = {"window_s": (1.0, 8.0), "min_packets": (3, 100),
                 "max_age_s": (.02, 2.0), "max_gap_s": (.01, 2.0),
                 "max_drift_rad": (0.000001, .1), "max_speed_rad_s": (0.000001, 1.0)}


def stationary_policy(options=None):
    require(options is None or isinstance(options, dict), "Stationary policy must be a mapping")
    options = options or {}
    require(not (set(options) - set(POLICY)), "Unsupported stationary policy field")
    policy = dict(POLICY)
    for key, val in options.items():
        val = number(val, "Invalid stationary threshold: " + key)
        lower, upper = POLICY_BOUNDS[key]
        require(lower <= val <= upper, "Stationary threshold outside allowed range: " + key)
        if key == "min_packets":
            require(float(val).is_integer(), "min_packets must be an integer")
            val = int(val)
        policy[key] = val
    require(policy["max_gap_s"] < policy["window_s"], "Gap threshold must be smaller than capture window")
    return policy


def bounded(path, limit):
    require(not path.is_symlink() and path.is_file(), "File unavailable")
    with path.open("rb") as f:
        data = f.read(limit+1)
    require(len(data)<=limit, "File exceeds size limit")
    return data


class PassiveTap:
    def __init__(self):
        self.buffers = {side: deque(maxlen=128) for side in SIDES}
        self.lock = threading.Lock()
        self.last_motion_ns = {side: 0 for side in SIDES}

    def observe(self, packet):
        with self.lock:
            self.buffers[packet["side"]].append(packet)

    def motion(self, side):
        with self.lock:
            self.last_motion_ns[side] = time.monotonic_ns()

    def capture(self, sides, status, policy_override=None):
        policy = stationary_policy(policy_override)
        state = status()
        with self.lock:
            now = time.monotonic_ns()
            buffers = {s: list(self.buffers[s]) for s in sides}
            moving = max(self.last_motion_ns.values())
        require(not any((state.get(s,{}).get("state") or {}).get("motion_state",0) != 0 for s in SIDES),
                "Cached robot state indicates motion")
        require(all(state.get(s,{}).get("feedback_status") == "LIVE" for s in sides), "Missing fresh idle robot state")
        windows, quality = {}, {}
        common_end = min((ps[-1]["received_monotonic_ns"] if ps else now) for ps in buffers.values())
        for side, packets in buffers.items():
            seen = set()
            selected = []
            for p in reversed(packets):
                if p["packet_sequence_index"] in seen:
                    continue
                selected.append(p)
                seen.add(p["packet_sequence_index"])
                if common_end-p["received_monotonic_ns"] >= policy["window_s"]*1e9:
                    break
            selected.reverse()
            n = len(selected)
            label = f"{side} ({n} unique packets): "
            require(n>=policy["min_packets"], label+"insufficient unique packets")
            stamp = np.array([p["received_monotonic_ns"] for p in selected],dtype=np.int64)
            dt = np.diff(stamp)/1e9
            require(0 <= now-stamp[-1] <= policy["max_age_s"]*1e9, label+"stale packet window")
            require(stamp[-1]-stamp[0]>=policy["window_s"]*1e9 and np.all(dt>0) and max(dt)<=policy["max_gap_s"], label+"short window or packet gap")
            require(moving < stamp[0], label+"motion during window")
            q = np.array([vec(p["actual_joints_rad"],6) for p in selected])
            drift = float(np.ptp(q,axis=0).max())
            speed = float(np.max(np.abs(np.diff(q,axis=0))/dt[:,None]))
            require(drift<=policy["max_drift_rad"] and speed<=policy["max_speed_rad_s"], label+"joint motion")
            quality[side] = {"unique_packets": n, "duration_s": float(sum(dt)), "max_gap_s": float(max(dt)),
                             "drift_rad": drift, "speed_rad_s": speed}
            windows[side] = selected
        return {"packets": windows, "quality": quality, "policy": policy, "robot_state": state}


def compact_run(raw, method):
    require(method in ("V1","V2"), "Static trials require stationary capture")
    packets = raw["actual_packets"]
    require(isinstance(packets,dict) and set(packets)<=set(SIDES) and
            0<sum(len(v) for v in packets.values())<=12000, "Invalid packet count or side")
    for side, series in packets.items():
        for p in series:
            require(p.get("side", side)==side, "Packet side mismatch")
            vec(p["actual_joints_rad"],6)
    result = {"packets": packets, "run_id": raw.get("run_id"), "model_revision":
              raw.get("plan_snapshot",{}).get("calibration",{}).get("model_revision")}
    if method == "V2":
        plan = raw["planned"]["samples"]
        require(3<=len(plan)<=30000,"Invalid plan length")
        result.update(plan={"t": [p["time_from_start_s"] for p in plan],
                            **{s:[p[s]["joints_rad"] for p in plan] for s in SIDES}},
                      start_ns=raw["execution"]["start_time_unix_ns"])
    return result


def config_check(method, c):
    require(method in METHODS and isinstance(c,dict), "Unknown method/configuration")
    if "stationary_policy" in c:
        require(method != "V2", "V2 is an offline run analysis, not stationary capture")
        c["stationary_policy"] = stationary_policy(c["stationary_policy"])
    repeat = c.get("repetitions",1)
    require(type(repeat) is int and 1<=repeat<=50, "Repetitions must be 1–50")
    c["repetitions"] = repeat
    criterion = c.get("criterion")
    if criterion is not None:
        if criterion.get("declared_at_ms") is not None:
            number(criterion["declared_at_ms"],"Invalid criterion timestamp")
        require(number(criterion["value"],"Missing numeric limit")>=0 and criterion["unit"]==METRICS[method][criterion.get("metric", next(iter(METRICS[method])))]
                and str(criterion.get("source") or "").strip() and str(criterion.get("uncertainty") or "").strip(),
                "Criterion requires matching unit, evidence basis and uncertainty")
    if method=="V4":
        require(c.get("side") in SIDES and isinstance(c.get("jigs"),dict) and 0<len(c["jigs"])<=50,"Select arm and jig")
    if method=="V5":
        training=c.get("training_sockets",[])
        require(isinstance(training,list) and not set(training).intersection(c.get("sockets",[])), "Separate training sockets")
        require(isinstance(c.get("sockets"),list) and 0<len(c["sockets"])<=100,"List physical socket IDs")


def check_slot(s,t,skip=None):
    m,c = s["method"],s["config"]
    if m not in ("V3","V4","V5"):
        return
    require(type(t["repeat"]) is int and 1<=t["repeat"]<=c["repetitions"],"Trial outside configured repetitions")
    fields = {"V3":("pose","direction"),"V4":("side","jig","socket"),"V5":("side","role","socket")}[m]
    require(all(isinstance(t.get(k),str) and t[k].strip() for k in fields),"Missing trial identity")
    if m=="V4":
        require(t["jig"] in c["jigs"] and t["side"]==c["side"] and t["socket"] in ("A","B"),"Invalid jig trial")
    if m=="V5":
        require(t["side"] in SIDES and t["role"] in ("training","validation") and t["socket"] in (c["sockets"] if t["role"]=="validation" else c.get("training_sockets",[])),"Invalid common point")
        other_roles = {x["trial"]["role"] for x in s["samples"] if not x.get("excluded") and x["trial"]["socket"]==t["socket"]}
        require(not other_roles or other_roles=={t["role"]},"Training and validation sockets must be disjoint")
    active = [x["trial"] for x in s["samples"] if not x.get("excluded") and x["key"]!=skip and x["trial"]["repeat"]==t["repeat"]]
    require(not any(all(a[k]==t[k] for k in fields) for a in active),"Trial already captured")
    if m=="V5" and t["side"]=="right":
        require(any(a["side"]=="left" and a["socket"]==t["socket"] and a["role"]==t["role"] for a in active),
                "Capture left first at this socket")


def check_sequence(session, trial, evidence):
    if session["method"] != "V5":
        return
    side = trial["side"]
    other = "right" if side == "left" else "left"
    for x in session["samples"]:
        t = x["trial"]
        if x.get("excluded") or t["side"] != other:
            continue
        if all(t[k] == trial[k] for k in ("socket", "role", "repeat")):
            left, right = (evidence, x) if side == "left" else (x, evidence)
            require(right["packets"]["right"][0]["received_at_ms"] > left["packets"]["left"][-1]["received_at_ms"],
                    "Capture left first, then a separate right window. Archive the outdated partner to retry.")


class ValidationSessionStore:
    def __init__(self, root=APP/"validation_sessions", runs=APP/"experiment_runs"):
        self.root, self.runs = Path(root), Path(runs)

    def path(self, sid):
        require(isinstance(sid,str) and re.fullmatch("[0-9a-f]{32}",sid),"Invalid session ID")
        return self.root/(sid+".json")

    def read(self,sid):
        return json.loads(bounded(self.path(sid),MAX_SESSION))

    def save(self,s):
        data = json.dumps(s,allow_nan=False,separators=(",",":")).encode()
        require(len(data)<=MAX_SESSION,"Session capacity exceeded")
        self.root.mkdir(parents=True,exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=self.root,delete=False) as f:
            try:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
                os.replace(f.name,self.path(s["id"]))
            finally:
                Path(f.name).unlink(missing_ok=True)
        # Derived, bounded UI index. The full immutable-evidence session remains
        # authoritative if a crash occurs between these two atomic writes.
        summary = {
            "id": s["id"], "method": s["method"],
            "created_ns": s["created_ns"], "updated_ns": time.time_ns(),
            "sample_count": len(s["samples"]),
            "active_samples": sum(not row.get("excluded", False) for row in s["samples"]),
            "state": (s.get("analysis") or {}).get("acceptance")
                or ("CAPTURED_NOT_ANALYZED" if s["samples"] else "NOT_STARTED"),
            "data_status": (s.get("analysis") or {}).get("data_status"),
        }
        metadata = json.dumps(summary, allow_nan=False, separators=(",", ":")).encode()
        try:
            with tempfile.NamedTemporaryFile(dir=self.root, delete=False) as f:
                try:
                    f.write(metadata)
                    f.flush()
                    os.fsync(f.fileno())
                    os.replace(f.name, self.path(s["id"]).with_suffix(".meta.json"))
                finally:
                    Path(f.name).unlink(missing_ok=True)
        except OSError:
            pass  # Status index is optional; never fail a saved evidence operation.

    def overview(self):
        result = {method: {"state": "NO_SESSION", "sample_count": 0,
                            "active_samples": 0, "session_id": None}
                  for method in METHODS}
        if not self.root.is_dir():
            return result
        for file in self.root.glob("*.meta.json"):
            sid = file.name.removesuffix(".meta.json")
            if not re.fullmatch("[0-9a-f]{32}", sid) or not self.path(sid).is_file():
                continue
            try:
                meta = json.loads(bounded(file, 8192))
            except (ValueError, OSError, json.JSONDecodeError):
                continue
            method = meta.get("method")
            if method not in METHODS or meta.get("id") != sid:
                continue
            current = result[method]
            if meta.get("updated_ns", 0) > current.get("updated_ns", 0):
                result[method] = {
                    "state": meta.get("state") or "UNKNOWN",
                    "sample_count": meta.get("sample_count", 0),
                    "active_samples": meta.get("active_samples", 0),
                    "data_status": meta.get("data_status"),
                    "session_id": sid,
                    "updated_ns": meta.get("updated_ns"),
                }
        return result

    def create(self,method,c):
        config_check(method,c)
        require(len(json.dumps(c))<=65536,"Configuration too large")
        s = {"schema_version":1,"engine_version":"validation/1","id":uuid.uuid4().hex,"method":method,"config":c,
             "created_ns":time.time_ns(),"snapshot":snapshot(),"samples":[],"analysis":None}
        self.save(s)
        return s

    @contextmanager
    def edit(self,sid):
        path = self.path(sid)
        require(path.is_file(),"Session unavailable")
        with path.with_suffix(".lock").open("a") as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            s = self.read(sid)
            yield s
            self.save(s)

    def run(self,rid,method):
        require(isinstance(rid,str) and RUN.fullmatch(rid),"Invalid run ID")
        folder = self.runs/rid
        require(not folder.is_symlink(),"Invalid run directory")
        data = bounded(folder/"raw.json", MAX_RUN)
        raw = json.loads(data)
        require(raw.get("run_id")==rid,"Recorded run ID mismatch")
        return compact_run(raw, method) | {"raw_sha256":hashlib.sha256(data).hexdigest()}

    def add(self,sid,key,trial,produce):
        require(isinstance(key,str) and KEY.fullmatch(key),"Invalid capture key")
        with self.edit(sid) as s:
            existing = next((x for x in s["samples"] if x["key"]==key), None)
            if existing:
                require(existing.get("original_trial",existing["trial"])==trial, "Capture key already assigned to another trial")
                return s
            require(len(s["samples"])<1000,"Trial limit exceeded")
            check_slot(s,trial)
            evidence = produce(s)
            if "raw_sha256" in evidence:
                require(not any(not x.get("excluded") and
                                ((evidence.get("run_id") and x.get("run_id")==evidence["run_id"]) or
                                 evidence["raw_sha256"]==x.get("raw_sha256")) for x in s["samples"]), "Run already imported")
            if "quality" in evidence:
                for side, ps in evidence["packets"].items():
                    for old in s["samples"]:
                        previous = old["packets"].get(side,[])
                        if previous and "quality" in old:
                            require(ps[0]["received_monotonic_ns"]>previous[-1]["received_monotonic_ns"] or
                                    ps[0]["received_at_ms"]>previous[-1]["received_at_ms"]+5000,
                                    "Stationary capture window already used")
                check_sequence(s, trial, evidence)
            s["samples"].append({"key":key,"trial":trial,"original_trial":dict(trial),"saved_ns":time.time_ns(),"excluded":False,**evidence})
            s["analysis"]=None
            return s
