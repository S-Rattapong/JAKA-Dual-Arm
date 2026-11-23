"""Validation-only FastAPI routes. Dependencies are a packet buffer and a cache reader."""
import asyncio
import csv
import io
import json
import hashlib
from threading import BoundedSemaphore

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response
from starlette.concurrency import run_in_threadpool

try:
    from .validation_engine import METHODS, SIDES, analyze, require, snapshot
    from .validation_store import RUN, ValidationSessionStore, check_slot, check_sequence, compact_run
except ImportError:
    from validation_engine import METHODS, SIDES, analyze, require, snapshot
    from validation_store import RUN, ValidationSessionStore, check_slot, check_sequence, compact_run

UPLOAD = 48*1024**2


def public(s):
    result = {k:v for k,v in s.items() if k not in ("snapshot","samples","analysis")}
    result["model_revision"] = json.loads(s["snapshot"]["calibration"])["revision"]
    result["samples"] = [{**{k:v for k,v in x.items() if k not in ("packets","plan","robot_state")},
                          "counts":{a:len(p) for a,p in x["packets"].items()},
                          "packet_preview":{a:p[-1] for a,p in x["packets"].items() if p}} for x in s["samples"]]
    if s["analysis"]:
        result["analysis"] = dict(s["analysis"],rows=s["analysis"]["rows"][:100])
        result["analysis"]["row_count"] = len(s["analysis"]["rows"])
    return result


def plot(s, run_index):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    a = s["analysis"]
    rows = a["rows"]
    require(rows, "No evaluable rows to plot")
    fig = Figure(figsize=(8,6),tight_layout=True)
    if s["method"]=="V2":
        require(0<=run_index<len(rows),"Invalid run index")
        row = rows[run_index]
        axes = fig.subplots(3)
        for axis,side in zip(axes[1:],SIDES):
            d = row[side]
            axes[0].plot(d["grid_ms"],d["rmse_rad"],label=side)
            ov = d["overlay"]
            for key,style in (("actual","-"),("plan",":"),("shifted","--")):
                times, positions = [], []
                for i,t in enumerate(ov["t"]):
                    if i and ov["segment"][i]!=ov["segment"][i-1]:
                        times.append(float("nan"))
                        positions.append([float("nan")]*6)
                    times.append(t)
                    positions.append(ov[key][i])
                for j,values in enumerate(zip(*positions)):
                    axis.plot(times,values,style,color=f"C{j}",linewidth=.8)
            axis.set(xlabel="Host time from plan start (s)",ylabel=side+" joints (rad)",
                     title="Actual (solid), plan (dotted), shifted plan (dashed)")
        axes[0].set(xlabel="Shift (ms)",ylabel="Joint RMSE (rad)")
        axes[0].legend()
    elif s["method"]=="V1":
        p,r = fig.subplots(2)
        for side in SIDES:
            data = [x for x in rows if x["side"]==side]
            if data:
                t = [(x["received_monotonic_ns"]-data[0]["received_monotonic_ns"])/1e9 for x in data]
                for ax,key in [(p,"error_mm"),(r,"orientation_deg")]:
                    ax.scatter(t,[x[key] for x in data],s=5,label=side)
        p.set_ylabel("Position error (mm)")
        r.set(xlabel="Host receipt time from first side packet (s)",ylabel="Orientation error (deg)")
        p.legend()
    else:
        ax = fig.subplots()
        for i,r in enumerate(rows,1):
            if "error_interval_mm" in r:
                ax.plot([i,i],r["error_interval_mm"],"|-",color="C1")
            else:
                ax.scatter(i,r["error_mm"],color="gray" if r.get("role")=="training" else "C0")
        ax.axhline(0,color="gray",linewidth=.5)
        ax.set(xlabel="Trial",ylabel="Error (mm)",title=METHODS[s["method"]])
    buf = io.BytesIO()
    FigureCanvasAgg(fig).print_png(buf)
    return buf.getvalue()


def export_csv(s):
    a = s["analysis"]
    entries = []
    for x in s["samples"]:
        entries.append({"record":"sample","key":x["key"],"excluded":x["excluded"],"reason":x.get("reason"),
                        "run_id":x.get("run_id"),"counts":{k:len(v) for k,v in x["packets"].items()},**x["trial"]})
    for row in [*a["rows"],*a.get("training",{}).get("rows",[])]:
        entries.append({"record":"result",**row})
    fields = list(dict.fromkeys(k for r in entries for k in r)) or ["record"]
    out = io.StringIO()
    w = csv.DictWriter(out,fieldnames=fields)
    w.writeheader()
    for r in entries:
        w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in r.items()})
    return out.getvalue()


def guarded(fn,*args):
    try:
        return fn(*args)
    except (KeyError,IndexError,ValueError,TypeError,AttributeError,RecursionError,OverflowError) as exc:
        raise HTTPException(422,f"Validation: {exc}") from exc


def validation_router(tap, status, store=None):
    store = store or ValidationSessionStore()
    router = APIRouter(prefix="/api/validation")

    capacity = BoundedSemaphore(2)

    @router.get("/catalog")
    def catalog():
        snap = snapshot()
        cal = json.loads(snap["calibration"])
        return {"methods": METHODS, "model_revision": json.loads(snap["model_metadata"])["calibration_revision"],
                "urdf_sha256": snap["urdf_sha256"], "calibration_revision": cal["revision"], "source": cal["source"]}

    @router.get("/overview")
    def overview():
        return store.overview()

    @router.get("/runs")
    def runs():
        return sorted((p.name for p in store.runs.iterdir() if RUN.fullmatch(p.name) and not p.is_symlink()
                       and (p/"raw.json").is_file()),reverse=True) if store.runs.is_dir() else []

    @router.get("/sessions")
    def sessions():
        return sorted([p.stem for p in store.root.glob("*.json") if len(p.stem)==32],
                      key=lambda sid:store.path(sid).stat().st_mtime,reverse=True)

    @router.get("/sessions/{sid}")
    def session(sid:str):
        return public(guarded(store.read,sid))

    def action(sid,op,b):
        if op=="create":
            return public(store.create(b["method"],b["config"]))
        if op in ("capture","import","run"):
            trial = b.get("trial",{})
            def produce(s):
                m = s["method"]
                if op=="capture":
                    require(m!="V2","V2 requires an original plan")
                    sides = SIDES if m=="V3" else [trial["side"]]
                    require(all(a in SIDES for a in sides),"Unknown arm")
                    current_model = snapshot()
                    require(all(current_model[k] == s["snapshot"][k] for k in
                                ("urdf_sha256", "calibration_sha256", "model_metadata_sha256")),
                            "Model or calibration changed since configuration. Start a new session for new captures.")
                    return tap.capture(sides,status,s["config"].get("stationary_policy")) | {"model_revision":json.loads(s["snapshot"]["calibration"])["revision"]}
                if op=="run":
                    return store.run(b["run_id"],m)
                raw = b["raw"]
                return compact_run(raw,m) | {"raw_sha256": hashlib.sha256(json.dumps(raw,sort_keys=True,separators=(",",":")).encode()).hexdigest()}
            return public(store.add(sid,b["key"],trial,produce))
        with store.edit(sid) as s:
            if op=="analysis":
                s["analysis"]=analyze(s)
            elif op in ("exclude","unexclude","retry","trial"):
                x = next((x for x in s["samples"] if x["key"]==b["key"]),None)
                require(x is not None,"Unknown sample")
                if op=="trial":
                    require(s["method"]=="V3","Manual gauge readings apply to V3")
                    x["trial"].update({k:b.get(k) for k in ("measured_mm","interval_mm")})
                elif op=="unexclude":
                    check_slot(s,x["trial"],x["key"])
                    require(not any(y is not x and x.get("run_id") and y.get("run_id")==x["run_id"] and not y.get("excluded") for y in s["samples"]),"Run already active")
                    check_sequence(s,x["trial"],x)
                    x["excluded"]=False
                else:
                    reason=b.get("reason","").strip()
                    require(0<len(reason)<=500,"Archive reason required")
                    x.update(excluded=True,reason=reason)
                x.setdefault("history",[]).append({"action":op,"details":b})
                s["analysis"]=None
            else:
                raise ValueError("Unknown validation action")
        return public(s)

    async def dispatch(request,sid,op):
        limit = UPLOAD if op=="import" else 128*1024
        body = bytearray()
        async def read_body():
            async for chunk in request.stream():
                if len(body)+len(chunk)>limit:
                    raise HTTPException(413,"Validation input too large")
                body.extend(chunk)
        if not capacity.acquire(blocking=False):
            raise HTTPException(429,"Validation busy. Retry this action.")
        try:
            await asyncio.wait_for(read_body(), timeout=30)
            return await run_in_threadpool(lambda:guarded(action,sid,op,json.loads(body or b"{}")))
        except asyncio.TimeoutError as exc:
            raise HTTPException(408,"Validation upload timed out") from exc
        finally:
            capacity.release()

    @router.post("/sessions")
    async def create(request:Request):
        return await dispatch(request,None,"create")

    @router.post("/sessions/{sid}/{op}")
    async def mutate(sid:str,op:str,request:Request):
        if op=="create":
            raise HTTPException(404)
        return await dispatch(request,sid,op)

    @router.get("/sessions/{sid}/export/{fmt}")
    def export(sid:str,fmt:str,run_index:int=0):
        s = guarded(store.read,sid)
        if fmt=="json":
            data,mime=json.dumps(s,ensure_ascii=False),"application/json"
        else:
            if not s["analysis"]:
                raise HTTPException(409,"Analyze this session first")
            if fmt=="csv":
                data,mime=export_csv(s),"text/csv"
            elif fmt=="png":
                data,mime=guarded(plot,s,run_index),"image/png"
            else:
                raise HTTPException(404)
        return Response(data,media_type=mime,headers={"Content-Disposition":f'inline; filename="{s["method"]}-{sid}.{fmt}"',
                                                    "Cache-Control":"no-store"})
    return router
