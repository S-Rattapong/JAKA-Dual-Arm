// User-driven, cache-only validation workflow in the existing operator pane.
const panel = document.querySelector("#validationPanel");
const home = document.querySelector("#validationHome");
const work = document.querySelector("#validationWork");
const message = document.querySelector("#validationMessage");
const base = "/api/validation";
const methods = {
  V1: ["FK cross-check", "เทียบ FK กับ TCP ในแพ็กเก็ตเดียวกัน ต้องยืนยันฟิลด์ หน่วย เฟรม Tool และรุ่นโมเดลก่อนคำนวณ ผลบอกความสอดคล้องของโมเดล"],
  V2: ["Host time shift", "ใช้ run ที่มีแผนเดิมและแพ็กเก็ตจริง วัดเวลาหน่วงบน host ค่า τ บวกหมายถึง Actual ตามหลัง"],
  V3: ["Surface gap", "วัดช่องว่างตาม normal ของผิวซ้าย เตรียมจุดและ normal สองผิวใน J6 และเกจวัดช่องว่าง"],
  V4: ["Single-arm touch", "แตะ A และ B ด้วยแขนเดียวกัน เตรียมระยะศูนย์กลางที่วัดอย่างอิสระ พร้อมค่า uncertainty"],
  V5: ["Common-point agreement", "แตะ socket เดียวกันตามลำดับ ซ้ายแล้วขวา เตรียมอย่างน้อย 4 จุดที่กระจายในสามมิติ และแยกจุดฝึก"],
};
let current, method, revision, retryTrial, busy = false, step = 1;
const armNames = ["left","right"];
const get = id => document.getElementById("v-"+id);
const storageKey = "jaka.validation.session";
function el(tag, text, parent, cls) {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (cls) e.className = cls;
  parent?.append(e);
  return e;
}
function group(title, parent=work) {
  const f=el("fieldset",undefined,parent);
  el("legend",title,f);
  return el("div",undefined,f,"v-grid");
}
function input(parent,id,label,options={}) {
  const l=el("label",label,parent);
  const e=el(options.items ? "select":"input",undefined,l);
  e.id="v-"+id;
  if (options.items) for (const v of options.items) el("option",v,e).value=v;
  else {e.type=options.type || "text"; if(e.type==="number") e.step="any";}
  if(options.value!==undefined)e.value=options.value;
  if(options.placeholder)e.placeholder=options.placeholder;
  return e;
}
const val = id => get(id)?.value.trim() || "";
const num = id => val(id)==="" ? null : Number(val(id));
function vector(id) {
  if(!val(id))return null;
  const parts=val(id).split(/[ ,]+/).map(Number);
  if(parts.length!==3 || !parts.every(Number.isFinite))throw Error("ต้องระบุเวกเตอร์ 3 ค่า • "+id);
  return parts;
}
function button(parent,label,fn) {
  const e=el("button",label,parent); e.type="button";
  e.onclick=()=>run(fn); return e;
}
async function run(fn) {
  if(busy)return;
  busy=true; panel.setAttribute("aria-busy","true"); message.textContent="กำลังประมวลผล…";
  try {await fn(); if(message.textContent==="กำลังประมวลผล…")message.textContent="";}
  catch(err){message.textContent=String(err.message || err);}
  finally{busy=false;panel.removeAttribute("aria-busy");}
}
async function api(path, body) {
  const res=await fetch(base+path,body===undefined ? {cache:"no-store"} :
    {method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
  if(!res.ok) {
    const d=await res.json().catch(()=>({}));
    throw Error(res.status===404 ? "Validation API ยังไม่พร้อมใน backend ที่กำลังรัน" : (d.detail || res.statusText));
  }
  try{return await res.json();}catch{throw Error("Validation API ไม่ได้ส่งข้อมูล session ที่อ่านได้");}
}
async function selectMethod(m) {
  current=null; retryTrial=null; method=m; step=1; revision=null; home.hidden=true; render();
  if(m==="V1" || m==="V5"){
    const model=await api("/catalog");
    revision=model.model_revision;
    details("World calibration / model · "+revision,{revision,urdf_sha256:model.urdf_sha256,
      calibration_revision:model.calibration_revision,source:model.source},get("model"),true);
  }
}
function homeView() {
  home.hidden=false; work.replaceChildren(); home.replaceChildren();
  const cards=el("div",undefined,home,"v-grid");
  const sourceNotes = {
    V1:"ข้อมูล: Joint Actual + Controller TCP จากแพ็กเก็ตเดียวกัน (ต้องยืนยัน Frame/Tool)",
    V2:"ข้อมูล: Planned Trajectory + Actual Joint Log เดิม (Host time shift)",
    V3:"ข้อมูล: Joint Feedback ขณะนิ่งสองแขน + ค่า Gap จากเกจจริง",
    V4:"ข้อมูล: Joint Feedback ขณะนิ่งทีละ Socket A/B + ระยะ Jig อ้างอิง",
    V5:"ข้อมูล: Joint Feedback ซ้าย/ขวาแตะ Socket เดียวกันตามลำดับ + World Calibration"
  };
  const statuses={};
  for (const [id,[name,description]] of Object.entries(methods)) {
    const card=el("div",undefined,cards,"v-method-card");
    button(card,id+" · "+name,()=>selectMethod(id)).className="v-card";
    el("p",sourceNotes[id],card,"v-method-source");
    el("p",description,card,"v-method-desc");
    statuses[id]=el("small","ล่าสุด: กำลังตรวจสอบ…",card,"v-method-status");
  }
  api("/overview").then(data=>{
    if(home.hidden)return;
    for (const id of Object.keys(statuses)) {
      const entry=data[id]||{};
      const status=entry.state||"NO_SESSION";
      statuses[id].textContent="ล่าสุด: "+(status==="NO_SESSION"?"ยังไม่มี Session":status)
        +(entry.active_samples ? " · "+entry.active_samples+" samples" : "");
      if (entry.session_id) statuses[id].title="Session "+entry.session_id;
    }
  }).catch(()=>{for(const node of Object.values(statuses))node.textContent="ล่าสุด: Backend Validation API ยังไม่พร้อม";});
  const actions=el("div",undefined,home,"v-actions");
  if(localStorage.getItem(storageKey))button(actions,"เปิดงานล่าสุด",()=>resume(localStorage.getItem(storageKey)));
  button(actions,"เลือก session ที่บันทึก",async()=>{
    const ids=await api("/sessions");
    if(!ids.length){message.textContent="ยังไม่มี session";return;}
    const list=input(home,"sessions","Session ID",{items:ids});
    button(home,"เปิด session",()=>resume(list.value));
  });
}
async function resume(id) {
  retryTrial=null;current=await api("/sessions/"+id);method=current.method;step=current.analysis ? 4 : current.samples.length ? 3 : 2;
  localStorage.setItem(storageKey,id);home.hidden=true;render();
}
function render() {
  work.replaceChildren();
  const nav=el("div",undefined,work,"v-actions");
  button(nav,"‹ Validation",homeView);
  el("h3",method+" · "+methods[method][0],work);
  el("p",methods[method][1],work);
  const steps=el("div",undefined,work,"v-actions");
  ["Configure","Capture / import","Inspect","Analyze / export"].forEach((v,i)=>{
    const b=button(steps,(i+1)+" "+v,()=>{step=i+1;render();});
    b.disabled=(!current && i!==0) || step===i+1;
    if(step===i+1)b.setAttribute("aria-current","step");
  });
  if(step===1)configure();
  if(step===2)collect();
  if(step===3)inspect();
  if(step===4)results();
}
function toolFields(g,a){
  const frame=input(g,a+"toolframe",a+" · Coordinate frame",{items:["","J6","mounted"]});
  const fields=[input(g,a+"toolname","Mounted tool ID"),input(g,a+"mount","Tool origin in J6 (m)"),
    input(g,a+"mountrot","Tool RPY in J6 (rad)")];
  function change(){for(const f of fields)f.parentNode.hidden=frame.value!=="mounted";}
  frame.onchange=change;change();
}
function toolDefinition(a){
  return {tool_frame:val(a+"toolframe"),tool_id:val(a+"toolname"),
    j6_from_tool:{translation_m:vector(a+"mount"),rotation_rpy_rad:vector(a+"mountrot")}};
}
function configure() {
  if(current){details("Configuration snapshot",current.config,work,true);return;}
  const model=el("div",undefined,work);model.id="v-model";
  const f=group("ตั้งค่าก่อนบันทึก");
  const unit=method==="V2"?"ms":"mm";
  const metric={V1:"position norm",V2:"|τL − τR|",V3:"|measured − FK|",V4:"|FK − reference|",V5:"validation XYZ norm"}[method];
  const choices={V1:["position_norm","orientation"],V2:["abs_delta_tau","abs_tau_left","abs_tau_right"]}[method];
  const score=choices?input(f,"metric","Acceptance metric",{items:choices}):null;
  const limit=input(f,"limit","ขีดจำกัดสูงสุด "+metric+" ("+unit+")",{type:"number"});
  if(score)score.onchange=()=>{limit.parentNode.firstChild.textContent="ขีดจำกัดสูงสุด "+score.value+" ("+(score.value==="orientation"?"deg":unit)+")";};
  input(f,"basis","แหล่งที่มาของเกณฑ์");input(f,"uncertainty","Uncertainty และหน่วย / ข้อจำกัด");
  if(["V1","V2"].includes(method))input(f,"declared","วันที่กำหนดเกณฑ์ตามเอกสารเดิม (สำหรับ imported run)",{type:"datetime-local"});
  if(["V3","V4","V5"].includes(method))input(f,"repetitions","จำนวนครั้งต่อชุด",{type:"number",value:5});
  if(method==="V1"){
    el("strong","MISSING_INFO · Controller raw pose ยังไม่ยืนยันความหมาย",work);
    for(const a of armNames){
      const g=group(a+" · Controller TCP provenance");
      input(g,a+"field","Controller field",{items:["","controller_actual_position_raw","controller_position_raw"]});
      for(const [k,title,items] of [["length","Translation unit",["m","mm"]],["angle","Angle unit",["deg","rad"]],
        ["orientation","Roll / pitch / yaw order",["RzRyRx"]],["frame","Reference frame",["base","world","user"]]])
        input(g,a+k,title,{items:["",...items]});
      input(g,a+"fid","Base / User frame ID");input(g,a+"tid","Active Tool / TCP ID");
      input(g,a+"tcp","TCP origin in J6 (m)",{placeholder:"x, y, z"});
      input(g,a+"rpy","TCP RPY in J6 (rad)");
      input(g,a+"user","User origin in world (m)");input(g,a+"urpy","User RPY in world (rad)");
      input(g,a+"source","หลักฐานการตรวจ field, calibration และ Tool");
      input(g,a+"verified","ยืนยันว่า field คือ TCP และโมเดลตรงกับการวัด",{items:["","ยืนยัน"]});
    }
  } else if(method==="V2"){
    const g=group("Shift search");
    for(const [k,title,v] of [["tau_min_ms","Minimum τ (ms)",-300],["tau_max_ms","Maximum τ (ms)",300],
      ["tau_step_ms","Step (ms)",5],["gap_ms","Maximum gap (ms)",250]])input(g,k,title,{type:"number",value:v});
  } else if(method==="V3"){
    for(const a of armNames){
      const g=group(a+" · Reference surface in tool frame");
      toolFields(g,a);input(g,a+"point","Surface point XYZ (m)");input(g,a+"normal","Surface normal XYZ");input(g,a+"source","แหล่งอ้างอิงผิว");
    }
    const g=group("Gauge");input(g,"brand","Brand / instrument ID");input(g,"type","Gauge type");
    for(const [k,title] of [["resolution_mm","Resolution (mm)"],["uncertainty_mm","Uncertainty (mm)"],["parallel_deg","Max plane angle (deg)"]])
      input(g,k,title,{type:"number"});
  } else {
    const g=group("Probe center in tool frame");
    for(const a of armNames){toolFields(g,a);input(g,a+"point",a+" XYZ (m)");input(g,a+"source",a+" offset source");}
    if(method==="V4"){
      input(g,"side","Arm",{items:armNames});
      const jigs=el("div",undefined,work);let n=0;
      function jig(){const j=group("Jig "+(++n),jigs);j.dataset.jig=n;input(j,"jig"+n,"Jig ID");
        input(j,"dist"+n,"Measured A–B (mm)",{type:"number"});input(j,"unc"+n,"Uncertainty (mm)",{type:"number"});
        input(j,"inst"+n,"Reference instrument / record");}
      jig();button(work,"เพิ่มชุด jig",jig);
    } else {input(g,"sockets","Validation socket IDs (คั่นด้วย comma)");input(g,"training","Training socket IDs (ถ้ามี)");}
  }
  if(method!=="V2") {
    const section=el("details",undefined,work);
    el("summary","Stationary feedback quality · ปรับได้ก่อนเริ่ม session",section);
    const fields=el("div",undefined,section,"v-grid");
    for(const [key,label,value] of [
      ["window_s","หน้าต่างนิ่งอย่างน้อย (s)",1],
      ["min_packets","จำนวนแพ็กเก็ตใหม่ขั้นต่ำ",6],
      ["max_age_s","อายุข้อมูลล่าสุดสูงสุด (s)",.25],
      ["max_gap_s","ช่องว่างระหว่างแพ็กเก็ตสูงสุด (s)",.25],
      ["max_drift_rad","Joint drift สูงสุด (rad)",.002],
      ["max_speed_rad_s","Joint speed สูงสุด (rad/s)",.02]
    ])input(fields,"quality_"+key,label,{type:"number",value});
  }
  button(work,"บันทึก configuration และเริ่ม session",async()=>{
    if(method==="V1" && (!revision || revision!==(await api("/catalog")).model_revision)){
      throw Error("ตรวจสอบ model revision อีกครั้งก่อนยืนยัน");
    }
    const c={repetitions:num("repetitions")??1,measurement_uncertainty:val("uncertainty") || "UNKNOWN"};
    if(method!=="V2") {
      c.stationary_policy={};
      for(const key of ["window_s","min_packets","max_age_s","max_gap_s","max_drift_rad","max_speed_rad_s"])
        c.stationary_policy[key]=num("quality_"+key);
    }
    if(val("limit") || val("basis"))c.criterion={value:num("limit"),unit,source:val("basis"),uncertainty:val("uncertainty")};
    if(c.criterion && val("metric")){c.criterion.metric=val("metric");if(val("metric")==="orientation")c.criterion.unit="deg";}
    if(c.criterion && val("declared"))c.criterion.declared_at_ms=new Date(val("declared")).getTime();
    if(method==="V1"){
      c.mapping={};
      for(const a of armNames)c.mapping[a]={field:val(a+"field"),length_unit:val(a+"length"),angle_unit:val(a+"angle"),
        orientation:val(a+"orientation"),frame:val(a+"frame"),frame_id:val(a+"fid"),tool_id:val(a+"tid"),
        source:val(a+"source"),verified:!!val(a+"verified"),model_revision:revision,
        j6_from_tcp:{translation_m:vector(a+"tcp"),rotation_rpy_rad:vector(a+"rpy")},
        world_from_user:{translation_m:vector(a+"user"),rotation_rpy_rad:vector(a+"urpy")}};
    } else if(method==="V2"){for(const k of ["tau_min_ms","tau_max_ms","tau_step_ms","gap_ms"])c[k]=num(k);}
    else if(method==="V3"){
      c.surfaces={};for(const a of armNames)c.surfaces[a]={...toolDefinition(a),point_m:vector(a+"point"),normal:vector(a+"normal"),source:val(a+"source")};
      c.parallel_deg=num("parallel_deg");c.gauge={brand:val("brand"),type:val("type"),resolution_mm:num("resolution_mm"),uncertainty_mm:num("uncertainty_mm")};
    } else {
      c.probes={};for(const a of armNames)c.probes[a]={...toolDefinition(a),point_m:vector(a+"point"),source:val(a+"source")};
      if(method==="V4"){
        c.side=val("side");c.jigs={};
        for(const j of work.querySelectorAll("[data-jig]")){
          const i=j.dataset.jig,id=val("jig"+i);if(!id || Object.hasOwn(c.jigs,id))throw Error("ระบุ Jig ID ที่ไม่ซ้ำ");
          c.jigs[id]={distance_mm:num("dist"+i),uncertainty_mm:num("unc"+i),source:val("inst"+i)};
        }
      } else {
        const ids=k=>val(k).split(",").map(s=>s.trim()).filter(Boolean);
        c.sockets=ids("sockets");c.training_sockets=ids("training");
      }
    }
    current=await api("/sessions",{method,config:c});localStorage.setItem(storageKey,current.id);step=2;render();
  });
}
function details(title,obj,parent,open=false){
  const d=el("details",undefined,parent);d.open=open;el("summary",title,d);el("pre",JSON.stringify(obj,null,2),d);
}
async function mutate(op,body){current=await api("/sessions/"+current.id+"/"+op,body);}
function reading(g) {
  const mode=input(g,"mode","Gauge reading",{items:["ระบุภายหลัง","Measured","Interval"]});
  const m=input(g,"measured","Independent reading (mm)",{type:"number"});
  const lo=input(g,"low","Lower bound (mm)",{type:"number"});
  const hi=input(g,"high","Upper bound (mm)",{type:"number"});
  function change(){m.parentNode.hidden=mode.value!=="Measured";lo.parentNode.hidden=hi.parentNode.hidden=mode.value!=="Interval";}
  mode.onchange=change;change();
}
function measurement(){return {measured_mm:val("mode")==="Measured"?num("measured"):null,
  interval_mm:val("mode")==="Interval"?[num("low"),num("high")]:null};}
function collect(){
  const c=current.config;
  const g=group("เลือกตัวอย่าง / trial");
  const trials=Array.from({length:c.repetitions},(_,i)=>String(i+1));
  if(method==="V4"){input(g,"side","Arm",{items:[c.side]});input(g,"jig","Jig",{items:Object.keys(c.jigs)});
    input(g,"socket","Socket",{items:["A","B"]});}
  if(method==="V5"){input(g,"socket","Physical socket ID",{items:[...c.sockets,...(c.training_sockets||[])]});input(g,"side","Arm",{items:armNames});}
  if(method==="V1")input(g,"side","Capture arm",{items:armNames});
  if(method==="V3"){
    input(g,"pose","Pose ID");input(g,"direction","Direction");input(g,"planned","Planned gap (mm)",{type:"number"});reading(g);
  }
  if(["V3","V4","V5"].includes(method))input(g,"repeat","Trial",{items:trials});
  const status=el("strong","",work);
  function trial(){
    if(method==="V1")return {side:val("side")};
    if(method==="V2")return {};
    const repeat=num("repeat");
    if(method==="V3")return {repeat,pose:val("pose"),direction:val("direction"),planned_mm:num("planned"),...measurement()};
    const t={repeat,side:val("side"),socket:val("socket")};
    if(method==="V4")t.jig=val("jig");
    else t.role=c.sockets.includes(t.socket)?"validation":"training";
    return t;
  }
  if(["V4","V5"].includes(method)){
    const sockets=method==="V4"?["A","B"]:[...c.sockets,...(c.training_sockets||[])];
    const jigs=method==="V4"?Object.keys(c.jigs):[""];
    outer:for(const jig of jigs)for(const i of trials)for(const socket of sockets)for(const side of method==="V4"?[c.side]:armNames){
      if(!current.samples.some(x=>!x.excluded && x.trial.repeat===Number(i) && x.trial.socket===socket && x.trial.side===side && (x.trial.jig||"")===jig)){
        if(get("jig"))get("jig").value=jig;get("socket").value=socket;get("side").value=side;get("repeat").value=i;break outer;
      }
    }
  }
  if(retryTrial){
    for(const k of ["side","jig","socket","repeat","pose","direction"])if(get(k) && retryTrial[k]!=null)get(k).value=retryTrial[k];
    if(get("planned"))get("planned").value=retryTrial.planned_mm??"";
  }
  function context(){const t=trial();status.textContent=[t.side,t.jig,t.socket && "Socket "+t.socket,t.role,t.pose,t.direction,
    t.repeat && "Trial "+t.repeat+"/"+c.repetitions].filter(Boolean).join(" · ");}
  g.addEventListener("input",context);context();
  async function save(op,body,identity){
    const t=trial(),slot="validation.pending."+current.id;
    const fingerprint=JSON.stringify([op,t,identity]);
    const pending=JSON.parse(localStorage.getItem(slot)||"null");
    const key=pending?.fingerprint===fingerprint ? pending.key : crypto.randomUUID();
    localStorage.setItem(slot,JSON.stringify({key,fingerprint}));
    await mutate(op,{key,trial:t,...body});
    localStorage.removeItem(slot);retryTrial=null;step=3;render();message.textContent="บันทึกตัวอย่างแล้ว";
  }
  if(method!=="V2"){
    el("p","เมื่อจัดท่าเรียบร้อยและหยุดนิ่งอย่างน้อย 1 วินาที ให้คัดลอกข้อมูลจาก receiver",work);
    button(work,"Capture stationary window",()=>save("capture",{},"cached"));
  }
  if(["V1","V2"].includes(method)){
    const source=group("นำเข้า original run");
    button(source,"เลือก saved run",async()=>{
      const ids=await api("/runs");
      if(!ids.length){message.textContent="ไม่มี saved run";return;}
      if(get("rid"))return;
      input(source,"rid","Run",{items:ids});button(source,"Import selected run",()=>save("run",{run_id:val("rid")},val("rid")));
    });
    const file=input(source,"raw","raw.json",{type:"file"});file.accept=".json";
    button(source,"Import JSON",async()=>{
      const f=file.files[0];if(!f || f.size>48*1024**2)throw Error("เลือก raw.json ขนาดไม่เกิน 48 MiB");
      await save("import",{raw:JSON.parse(await f.text())},[f.name,f.size,f.lastModified]);
    });
  }
}
function inspect(){
  const all=current.samples;
  if(!all.length){el("p","ยังไม่มีตัวอย่าง",work);button(work,"เลือกแหล่งข้อมูล",()=>{step=2;render();});return;}
  const sc=el("div",undefined,work,"v-scroll"),tab=el("table",undefined,sc),head=el("tr",undefined,tab);
  for(const name of ["Sample","Side / socket / trial","Packets","Status"])el("th",name,head);
  const chosen=input(work,"sample","Inspect sample",{items:all.map(x=>x.key),value:all.at(-1).key});
  for(const [i,x] of all.entries()){
    const tr=el("tr",undefined,tab);
    button(el("td",undefined,tr),String(i+1),()=>{chosen.value=x.key;show();});
    el("td",[x.trial.side,x.trial.socket,x.trial.repeat,x.run_id].filter(Boolean).join(" · "),tr);
    el("td",Object.entries(x.counts).map(v=>v.join(" ")).join(", "),tr);
    el("td",x.excluded?"Archived · "+x.reason:"Saved",tr);
  }
  const view=el("div",undefined,work);
  function show(){
    view.replaceChildren();const x=all.find(x=>x.key===chosen.value);
    if(x.quality){
      resultTable(view,["Arm","Unique packets","Window (s)","Max gap (s)","Drift (rad)","Speed (rad/s)"],
        Object.entries(x.quality).map(([side,q])=>[side,q.unique_packets,q.duration_s,q.max_gap_s,q.drift_rad,q.speed_rad_s]));
    }
    if(method==="V3" && x.trial.measured_mm==null && x.trial.interval_mm==null)
      el("p","รอค่าระยะจากเกจหรือช่วงการวัดก่อนวิเคราะห์",view);
    details("Trial and latest packet",x,view);
    const g=group("ตรวจตัวอย่าง",view);
    if(method==="V3" && !x.excluded){
      reading(g);button(g,"บันทึกค่าเกจ",async()=>{await mutate("trial",{key:x.key,...measurement()});render();});
    }
    if(!x.excluded)input(g,"reason","เหตุผลที่ archive / retry");
    for(const op of x.excluded?["unexclude"]:["exclude","retry"])button(g,op,async()=>{
      await mutate(op,{key:x.key,reason:val("reason")});retryTrial=op==="retry"?x.trial:null;step=op==="retry"?2:3;render();
    });
  }
  chosen.onchange=show;show();
  const acts=el("div",undefined,work,"v-actions");
  button(acts,"บันทึกตัวอย่างถัดไป",()=>{step=2;render();});
  button(acts,"Analyze",async()=>{await mutate("analysis",{});step=4;render();});
}
function format(value){
  if(Array.isArray(value))return value.map(format).join(" / ");
  if(typeof value==="number")return Number(value.toPrecision(5)).toString();
  return value??"—";
}
function resultTable(parent,headers,rows){
  const sc=el("div",undefined,parent,"v-scroll");
  const table=el("table",undefined,sc);
  const head=el("tr",undefined,el("thead",undefined,table));
  for(const title of headers)el("th",title,head).scope="col";
  const body=el("tbody",undefined,table);
  for(const values of rows){
    const tr=el("tr",undefined,body);
    for(const value of values)el("td",format(value),tr);
  }
}
function measurements(a){
  const name=key=>current.samples.find(x=>x.key===key)?.run_id || key;
  const summary=a.absolute_error;
  if(summary)resultTable(work,["Metric","Mean","RMS","Max"],
    [[a.metric+" ("+a.unit+")",summary.mean,summary.rms,summary.max]]);
  if(a.orientation_deg)resultTable(work,["Metric","Mean","RMS","Max"],
    [["SO(3) (deg)",a.orientation_deg.mean,a.orientation_deg.rms,a.orientation_deg.max]]);
  if(a.run_summary_ms)resultTable(work,["Across runs (ms)","Mean","Sample SD"],
    Object.entries(a.run_summary_ms).map(([k,v])=>[k,v.mean,v.sd]));
  if(a.by_point_mm)resultTable(work,["Socket","Mean (mm)","RMS (mm)","Max (mm)"],
    Object.entries(a.by_point_mm).map(([k,v])=>[k,v.mean,v.rms,v.max]));
  if(a.repeat_dispersion)resultTable(work,["Jig / arm / socket","Captures","Radial RMS (mm)"],
    a.repeat_dispersion.map(x=>[x.group,x.count,x.radial_rms_mm]));
  if(!a.rows.length)return;
  el("h4","Trial results · "+a.row_count+(a.row_count>a.rows.length?" (preview "+a.rows.length+")":""),work);
  const rows=a.rows;
  if(method==="V1")resultTable(work,["Side","Host receipt ns","ΔX / ΔY / ΔZ (mm)","Norm (mm)","SO(3) (deg)"],
    rows.map(r=>[r.side,r.received_monotonic_ns?.toString(),r.xyz_mm,r.error_mm,r.orientation_deg]));
  if(method==="V2"){
    resultTable(work,["Run","τL (ms)","τR (ms)","Δτ (ms)"],rows.map(r=>[name(r.sample),r.left.tau_ms,r.right.tau_ms,r.delta_tau_ms]));
    resultTable(work,["Run / side","Packets","Before (rad)","After (rad)","J1–J6 residual RMS (rad)"],
      rows.flatMap(r=>armNames.map(k=>[name(r.sample)+" / "+k,r[k].common_packets,r[k].before_rad,r[k].after_rad,r[k].joint_rmse_rad])));
  }
  if(method==="V3")resultTable(work,["Pose / direction","Trial","Planned (mm)","FK (mm)","Gauge (mm)","Error (mm)","Planes (deg)"],
    rows.map(r=>[[r.pose,r.direction],r.repeat,r.planned_mm,r.fk_gap_mm,r.interval_mm??r.measured_mm,r.error_interval_mm??r.error_mm,r.parallel_deg]));
  if(method==="V4")resultTable(work,["Jig","Arm","Trial","Distance (mm)","Error (mm)"],
    rows.map(r=>[r.jig,r.side,r.repeat,r.distance_mm,r.error_mm]));
  if(method==="V5")resultTable(work,["Socket","Trial","L − R XYZ (mm)","3D norm (mm)"],
    rows.map(r=>[r.socket,r.repeat,r.xyz_mm,r.error_mm]));
}
function results(){
  const path=base+"/sessions/"+current.id+"/export/";
  const a=current.analysis;
  button(work,"Analyze saved evidence",async()=>{await mutate("analysis",{});render();});
  function download(fmt,index=0){const link=el("a","Download "+fmt.toUpperCase(),actions);
    link.href=path+fmt+"?run_index="+index;link.download=method+"-"+current.id+"."+fmt;return link;}
  const actions=el("div",undefined,work,"v-actions");download("json");
  if(!a)return;
  el("p","Data usable · "+(a.enough_data?"เพียงพอ":"ข้อมูลไม่ครบ / มีข้อจำกัด")+"  |  Acceptance · "+a.acceptance,work);
  const issues=[...a.issues,...(a.criterion_issues||[])];
  if(issues.length){
    const list=el("ul",undefined,work);
    for(const issue of issues)el("li",issue,list);
  }
  if(a.criterion)el("p","Criterion ≤ "+a.criterion.value+" "+a.unit+" · "+a.criterion.source+" · "+a.criterion.uncertainty,work);
  if(a.limitation)el("p",a.limitation,work);
  measurements(a);
  if(a.training?.rows.length || a.training?.issues.length)details("Training points",a.training,work);
  download("csv");
  if(a.row_count){
    const pick=method==="V2"?input(work,"plotrun","Run for plot",{items:a.rows.map(r=>r.sample)}):null;
    const img=el("img",undefined,work);img.alt=method+" validation plot";
    const link=download("png");
    function select(){const url=path+"png?run_index="+(pick?.selectedIndex||0)+"&n="+Date.now();img.src=url;link.href=url;}
    if(pick)pick.onchange=select;select();

  }
}
// Independent dropdown below Legacy utilities; opening the native <details>
 // initializes the panel without relying on the top operator navigation.
panel.addEventListener("toggle",()=>{if(panel.open && !home.childElementCount)homeView();});
