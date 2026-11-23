"""Offline browser contract for supervised Phase5 Execute loops (no ROS/motion)."""
import unittest

from dual_arm_app.tests.test_phase5_execution_preview_authority import (
    run_phase5_browser_harness,
)


class Phase5LoopBrowserOfflineTests(unittest.TestCase):
    def test_loop_mode_requires_explicit_click_and_server_signed_lease(self):
        result = run_phase5_browser_harness(r"""
const elements = new Map();
function ui(id) {
  if (!elements.has(id)) elements.set(id, {
    id, textContent: "", disabled: false, hidden: false, checked: false,
    value: id === "digitalTwinPhase5LoopMode" ? "once"
      : id === "digitalTwinPhase5LoopCount" ? "2" : "",
    dataset: {}, listeners: {},
    addEventListener(type, callback) {this.listeners[type] = callback},
    closest() { return {dataset: {}}; },
  });
  return elements.get(id);
}
const storage = new Map();
const sessionStorage = {
  getItem(key){return storage.has(key)?storage.get(key):null},
  setItem(key,value){storage.set(key,String(value))},
  removeItem(key){storage.delete(key)},
};
globalThis.window = globalThis;
globalThis.sessionStorage = sessionStorage;
globalThis.document = {
  readyState:"complete",
  visibilityState:"visible",
  getElementById: id => ui(id),
  addEventListener(){},
};
globalThis.addEventListener = ()=>{};
const intervals=[];
globalThis.setInterval=(fn,ms)=>{intervals.push({fn,ms});return intervals.length;};
let confirms=0;
globalThis.confirm=()=>{confirms++;return true;};
let loaded=null;
globalThis.dualArmDigitalTwin={
  loadPlannedTrajectory(trajectory,source){
    loaded={source,durationS:trajectory.points.at(-1).time_from_start_s,
      pointCount:trajectory.points.length};
    return this.getTrajectoryPreviewState();
  },
  getTrajectoryPreviewState(){return loaded && {...loaded,status:"READY"}},
  getPlannedPreviewState(){return loaded&&{visible:true,source:loaded.source,loadState:{status:"READY"}}},
  clearPlannedTrajectory(){loaded=null},
  stopPlannedTrajectory(){},
  setTrajectoryPlaybackRate(){},
  playPlannedTrajectory(){},
};
const artifact={
  available:true, artifact_fingerprint:"closed-loop-artifact",
  generation:1,plan_fingerprint:"closed-plan",
};
const status={
  ok:true,artifact,phase4_gate:{execution_ready:true,status:"READY"},
  transport:{robot_connection:{both_ready:true}},
  safe_state:{ok:true},start_match:{match:true},
  operator_authority:{pending:false},
  blocking_reasons:[],ready_to_prepare:true,stop_generation:5,
  execution:{state:"IDLE"},driver_feedback:{combined_state:"IDLE",drivers:{}},
};
const artifactPayload={
  ok:true,available:true,status:"FROZEN",generation:1,
  artifact:{
    artifact_fingerprint:"closed-loop-artifact",
    trajectory:{name:"closed",duration_s:2,common_timestamps_s:[0,1,2],
      left:{positions_rad:[[0,0,0,0,0,0],[.1,0,0,0,0,0],[0,0,0,0,0,0]]},
      right:{positions_rad:[[0,0,0,0,0,0],[0,.1,0,0,0,0],[0,0,0,0,0,0]]}},
  },
};
let loopState={
  id:null, state:"IDLE",mode:"once",count:1,completed_cycles:0,
  current_cycle:0,
};
const closure={
  eligible:true,
  reason:"JOINT_CLOSED_START_END",
  max_start_end_joint_delta_rad:0,
  artifact_fingerprint:artifact.artifact_fingerprint,artifact_generation:1,
};
const requests=[];
globalThis.fetch=async(url,options={})=>{
  const method=options.method||"GET";
  requests.push({url,method,body:options.body||null});
  let payload;
  if(url.endsWith("/phase5/execution-artifact"))payload=artifactPayload;
  else if(url.endsWith("/phase5/execution"))payload=status;
  else if(url.endsWith("/loop/status"))payload={loop:loopState,repeatability:closure};
  else if(url.endsWith("/loop/start")){
    const body=JSON.parse(options.body);
    loopState={id:"test-loop-id",state:"RUNNING",mode:body.mode,
      count:body.count,completed_cycles:0,current_cycle:1};
    payload={ok:true,accepted:true,id:"test-loop-id",lease_token:"server-token"};
  }else if(url.endsWith("/loop/heartbeat"))payload={ok:true,active:true,completed_cycles:0};
  else if(url.endsWith("/loop/cancel")){
    loopState.cancel_after_current_requested=true;
    payload={ok:true,cancel_after_current_requested:true};
  } else throw Error("Unexpected network request: "+url+" "+method);
  return {ok:true,status:200,json:async()=>payload};
};
await import("./phase5.mjs");
for(let i=0;i<35 && !globalThis.dualArmPhase5Execution.previewReady();i++){
  await new Promise(resolve=>setImmediate(resolve));
}
if(!globalThis.dualArmPhase5Execution.previewReady())throw Error("Expected frozen preview");
for(let i=0;i<25 && !requests.some(x=>x.url.endsWith("/loop/status"));i++){
  await new Promise(resolve=>setImmediate(resolve));
}
const noMotionBeforeClick=requests.every(x=>x.method==="GET");
const onceMode=ui("digitalTwinPhase5LoopMode").value;
// Simulate browser form restoration: the select value changes without any event.
ui("digitalTwinPhase5LoopMode").value="count";
await globalThis.dualArmPhase5Execution.refreshStatus();
const autoRestoredCountFieldVisible=ui("digitalTwinPhase5LoopCountGroup").hidden===false;
ui("digitalTwinPhase5LoopMode").value="once";
await globalThis.dualArmPhase5Execution.refreshStatus();
ui("digitalTwinPhase5LoopMode").value="count";
ui("digitalTwinPhase5LoopMode").listeners.change();
ui("digitalTwinPhase5LoopCount").value="3";
for(let i=0;i<50 && (
  !ui("digitalTwinPhase5LoopEligibility").textContent.includes("JOINT-CLOSED")
  || ui("digitalTwinPhase5Prepare").disabled
);i++){
  await new Promise(resolve=>setImmediate(resolve));
}
const countGroupVisible=ui("digitalTwinPhase5LoopCountGroup").hidden===false;
const countActionLabel=ui("digitalTwinPhase5Prepare").textContent;
const enabled=ui("digitalTwinPhase5Prepare").disabled===false;
if(!enabled)throw Error("Closed fixed loop should be enabled in mock");
await ui("digitalTwinPhase5Prepare").listeners.click();
const startRequests=requests.filter(x=>x.url.endsWith("/loop/start")&&x.method==="POST");
const regularExecuteRequests=requests.filter(x=>x.url.endsWith("/phase5/execute")&&x.method==="POST");
const storedLease=JSON.parse(sessionStorage.getItem("jaka-phase5-supervised-loop-v1")||"null");
await globalThis.dualArmPhase5Execution.refreshStatus();
for(let i=0;i<35 && !requests.some(x=>x.url.endsWith("/loop/heartbeat"));i++){
  await new Promise(resolve=>setImmediate(resolve));
}
const heartbeatRequests=requests.filter(x=>x.url.endsWith("/loop/heartbeat")&&x.method==="POST");
const cancelEnabled=ui("digitalTwinPhase5LoopCancel").disabled===false;
await ui("digitalTwinPhase5LoopCancel").listeners.click();
const cancelRequests=requests.filter(x=>x.url.endsWith("/loop/cancel")&&x.method==="POST");
ui("digitalTwinPhase5LoopMode").value="forever";
ui("digitalTwinPhase5LoopMode").listeners.change();
const foreverCountHidden=ui("digitalTwinPhase5LoopCountGroup").hidden===true;
console.log(JSON.stringify({
 noMotionBeforeClick,onceMode,autoRestoredCountFieldVisible,countGroupVisible,enabled,
 startRequests,regularExecuteRequests,storedLease,
 heartbeatRequests,cancelRequests,confirms,cancelEnabled,
 foreverCountHidden,countActionLabel,intervalCount:intervals.length,
 errorText:ui("digitalTwinPhase5Error").textContent,
 modeNow:ui("digitalTwinPhase5LoopMode").value,
 previewReady:globalThis.dualArmPhase5Execution.previewReady(),
 listenerSource:ui("digitalTwinPhase5Prepare").listeners.click.toString().slice(0,360),
 operatorAction:ui("digitalTwinPhase5OperatorAction").textContent,
 loopStatusDisplay:ui("digitalTwinPhase5LoopEligibility").textContent
}));
""")
        self.assertTrue(result["noMotionBeforeClick"])
        self.assertEqual(result["onceMode"], "once")
        self.assertTrue(result["autoRestoredCountFieldVisible"])
        self.assertTrue(result["countGroupVisible"])
        self.assertIn("3 cycles", result["countActionLabel"])
        self.assertTrue(result["enabled"])
        self.assertEqual(len(result["startRequests"]), 1, result)
        self.assertIn('"count":3', result["startRequests"][0]["body"])
        self.assertIn('"operator_confirmed":true', result["startRequests"][0]["body"])
        self.assertEqual(result["regularExecuteRequests"], [])
        self.assertEqual(result["storedLease"]["id"], "test-loop-id")
        self.assertEqual(result["storedLease"]["lease_token"], "server-token")
        self.assertTrue(result["heartbeatRequests"])
        self.assertEqual(len(result["cancelRequests"]), 1)
        self.assertEqual(result["confirms"], 1)
        self.assertTrue(result["cancelEnabled"])
        self.assertTrue(result["foreverCountHidden"])


if __name__ == "__main__":
    unittest.main()
