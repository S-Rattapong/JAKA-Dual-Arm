// Run the real panel script with the real HMI markup. No HMI startup scripts
// execute and the sole transport is an in-memory validation fixture.
const {test}=require("node:test");
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const {JSDOM}=require("jsdom");
const web=path.join(__dirname,"../web");
const html=fs.readFileSync(path.join(web,"index.html"),"utf8");
const script=fs.readFileSync(path.join(web,"validation.js"),"utf8");
const tick=()=>new Promise(resolve=>setTimeout(resolve,5));

test("navigate, configure, capture once, inspect, analyze and resume",async()=>{
  const dom=new JSDOM(html,{url:"https://offline.invalid/",runScripts:"outside-only"});
  const w=dom.window,doc=w.document,paths=[];
  let saved;
  Object.defineProperty(w,"crypto",{value:{randomUUID:()=>"unique-key"}});
  w.fetch=async(url,init)=>{
    paths.push(url);
    assert.ok(url.startsWith("/api/validation/"));
    let result;
    if(url.endsWith("/catalog"))result={model_revision:"fixture"};
    else if(url.endsWith("/overview"))result={V1:{state:"NO_SESSION"},V4:{state:"CAPTURED_NOT_ANALYZED",active_samples:2}};
    else if(url.endsWith("/sessions")){
      const b=JSON.parse(init.body);saved={id:"offline",method:b.method,config:b.config,samples:[]};result=saved;
    }else if(url.endsWith("/capture")){
      const b=JSON.parse(init.body);
      saved.samples.push({key:b.key,trial:b.trial,counts:{left:12},excluded:false});result=saved;
    }else if(url.endsWith("/analysis")){
      saved.analysis={enough_data:false,acceptance:"CANNOT_EVALUATE",issues:["Incomplete A/B"],row_count:0,rows:[]};result=saved;
    }else if(url.endsWith("/offline"))result=saved;
    else throw Error(url);
    return {ok:true,json:async()=>JSON.parse(JSON.stringify(result))};
  };
  w.eval(script);
  const panel=doc.getElementById("validationPanel");
  assert.equal(panel.parentElement,doc.body);
  assert.ok(panel.previousElementSibling.classList.contains("hmi-utility-bay"));
  assert.ok(!panel.previousElementSibling.classList.contains("hmi-validation-bay"));
  assert.equal(doc.querySelectorAll("#validationPanel").length,1);
  assert.equal(doc.querySelectorAll('#digitalTwinOperatorPane #validationPanel').length,0);
  assert.equal(doc.querySelector('a[href="#validationPanel"]'),null);
  panel.open=true;panel.dispatchEvent(new w.Event("toggle"));await tick();
  assert.equal(doc.querySelectorAll(".v-card").length,5);
  assert.ok(paths.every(x=>x.endsWith("/overview")));
  assert.match(panel.textContent,/CAPTURED_NOT_ANALYZED/);
  const btn=text=>[...panel.querySelectorAll("button")].find(e=>e.textContent.includes(text));
  async function click(text){assert.ok(btn(text),text);btn(text).click();await tick();}
  await click("V4 ·");
  doc.getElementById("v-jig1").value="J1";
  await click("บันทึก configuration");
  assert.match(panel.textContent,/left · J1 · Socket A · Trial 1\/5/);
  const capture=btn("Capture stationary window");capture.click();capture.click();await tick();
  assert.equal(paths.filter(x=>x.endsWith("/capture")).length,1);
  assert.match(panel.textContent,/Saved/);
  await click("Analyze");
  if(!saved.analysis)await click("Analyze saved evidence");
  assert.match(panel.textContent,/CANNOT_EVALUATE/);
  await click("‹ Validation");await click("เปิดงานล่าสุด");
  assert.match(panel.textContent,/CANNOT_EVALUATE/);
  dom.window.close();
});

test("method-specific forms share the existing viewport and show API availability",async()=>{
  const dom=new JSDOM(html,{url:"https://offline.invalid/",runScripts:"outside-only"});
  const w=dom.window,d=w.document;
  w.fetch=async url=>{
    assert.ok(url==="/api/validation/catalog" || url==="/api/validation/overview");
    if(url.endsWith("/overview"))return {ok:true,json:async()=>({})};
    return {ok:true,json:async()=>({model_revision:"fixture revision",source:{method:"fixture"}})};
  };
  w.eval(script);
  const dropdown=d.getElementById("validationPanel");
  assert.equal(dropdown.previousElementSibling.getAttribute("aria-label"),"Utility Bay");
  dropdown.open=true;dropdown.dispatchEvent(new w.Event("toggle"));await tick();
  const panel=d.querySelector("#validationPanel");
  const click=async name=>{
    const b=[...panel.querySelectorAll("button")].find(b=>b.textContent.includes(name));
    assert.ok(b,name);b.click();await tick();
  };
  for(const [method,ids] of Object.entries({V1:["leftfield","rightverified"],V2:["tau_min_ms","gap_ms"],
    V3:["leftnormal","parallel_deg","uncertainty_mm"],V5:["lefttoolframe","sockets","training"]})){
    await click(method+" ·");for(const id of ids)assert.ok(d.getElementById("v-"+id),id);
    if(method==="V1" || method==="V5")assert.match(panel.textContent,/fixture revision/);
    await click("‹ Validation");
  }
  w.fetch=async()=>({ok:false,status:404,json:async()=>({detail:"Not found"})});
  await click("V1 ·");
  assert.match(d.querySelector("#validationMessage").textContent,/API ยังไม่พร้อม/);
  assert.equal(d.querySelectorAll("#digitalTwinViewer").length,1);
  dom.window.close();
});
