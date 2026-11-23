// Run once as a DevTools snippet in the HMI tab. No network requests or patches.
(() => {
  if (window.exp2Performance) return;
  const events = [];
  const observers = [];
  let omitted = 0;
  let previous;
  let frame;
  let active = true;
  let final;
  const bridge = () => ({before_ms: performance.now(), wall_ms: Date.now(), after_ms: performance.now()});
  const initial = bridge();
  const record = (type, start_ms, duration_ms, path = "") => {
    if (events.length < 20000) events.push({type, start_ms, duration_ms, path});
    else omitted++;
  };
  const tick = now => {
    if (previous !== undefined) record("raf", previous, now - previous);
    previous = now;
    frame = requestAnimationFrame(tick);
  };
  frame = requestAnimationFrame(tick);
  const supported = globalThis.PerformanceObserver?.supportedEntryTypes || [];
  function collect(entries) {
    for (const e of entries) {
      if (e.entryType === "longtask") record("longtask", e.startTime, e.duration);
      else if (["fetch", "xmlhttprequest"].includes(e.initiatorType))
        record("api", e.startTime, e.duration, new URL(e.name, location.href).pathname);
    }
  }
  for (const type of ["resource", "longtask"].filter(t => supported.includes(t))) {
    const observer = new PerformanceObserver(list => collect(list.getEntries()));
    observer.observe({type});
    observers.push(observer);
  }
  window.exp2Performance = {stop() {
    if (active) {
      active = false;
      final = bridge();
      cancelAnimationFrame(frame);
      for (const o of observers) { collect(o.takeRecords()); o.disconnect(); }
    }
    const api = events.filter(e => e.type === "api").sort((a,b) => a.start_ms-b.start_ms);
    let latestEnd = -Infinity;
    let overlappingRequests = 0;
    for (const e of api) {
      if (e.start_ms < latestEnd) overlappingRequests++;
      latestEnd = Math.max(latestEnd, e.start_ms + e.duration_ms);
    }
    return {schema_version: "exp2-browser-performance/v1", origin_wall_ms: performance.timeOrigin,
      initial, final, supported, omitted, overlappingRequests, events};
  }, download(format = "json") {
    const data = this.stop();
    const quote = s => '"' + String(s).replaceAll('"', '""') + '"';
    const content = format === "csv" ? ["type,start_ms,duration_ms,path",
      ...events.map(e => [e.type,e.start_ms,e.duration_ms,e.path].map(quote).join(","))].join("\n")
      : JSON.stringify(data);
    const link = document.createElement("a");
    link.href = URL.createObjectURL(new Blob([content]));
    link.download = "exp2-browser." + format;
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }};
})();
