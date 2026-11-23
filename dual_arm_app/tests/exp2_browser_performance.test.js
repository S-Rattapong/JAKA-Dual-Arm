// node --test dual_arm_app/tests/exp2_browser_performance.test.js
const {test} = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

test("opt-in observation across frames and overlapping fetches, then disconnect", () => {
  let tick;
  let disconnected = 0;
  let canceled = 0;
  const observers = {};
  class Observer {
    static supportedEntryTypes = ["resource", "longtask"];
    constructor(callback) { this.callback = callback; }
    observe({type}) { observers[type] = this; }
    takeRecords() { return []; }
    disconnect() { disconnected++; }
  }
  const env = {
    window: {}, performance: {now: () => 10, timeOrigin: 1700000000000},
    location: {href: "https://offline.invalid"}, PerformanceObserver: Observer, URL,
    requestAnimationFrame(cb) { tick = cb; return 7; },
    cancelAnimationFrame(id) { assert.equal(id, 7); canceled++; },
  };
  const script = fs.readFileSync(path.join(__dirname, "../../tools/exp2_browser_performance.js"), "utf8");
  vm.runInNewContext(script, env);
  vm.runInNewContext(script, env);
  tick(10);
  tick(64);
  observers.resource.callback({getEntries: () => [11,14,40].map(startTime => ({
    entryType: "resource", initiatorType: "fetch", name: "/api/status?private=abc",
    startTime, duration: 12,
  }))});
  observers.longtask.callback({getEntries: () => [{entryType: "longtask", startTime: 11, duration: 51}]});
  const report = env.window.exp2Performance.stop();
  assert.equal(report.events[0].duration_ms, 54);
  assert.equal(report.events.length, 5);
  assert.equal(report.events[1].path, "/api/status");
  assert.equal(report.overlappingRequests, 1);
  assert.equal(report.omitted, 0);
  assert.deepEqual(report, env.window.exp2Performance.stop());
  assert.equal(disconnected, 2);
  assert.equal(canceled, 1);
});
