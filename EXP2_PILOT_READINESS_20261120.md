# JAKA Dual Arm — Offline EXP2 / Performance Pilot Readiness
Date: 2026-11-20 (Asia/Bangkok)

## Scope and isolation
- Offline implementation directory: /home/newtonrattapong/JAKA-exp2-offline-review
- Initial Windows Git bundle commit: 0926fedf5e7041bbf281722b9d28bbb376b43a37, based on 9c89f23.
- Live workspace /home/newtonrattapong/jaka_ws (branch feature/coupled-ik-v1, ac514b5) was NOT modified, reloaded, or restarted by this task. The drivers and web backend were already running at audit start; do not confuse their capabilities with the new offline files.
- No real robot motion or additional controller/network probing was performed by the EXP2 implementation.
- No C++ driver, servo motion code, servo_step_num, or 24 ms command interval modifications.

## What has been implemented (opt-in)
1. Separate endpoint evidence stream in raw.json.endpoint_evidence (exp2-host-endpoints/v1). Retains pre-arm/pre-release sample history and records past authoritative terminal until settled or bounded timeout. Legacy actual_packets and EXP1/EXP3 metrics preserved.
2. Opt-in checkbox in existing Experiment Recorder HMI; Request Model carries endpoint_evidence. Default OFF. FINALIZING state blocks new arm and protects prior run.
3. Automatic background finalization after settle or timeout with no browser polling necessary. Disk write failure can be retried; process crash before persistence still loses in-memory evidence.
4. Host-side per-arm start and settle brackets, sign of R-L, overlap/insufficient resolution, stationary noise and receive gap diagnostics; export per-sample CSV, PNG overview, four-panel start/stop zoom.
5. Per-run driver status aggregate snapshots from existing get_execution_status: dispatch lateness/jitter, missed cycles, ServoJ call durations/overrun, first/last host dispatch and return times, command period. No per-tick trace, no controller acquisition clock.
6. Opt-in browser performance collector (rAF frame time, long tasks, overlapping requests) and standalone Linux /proc collector (CPU/RAM/process I/O), both offline-tested.
7. Batch table creator for run-id directories and optional per-run plots; user-defined --resolution-ms threshold (do not guess numerical acceptance criterion).

## Commands to analyze existing saved runs OFFLINE
Run in the offline worktree. Avoid reusing filenames as the tools intentionally will not overwrite existing outputs.

- PYTHONPATH=. python3 -m tools.exp2_host_endpoints RUN/raw.json --samples-csv samples.csv --plot-png overview.png
- PYTHONPATH=. python3 -m tools.exp2_event_zoom RUN/raw.json start_stop_zoom.png
- PYTHONPATH=. python3 -m tools.exp2_batch_report RUN_ROOT summary.csv --resolution-ms T --plots-dir plot_directory
- python3 tools/exp2_dispatch_summary.py RUN/raw.json
- python3 tools/exp2_host_performance.py host.jsonl --samples 60 --interval-s 1
- Load tools/exp2_browser_performance.js as an opt-in DevTools Snippet, then export JSON (clock bridge + event metadata) and CSV. Compare measurement OFF and ON to detect instrumentation overhead.

For old runs without recorded pre-start and post-stop evidence, the STRICT new method may return INCOMPLETE even when an earlier, weaker method could infer a start bracket. Do not fabricate stop time.

## Synthetic validation, NOT robot measurements
- Right arm motion begins 30 ms after left; stops nominally equal; irregular sample frequencies.
- Computed R-L start interval [25,40] ms -> Right later.
- Computed R-L settle interval [-5,10] ms -> ordering not resolved (zero included).
- PNG preview: /home/newtonrattapong/Downloads/JAKA_EXP2_SYNTHETIC_start_stop_zoom_20261120.png

## October Original Data diagnostic baseline (20 runs)
Read-only reanalysis from Original joint/FK files with host monotonic receive timestamps:
- 12/20 runs include at least one gap >250 ms.
- 84 gap intervals (>250 ms).
- 358 samples with relative translation error >5 mm, of which only 10 lie within a gap or 100 ms of its end.
- 30 samples >10 mm, only 3 near such gaps.
- An error peak at NOM R09 is 13.421 mm with no long receive gap, so feedback receive gaps are NOT a complete explanation.
- This is association, not causal evidence; timestamps are host-derived and error from encoder + FK.
Files: /home/newtonrattapong/Downloads/JAKA_20261120_original_baseline_notes.md
       /home/newtonrattapong/Downloads/JAKA_20261120_receive_gaps_vs_EXP3_peaks_original.csv
       /home/newtonrattapong/Downloads/JAKA_EXP2_20run_start_stop_legacy_20261120.csv

## Pilot approval gate — STOP BEFORE ROBOT ACCESS
Current offline code is NOT deployed into the running system. Nothing about physical safety or real-robot validity is proven by tests.
Before proceeding:
1. Operator confirms cell access, E-stop availability, fixture, payload, locked grasp and collision clearances. Check physical state; no assumption from status-only process audits.
2. Agree on when to stop/restart the live backend so new opt-in code can be integrated carefully from the isolated branch. Do not force checkout or delete dirty production worktree. Audit dependency versions, source, test results, and runtime feedback contract before deployment.
3. Explicitly authorize read-only static feedback phase (no movement). Capture at least one second baseline BOTH arms; measure arrival intervals, noise, time bridge, and evaluate suitable pre-registered threshold before moving.
4. Explicit second authorization before ONE single low-risk motion pilot using approved path and existing speed/acceleration profile, **unchanged 24 ms ServoJ period**. Do not silently reduce speed or modify path from validated trajectory.
5. Verify that each arm has qualified START/STOP, pre-baseline and tail, no erroneous clock resets, and that status becomes CLOSED; inspect real samples and uncertainty brackets.
6. Only after successful pilot and operator approval plan NOM and LOW 10 repeats each, with pre-defined data quality and resolution acceptance thresholds and stop criteria. Preserve all original raw data.

## Honest measurement limitations
- Start/stop are host receive-clock threshold crossing time BRACKETS, not hardware controller synchronization or absolute physical movement time.
- No verified timestamp from controller sampling instant in current Port10000 packet schema. Local packet_sequence_index is receiver-owned.
- Driver aggregate dispatch diagnostics can indicate timing trouble within a run but CANNOT prove exact time alignment of an individual servo deadline miss to a specific error peak without per-tick timestamps.
- Browser frame timing may reveal UI stalls but cannot prove a robot stall.
- EXP1 / EXP3 results use encoder + kinematic model, not external metrology.
- Persist after tail is asynchronous; abrupt process death may lose in-memory evidence.

## Additional regression note
Additional broad Phase 5 source tests have two known stale UI string/placement expectations that also fail against unchanged HEAD HTML, independent of these EXP2 changes. They must not be counted as new regressions and should be addressed separately, without changing real motion controls casually.
