# EXP-2/EXP-3 measurement diagnostics (23 Nov 2026)

Offline, read-only scripts used to diagnose the Port10000 receive gaps and the
EXP-3 relative-pose spikes. None of them write into `experiment_runs/`, start ROS,
or contact a controller. Paths assume this workstation (`/home/newtonrattapong/jaka_ws`).
Run with `PYTHONDONTWRITEBYTECODE=1 nice -n 19 python3 -B <script> ...`.

| Script | Purpose |
|---|---|
| `repro/fake_controller.py`, `repro/harness.py`, `repro/analyze.py` | Offline reproduction: fake 10 Hz Port10000 pusher on 127.0.0.1 + production receiver; modes `baseline`, `list_runs`, `list_runs_loop`, `manifest_only`. `harness.py WS RUNS OUT MODE` then `analyze.py MODE` in the same directory. |
| `armexec_all.py RUNS_DIR` | ARM→execute delay vs receive gaps for every saved run. |
| `check_run.py "<label>"` | Quality summary of one run (gaps, bursts, observations, EXP1/EXP2 coverage). |
| `exp3_detail.py RUN_DIR` | EXP-3 host vs phase summary, peaks vs waypoints/object speed, resampled series (JSON). |
| `spike_drill.py RUN_DIR T0 T1` | Raw packets around a time window. |
| `timestamp_check.py RUN_DIR` | Host receive time vs controller-inferred sample time per packet. |
| `lag_series.py RUN_DIR` | Per-packet tracking lag (controller joint_position vs actual) and outliers. |
| `fk_vs_cart.py`, `fk_cmd_vs_cart.py` | FK step length vs controller Cartesian step (showed `actual_position` is command-derived). |
| `exp3_retime2.py RUN_DIR` | Sensitivity: production `_analyze_host_time` on re-timed / outlier-excluded copies. |

The production implementation of the controller-inferred comparison is
`dual_arm_app/backend/experimental_controller_timebase.py`
(GET `/api/experiments/{run_id}/exp3-timebase`, HMI section "EXP-3 Time Base").
