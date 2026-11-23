# Campaign A — Path A — NOM vs LOW: Batch QC

Audit date: 2026-10-19; independent READ-ONLY processing of 20 persisted runs. No robot motion or raw evidence modifications.

**All runs: CLOSED / COMPLETED. MQ quality is separate from successful execution.**

| Condition | Captured | Strict MQ PASS | Strict MQ BLOCKED |
|---|---:|---:|---:|
| NOM | 10 | 8 | 2 |
| LOW | 10 | 0 | 10 |

NOM uses speed 100% / acceleration 100% and 24.528 s planned duration.
LOW uses speed 50% / acceleration 100% and 45.024 s planned duration.
Both conditions have 8 waypoint poses identical to floating-point precision, with multi-axis translation and rotation.
Saved `path_type=LINEAR` is **only the manually selected Recorder metadata** and does not reflect the multi-axis path; describe it as Mixed Translation–Rotation in scientific results.

MQ acceptance thresholds are **not relaxed**: direct Port10000 packets, strictly increasing host monotonic nanosecond timestamp, host wall-clock ms nondecreasing, max per-arm monotonic receive gap <=250 ms, phase coverage >=95%, provenance integrity.
Fix made to MQ-only timestamp logic: equal integer-millisecond host receive stamps are allowed if nanosecond monotonic timestamps and packet sequences increase. This does not alter raw data and preserves the 250ms gap exclusion.

Strict MQ failures do **not** prove motion failure and do not mandate deleting runs. Evidence with gaps remains available, but do not silently pool blocked runs into the primary final statistics.
LOW R01 additionally has phase usable fraction below 95%; examine timing and coverage before deciding on supplementary/exploratory analysis.
No EXP2 synchronization metrics have been derived by the existing web EXP-2 analysis, which instead measures rigid grasp / relative pose (new thesis EXP3).

Each run ID and per-arm packet/gap details are in `summary.csv`. Report outcome is *data qualification only*, not actual robot pose/force ground truth.
