# JAKA Dual Arm — Campaign A / Path A (2026-10-19)

**Dataset status:** Collected and archived. Not every run passes the project's strict Measurement Qualification screen. Do not report the entire campaign as MQ-PASS.

## Acquisition
- Two JAKA A12 arms; cooperative planned dual-arm trajectories; **20 captured runs** in 2 conditions: NOM 10 and LOW 10.
- NOM: planning speed scale **100%**, acceleration scale **100%**, 24.528 s trajectory.
- LOW: planning speed scale **50%**, acceleration scale **100%**, 45.024 s trajectory.
- Path: same **8 waypoint positions and orientations** to floating-point tolerance across the two conditions. This is a *mixed translation–rotation multi-waypoint path* (not a pure straight line), even though operator metadata says `path_type=LINEAR`.
- Recorded `fixture_condition=NONE`. This dataset is for EXP1 (tracking), EXP2 (dual-arm synchronization), EXP3 (relative-pose preservation), **NOT EXP4 loaded manipulation**.
- Code snapshot at collection: `ac514b53a2619193aa4b87a4dc62de361b659f47` (base of this data branch).
- Scope: Experiment Recorder actual packet source JAKA Port10000; sample `received_at_ms` is host receive, not robot controller sample acquisition time.

## Dataset structure
```text
dual_arm_app/experiment_runs/exp-*/  [20 distinct directories]
  manifest.json        Run identity, path and config
  raw.json             Authoritative original evidence; DO NOT EDIT
  aligned.json         Derived alignment
  exp1_analysis.json   Existing joint tracking analysis
  exp2_analysis.json   Existing named 'EXP-2' rigid-grasp / relative-pose analysis
  planned.csv          Trajectory samples
  observed.csv         Observations
docs/mq_reports/20261019_campaign_A_batch/
  summary.csv          20-run screening, counters and gap stats
  summary.md           Dataset MQ limitations
  dataset_index.json   Run ID index + per-file SHA-256
  checksums.sha256     All 140 evidence-file hashes
  by_run/*.json        Individual MQ screening results
real_robot_scripts/mq_qualification.py  Offline/live MQ CLI, read-only
dual_arm_app/tests/test_mq_qualification.py  Synthetic fail-closed tests
```

## Quality screening
- **NOM: 8/10 strict MQ PASS, 2/10 BLOCKED** due to receive intervals over 250 ms.
- **LOW: 0/10 strict MQ PASS, 10/10 BLOCKED** due to receive intervals over 250 ms; LOW R01 also has phase-matched coverage only ~93.53%, below the 95% reference.
- **All 20** saved states were CLOSED/COMPLETED with raw, manifest and analyses present. *Execution completion does not mean evidence quality PASS.*
- MQ CLI timestamp refinement allows duplicate millisecond host wall-clock timestamps when nanosecond monotonic receive timestamps and packet sequence increase. It does **not** relax the 250ms bound or modify raw data.
- Previously-derived EXP2 relative-pose analysis often says PARTIAL: only valid segments are usable. The data contain host-side temporal jitter, no controller-side synchronized hardware clock, and no external pose metrology.
- Screening failures should remain documented. Investigate sensitivity of engineering conclusions to poor coverage. No post-hoc threshold change to force PASS.

## Opening on Windows

If you have **already cloned** the JAKA-Dual-Arm repository:
```powershell
cd C:/path/to/JAKA-Dual-Arm
git fetch origin
git switch -c data/campaign-a-path-a-20261019 --track origin/data/campaign-a-path-a-20261019
```
If your branch is already created locally, use `git switch data/campaign-a-path-a-20261019`.
**Commit or stash any changes on Windows first before switching** to avoid checkout conflicts.

For a **new clone**:
```powershell
git clone --single-branch --branch data/campaign-a-path-a-20261019 https://github.com/S-Rattapong/JAKA-Dual-Arm.git
cd JAKA-Dual-Arm
```

Verify all 140 evidence files against stored hashes:
```powershell
python tools/verify_campaign_a.py
```
Then inspect `docs/mq_reports/20261019_campaign_A_batch/summary.csv` and the 20 run folders.

## Thesis experiment mapping

- Thesis **EXP1 Trajectory Tracking**: `exp1_analysis.json` and `raw.json`; assess error definition/coverage.
- Thesis **EXP2 Dual-Arm Synchronization**: derive from per-driver execution feedback and recorded packet timing/phase; NOT calculated by the existing UI panel called EXP-2. Report host-clock limitations and timing jitter.
- Thesis **EXP3 Relative Pose**: `exp2_analysis.json` is actually rigid-grasp/relative pose in the older software nomenclature. Use phase-synchronized usable subset with explicit coverage and FK model uncertainty.
- Thesis **EXP4** should be recorded as a **separate campaign with loaded-workpiece protocol**.

## Safety / provenance
No robot motion is commanded by this data branch, checksum verifier or MQ CLI. The original experiment run directories are left in their existing Ubuntu working tree without modification. This export was produced using a separate Git Worktree specifically to avoid staging unrelated older tracked-file deletions or modifying live robot code. The raw evidence is immutable; avoid editing raw or silently excluding failed runs.
