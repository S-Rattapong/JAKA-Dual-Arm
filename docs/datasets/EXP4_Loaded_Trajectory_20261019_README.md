# JAKA A12 Dual Arm — EXP4 Loaded Cooperative Trajectory (2026-10-19)

## What was recorded
- **10 named experimental repetitions** of a *single executed out-and-back trajectory*: left → right → left, using a hook-type gripper without an open/close actuator.
- There are **11 distinct recorded Run IDs**, because the display label `EXP4_LOADED_R07` was used twice. Both original recordings are retained. The experimenter must choose which R07 corresponds to the intended set of 10 before final statistical analysis.
- **All 11 recordings are `CLOSED` / `COMPLETED`**, with duration 63.072 seconds and a common plan fingerprint. The planned trajectory contains 2,629 samples; the configured path contains 19 waypoints (W0–W18).
- Each Run ID is a directory under `dual_arm_app/experiment_runs/` containing the original `manifest.json`, `raw.json`, `aligned.json`, `exp1_analysis.json`, `exp2_analysis.json`, `observed.csv` and `planned.csv`.

## Experiment outcome vs data quality

Execution completion **is not a guarantee of error-free timing or successful physical pick/place**. All recordings have moments with >250 ms between host-side receive timestamps. Raw Port10000 streams contain roughly 680–696 per-side packets over the ~63-second trajectory, so offline graphing and coverage-controlled comparisons are possible, but unavailable intervals and timing uncertainty must be displayed.

Existing `exp2_analysis.json` is the software's *rigid-grasp / relative-pose* analyzer (the thesis's EXP3), not EXP4 repeatability. It is often `PARTIAL`, including cases where phase matching is ambiguous. Do not interpret this as raw samples being erased. Recompute EXP4 trajectories from `raw.json.actual_packets` and the same URDF FK/model configuration, checking time alignment and cross-arm coverage.

The estimated object center from encoder FK/locked grasp is **model-based**, not optical measurement of the workpiece. No physical object position ground truth or external sensor measurement is present.

## Files to use

```text
dual_arm_app/experiment_runs/exp-.../   11 EXP4 directories, each has 7 original files
docs/mq_reports/20261019_exp4_loaded_batch/
  summary.csv                 11-run diagnostic summary
  summary.md                  Data quality and limitations
  dataset_index.json          Per-run metadata, including 77 original-file SHA256
  checksums.sha256            SHA256 for all 77 original EXP4 files
tools/verify_exp4.py          Offline integrity verifier on Ubuntu or Windows
```

**Do not delete one of the two R07 runs** until the final dataset selection decision has been documented. Keep both in the archive regardless of the final choice.

## Get this dataset on Windows

This EXP4 dataset has been **added to the same branch that already contains the 20 EXP1–EXP3 records**:

`data/campaign-a-path-a-20261019`

If you already cloned this data branch on Windows, open a PowerShell terminal inside its repository and run:

```powershell
git status
git pull --ff-only origin data/campaign-a-path-a-20261019
python tools/verify_exp4.py
python tools/verify_campaign_a.py
```

**If there are local modifications, commit or stash them before pulling.** If creating a new clone instead:

```powershell
git clone --single-branch --branch data/campaign-a-path-a-20261019 https://github.com/S-Rattapong/JAKA-Dual-Arm.git
cd JAKA-Dual-Arm
python tools/verify_exp4.py
```

A valid offline integrity check prints `PASS — 11 runs / 77 original evidence files intact`.

## Offline analysis strategy

1. Confirm intended R07 among the two distinct runs.
2. Split each single Execute into outward and return legs using the saved plan waypoint/timing boundaries. The first and second legs belong to **the same recorded run**.
3. Plot Left/Right TCP and estimated object-center 3D/2D paths reconstructed from Port10000 actual joint encoders via URDF FK, and the planned trajectory as reference.
4. Report nominal-versus-actual tracking and repeatability across the 10 selected runs, with interpolation coverage, host timing jitter and dropout effects separated from physical motion accuracy.
5. Do not silently fill long data gaps or loosen MQ acceptance criteria just to obtain a PASS.

No robot motion was commanded to create this GitHub data archive. The original Ubuntu experiment folders were not altered.
