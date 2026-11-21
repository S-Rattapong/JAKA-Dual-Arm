# JAKA Dual-Arm — Experimental Evidence Archive (20 November 2026)

Source: /home/newtonrattapong/jaka_ws, feature/coupled-ik-v1 @ ac514b53a2619193aa4b87a4dc62de361b659f47 when archived.

This branch preserves experimental evidence independently from unfinished code modifications in the primary working directory.
No robot commands, service restart, code build, or checkout was required.

## Collected experiment runs

- exp-20261120T031059766Z-1a3f3576 — EXR Test — CLOSED
- exp-20261120T035609720Z-ecc9a158 — exp2test — CLOSED
- exp-20261120T094736214Z-a30c0de2 — EXP1_New_Run01 — DISARMED before execution, kept for audit
- exp-20261120T095107453Z-11f62ceb — EXP1_New_Run01 — COMPLETED, poor receive-time packet quality
- exp-20261120T100024884Z-2b3d9bae — EXP1_New_Run02 — COMPLETED, poor receive-time packet quality

Data under dual_arm_app/experiment_runs/<run_id> with raw, manifest, alignment and available analyses/CSVs unchanged.
Both late completed runs have Port10000 receive-time gaps exceeding 250 ms. Do not claim they passed measurement qualification.

## Validation evidence

- V1: 58bd540104944149b34d24e5eef42db2 — Left static capture, FK vs controller pose not verified.
- V2: eeebedfdac6d4644b2e319b03b80e89a — Preserve original evidence; consult session analysis.
- V4: 0ac21b8215974bdfb650e797199e9d83 — Left 5 A/B pairs, J6 origin proxy, characterisation only.
- V5: 933dadf29aa543da9e19b0fb7d8e058d — P2 paired captures only, P1 archived as incomplete, offset unverified.

Session files and metadata in dual_arm_app/validation_sessions/. Runtime lock files intentionally excluded.
20 November EXP saved path, readiness and validation-methodology documents also included.

Time provenance: Port10000 timestamps represent host receive/parse time, not confirmed controller acquisition times.
This GitHub repository is public; evidence includes joint trajectories and calibration snapshots.
