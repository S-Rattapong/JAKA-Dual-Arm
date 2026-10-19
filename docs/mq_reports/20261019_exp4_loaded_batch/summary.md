# EXP4 Loaded Cooperative Motion — Raw Dataset Archive (2026-10-19)

**11 distinct Run IDs with 10 distinct display labels** were archived. `EXP4_LOADED_R07` occurs twice; both sessions retained, and neither is silently selected as the canonical seventh repetition.

All 11 have manifest `CLOSED` and execution `COMPLETED`, with 2,629 planned samples and a 63.072-second trajectory each. Plan fingerprints are identical. Each contains 7 original experiment files (77 total). The dataset is intact and checksummed against the original Ubuntu files.

Strict Measurement Quality: gaps greater than 250 ms between host receive timestamps occur in these runs. Do not claim that passing execution means all timing quality criteria passed. Existing `exp2_analysis.json` is *rigid grasp / relative pose* (thesis EXP3), not loaded pick-place repeatability. Its status `PARTIAL` is not a statement that all raw joint packets are missing.

A hook gripper with no open-close actuator is used. A single Execute contains both outward and return legs. No grasp/release sensor or external object pose ground-truth is recorded. Derive Left TCP/Right TCP/Object-center trajectories using URDF FK from `raw.json.actual_packets`, report host-clock and interpolation limits, and separately identify leg boundary waypoints.

Files: `summary.csv` (per-run info), `dataset_index.json` (metadata + SHA256), `checksums.sha256` (77 original files). Verify using `python tools/verify_exp4.py`.
