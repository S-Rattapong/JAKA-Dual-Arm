# JAKA Dual Arm — Real-Robot Preflight / Preliminary MQ Audit
Date: 2026-10-19 (Thailand time)
Status: NO-MOTION CHECKS ONLY; NOT FORMAL MQ PASS; NOT RELEASED FOR REAL-ROBOT EXP1–EXP4

## Scope
Audited running ROS drivers, MoveIt/Web Backend, execution and interlock, saved run evidence, passive Port10000 telemetry. No motion commands, no replan, no driver restart, no alteration to user paths, no Git push.

## Runtime
- Workspace: ~/jaka_ws; branch feature/coupled-ik-v1; HEAD ac514b53a261.
- ROS left/right drivers, MoveIt and Web Backend seen running.
- Robot-status eventually reported both LIVE, power=1, servo=1, motion=0, collision=0. Physical environment and E-stop status NOT verified.
- During audit, ROS cache briefly stale (~19.7s) for both arms despite fresh direct Port10000. Cause unknown; recovered without restart. Requires monitoring before trials.
- Port10000 read-only via /api/digital-twin/joints seen fresh for both arms.

## Passive Stream Smoke Test (not formal MQ)
Duration ~9 seconds, read-only snapshots through /api/digital-twin/joints. This is API polling; it is NOT guaranteed raw packet-complete capture.
- Left 83 distinct receive timestamps, mean observed gap 109.3ms, max 197ms, >400ms gaps 0.
- Right 82 distinct receive timestamps, mean observed gap 111.2ms, max 194ms, >400ms gaps 0.
- Expected direct packet rate ~10Hz. No motion executed by auditor.
- Conclusion: LIVE DURING THIS SAMPLE WINDOW, formal recorder V3 end-to-end and phase coverage NOT qualified in this check.

## Execution Interlock
- Phase 4 status READY only under CURRENT PROJECT VALIDATION SCOPE; Object/Environment collision and acceleration-limit checks are deferred (not passed).
- Recorder IDLE.
- Start Match threshold = 0.01 rad per joint, assessed against frozen artifact sample 0 using fresh ROS cached actual.
- Earlier at time of diagnosis max discrepancy 0.05046 rad. Later changed to 0.79656 rad (45.64deg) while user-facing runtime/artifact changed.
- Latest observed artifact generation 41, 1607 samples, 38.544 s. Latest execution feedback showed COMPLETED for gen 41, but current robot pose differs from that artifact's sample 0. This may be normal if prior path ended elsewhere; do not mistake it for an automatically recoverable initial state.
- Execute readiness blocked by ACTUAL_START_DELTA_EXCEEDS_THRESHOLD. DO NOT disable/bypass interlock.
- 'Move to Initial' can target remembered previous execution start, not necessarily current frozen artifact start; cannot presume safe, especially with attached workpiece.
- No physical safety clearance, grasp/payload details, or obstacle clearance independently verified.

## Historical datasets
- experiment_runs: 22 run dirs, 22 manifest/raw/aligned triplets parsed with consistent run IDs, 21 COMPLETED/CLOSED + 1 ARMED.
- EXP1_FORMAL: 9; EXP2_CLEARANCE old: 5; EXP2_FORMAL_CLEARANCE old: 5; EXP2_V2: 2 (one ARMED); EXP2_V3: 1.
- 16 EXP1 AVAILABLE analyses, 16 EXP2 AVAILABLE/PARTIAL analyses.
- Only 2 run directories have direct actual_packets. EXP2_CLEARANCE_V3_R01 has left 286/right 288 raw packets, phase-matched 524/540 usable (~97.0%), analysis status PARTIAL.
- Do not treat historical named FORMAL datasets as final new-protocol thesis evidence without qualifying changes in motion/measurement stack.
- Git workspace contains 46 tracked-deleted files from earlier runs and 22 present untracked run folders. Do NOT use git clean, git add ., or blindly restore/delete.
- Saved center paths: Test1 (translation-only), EXP1 (mixed), EXP2 (complex), TestA, TestC.
- Disk space checked ~90GB available.

## Remaining release gates
1. Operator independently confirms physical workspace clear, correct fixture/workpiece/grasp/payload, emergency stop availability, and ability to supervise each motion.
2. Resolve/approve exact frozen plan initial state; rerun Phase4 and start match against current actual. No automatic recovery motion.
3. Freeze Git SHA, base calibration revision, grasp and path revision, current motion settings, speeds, replicates and predeclared exclusion rules.
4. Formal MQ: synthetic measurement sanity test, direct packet recorder end-to-end qualification (data integrity, monotonic time, both arms, gaps and V3 fields), phase-match coverage acceptance, explicit predeclared thresholds.
5. Only afterward execute supervised robot runs: Campaign A shared EXP1–EXP3 evidence; Campaign B lightweight EXP4 with separate physical/fixture checks.
6. Preserve raw and provenance; generate CSV/plots and GitHub export after independent quality audit.

NO REAL ROBOT MOTION WAS COMMANDed BY THIS AUDIT.
