# Selective Scope Reset 2026-08-05

## Purpose

The previous reset direction removed too much of the working application. This selective reset removes only the named legacy object/servo/calibrated workflows while keeping the established basic dual-robot controls, persistence, and operator UI intact.

Restore point: tag `backup-v1-before-rewrite-2026-08-05`, commit `303fd6cd7ed58707ac57931f6c1a700273d77e87`.

## Preserved Features

- Live Position / Direct Move
- Direct Joint Move for Left, Right, and Both
- Direct TCP Move with generic TCP -> IK -> Joint Move
- Manual Jog in Joint, Base TCP, and Tool TCP modes
- Both-arm synchronization options and configured speed/acceleration values
- STOP BOTH, Home Both, Refresh Status
- Waypoint Manager and existing waypoint persistence
- Program / Sequence, existing program persistence, and all loop modes
- FastAPI, ROS/JAKA status and basic motion clients, configuration, console/error logging, and activity status

The detailed pre-edit mapping is in `docs/archive/selective_removal_map_2026-08-05.md`. The baseline route/button/data snapshot is in `docs/archive/preserved_feature_snapshot_2026-08-05.txt`.

## Removed Features

- D32.5C Servo Smooth Motion Test
- D32 Object Frame / Cooperative Setup
- D35 Object Frame Calibration
- D35.5A World/Base Calibration
- D35.5B Calibrated Rigid Preview With World Transform
- Workflow-specific D35.5D IK/joint-jump validation
- D35.6A Verified Object Pose Commit
- D35.6A-R Recover Pending Commit
- D39 calibrated world-rigid workflow
- D35.7 user-facing Action Log

## Active Architecture

`dual_jaka_web_backend.py` now has four active layers:

1. Left/right state subscriptions and generic JAKA service clients
2. Basic direct motion, jog, STOP, Home, and motion status handling
3. Waypoint and Program/Sequence persistence/execution
4. FastAPI routes and Web UI for those preserved capabilities

The active backend does not contain object-frame state, cooperative targets, calibration transforms, pending object commits, calibrated world-rigid execution, or servo test endpoints. Generic `GetIK` remains because Direct TCP Move depends on it.

## Preserved Routes

- `GET /`, `/api/status`, `/api/robot/activity`, `/api/waypoints`, `/api/program/list`
- `POST /api/jog/start`, `/api/jog/heartbeat`, `/api/jog/stop`
- `POST /api/stop`, `/api/home`
- `POST /api/waypoint/save`, `/api/waypoint/run`, `/api/waypoint/delete`
- `POST /api/sequence/run`, `/api/sequence/stop`
- `POST /api/program/save`, `/api/program/load`, `/api/program/delete`, `/api/program/run`
- `POST /api/direct/joint_move`, `/api/direct/tcp_move`

## Removed Routes

- `POST /api/servo/enable`, `/api/servo/tiny_test`
- All `/api/object/*` routes from the backup implementation
- All `/api/world_calibration/*` routes from the backup implementation

The exact removed route set is asserted in `dual_arm_app/tests/test_selective_scope_reset.py` using the pre-edit route inventory.

## Files Modified

- `README.md`
- `dual_arm_app/backend/dual_jaka_web_backend.py`
- `dual_arm_app/web/index.html`
- `dual_arm_app/tests/test_selective_scope_reset.py` (added)
- `docs/JAKA_DUAL_ARM_ACTIVE_ROADMAP.md` (added)
- `docs/SELECTIVE_SCOPE_RESET_2026-08-05.md` (added)
- `docs/archive/preserved_feature_snapshot_2026-08-05.txt` (added before source edits)
- `docs/archive/selective_removal_map_2026-08-05.md` (added before source edits)

## Files Deleted

- `dual_arm_app/tools/export_d35_7_moveset.py`
- `real_robot_scripts/06_servoj_anti_shake.sh`
- `real_robot_scripts/06_servoj_demo_large.sh`
- `real_robot_scripts/06_servoj_demo_large_dryrun.sh`
- `real_robot_scripts/06_servoj_dryrun.sh`
- `real_robot_scripts/06_servoj_smooth.sh`
- `real_robot_scripts/06_servoj_sync_dryrun.sh`
- `real_robot_scripts/06_servoj_sync_execute.sh`
- `real_robot_scripts/06_servoj_velocity_dryrun.sh`
- `real_robot_scripts/06_servoj_velocity_execute.sh`
- `real_robot_scripts/06_servoj_velocity_smooth_execute.sh`

The reusable Python servo bridge/transport primitives remain in place; only the test-specific `06_servoj_*` launch wrappers were removed.

## Data Integrity

Post-edit values match the pre-edit snapshot exactly:

| Path | Files | SHA-256 tree checksum | Result |
|---|---:|---|---|
| `dual_arm_app/programs` | 3 | `b72b8c3257e9e063a625391a9bd8dc499e9b94b2b3a16e81162f2e2fb9c89f6b` | unchanged |
| `dual_arm_app/tasks` | 1 | `ba3f7d2d067c8acb17896eba590f81caf75e0e6a079be9ab5c232ee9aac1dee0` | unchanged |
| `dual_arm_app/objects` | 2 | `78bd25bb7e4a663a8221c7bc38ebf7f56d6f2d32d7ebf360dbb79bc5e1d17a4e` | unchanged |
| `dual_arm_app/experiment_logs` | 25 | `a052b6f890dc7960012c43384a9e60f9189aefc461961344a878a54c8309d6ef` | unchanged |
| `dual_arm_app/exports` | 9 | `3e393e63c4df72888eb876b532919054a55d09443a2376da3c4a5ff9e3f501d9` | unchanged |
| `g4e_results` | 1 | `0c4852580a9bd13959ac8005a37be1a74ab0bf87361b71232c3e50d5d43c1fde` | unchanged |
| `real_robot_configs` | 5 | `409b2c123cbe0813667457560691f6098ef4e414b83acbb50150b3aa6ef55d3c` | unchanged |
| `src/jaka_ros2` | 1106 | `17e3d6bfd2c52353761ac77085d55962b98bf5774f58ee0db4bf36c2ad5619e9` | unchanged |
| root TF Graph artifacts | 6 | `be2f2538cb3ba894b957c31e50c930344386f23d41fc951458695447647430d6` | unchanged |

`Graph`, `Result`, and `dual_arm_desktop_app` directories were absent before and remain absent.

## Validation

- Backend Python syntax compilation: passed
- Standard-library mocked import/route/UI/data test suite: 8 tests passed
- HTML parser validation: passed
- Static active-source legacy scans: passed
- `pytest`: test file is pytest-discoverable, but the `pytest` executable is not installed in this environment
- JavaScript engine syntax check: unavailable because Node.js and equivalent engines are not installed; inline script structure and removed-reference scans were checked statically
- No backend server, ROS node, driver, or real client was started
- No HTTP endpoint or motion command was called

## Remaining Historical References

Inactive `*.bak*` files, archived logs/exports, object JSON files, `world_frame_calibration.json`, and unrelated ROS/prototype packages may still contain legacy names. They were intentionally preserved because they are historical or mixed-ownership assets and are not imported or rendered by the active Web application.

No new control algorithm has been implemented.
