# JAKA Dual Arm Active Roadmap

Updated: 2026-08-05  
Branch: `rewrite/selective-scope-reset-v2`

## Active Baseline

The active application remains a dual-robot FastAPI and Web UI control system. The following capabilities are intentionally preserved:

- Live left/right joint and TCP status
- Direct Joint Move for Left, Right, and Both
- Direct TCP Move through generic TCP-to-IK-to-Joint Move
- Manual Joint, Base TCP, and Tool TCP Jog
- STOP, Home, and status refresh
- Waypoint capture, persistence, list, run, add, and delete
- Program/Sequence persistence and execution, including once, fixed-count, and forever loop modes
- Basic ROS/JAKA clients, configuration loading, status reporting, motion logging, and error reporting

## Archived From Active Application

The following legacy high-level workflows are archived and no longer registered in the active backend or rendered in the active Web UI:

- D32.5C Servo Smooth Motion Test
- D32 Object Frame / Cooperative Setup
- D35 Object Frame Calibration
- D35.5A World/Base Calibration
- D35.5B Calibrated Rigid Preview With World Transform
- D35.5D object/calibrated-workflow IK and joint-jump validation
- D35.6A Verified Object Pose Commit
- D35.6A-R Recover Pending Commit
- D39 calibrated world-rigid operator workflow
- D35.7 user-facing Action Log

Generic IK used by Direct TCP Move remains active. Ordinary Base TCP/Tool TCP Jog, Waypoint Save, Program Save, and status/error logging are not part of the archived scope.

## Data Policy

Historical object records, world-frame calibration data, backups, experiment logs, exports, tasks, programs, waypoints, robot configurations, ROS descriptions, and generated thesis artifacts remain unchanged. Inactive backup files may still contain archived implementation text and are retained only as historical material.

## Next Phase

No replacement cooperative-control algorithm is implemented in this reset. Any future algorithm must begin from the preserved basic-control baseline and receive a separate design, safety review, and implementation task.

