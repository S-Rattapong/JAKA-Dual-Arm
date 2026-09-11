# Operator runtime

Run `./operator_runtime/install_operator_services.sh` to install the fixed systemd user units and Desktop launcher. Installation only reloads unit definitions. The workspace is expected at `~/jaka_ws`, with the ROS Humble workspace already built.

Open **JAKA Dual Arm Control** to start MoveIt and Web, follow their and driver logs in a terminal, and open an isolated Firefox profile. Connect Robots starts only missing driver sides. Like the Dev commands, operator drivers use `auto_enable:=true`: connection powers and enables the robot. Existing Dev drivers are detected and retained. Operator controls require the new driver services and fresh actual feedback.

Closing Firefox or signalling the launcher waits for read-only Phase5 status and fresh idle motion from both robots before stopping Web and MoveIt. Unknown status retains these services and prints a waiting warning. Drivers are never stopped by the launcher. Keep the terminal open while waiting. No STOP command is issued. A forced launcher kill cannot run cleanup; service units remain independent.

The status API reports Web RUNNING while it serves the page. If the backend dies, the frontend must subsequently display ERROR; there is no always-on supervisor. This change does not add the System Control frontend.

API: `GET /api/system-control/status`, `POST /api/system-control/connect-robots`, `POST /api/system-control/robot/{left|right}/{power|enable}` with `{"enabled": true|false}`, and `POST /api/system-control/reset-moveit`. Reset requires safe idle feedback and refuses external MoveIt. Power off requires an explicit prior disable. SDK acceptance is not actual-state confirmation; inspect subsequent RobotMsg feedback.
