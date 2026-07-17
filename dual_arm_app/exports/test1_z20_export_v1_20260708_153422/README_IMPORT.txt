D40.1 Moveset Export

Moveset: test1_z20_export_v1
Object: Test1
Target delta: [0.0, 0.0, 20.0, 0.0, 0.0, 0.0]
Segments: 2

Generated files:
- moveset_both.json          Full neutral moveset data for both arms
- plan_raw.json              Raw D35.7 dry-run response
- left_waypoints_deg.csv     Left arm joint targets in degree
- right_waypoints_deg.csv    Right arm joint targets in degree
- left_jaka_program.jks      JAKA Script draft for left arm
- right_jaka_program.jks     JAKA Script draft for right arm
- left_tcpip_joint_move.jsonl  TCP/IP command-list draft for left arm
- right_tcpip_joint_move.jsonl TCP/IP command-list draft for right arm

Important:
1. These files were generated from dry-run only.
2. Running .jks files in JAKA Software can move the real robot.
3. Joint targets were converted from radian to degree for JAKA Script.
4. Left/right programs are independent. They do NOT guarantee dual-arm synchronization by themselves.
5. For a real cooperative lift, add a start handshake/interlock via IO before running both arms.
6. Review tool frame, user frame, payload, speed, acceleration, IO mapping, and workspace clearance.
