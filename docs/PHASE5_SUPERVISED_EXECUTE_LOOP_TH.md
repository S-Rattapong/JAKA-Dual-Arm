# คู่มือ Phase 5 Supervised Execute Loop — JAKA Dual Arm
วันที่ 20 พฤศจิกายน 2569 | ทำงานใน Workspace เดิม /home/newtonrattapong/jaka_ws

## โหมดที่เพิ่ม
- Once: Execute Frozen Trajectory หนึ่งครั้งตาม workflow เดิม
- Fixed count: เลือก 2–999 รอบ ระบบตรวจเงื่อนไขใหม่ก่อนทุก Cycle
- Forever supervised: วนต่อเมื่อ Browser ส่ง Heartbeat ต่อเนื่อง และสูงสุด 60 นาทีต่อ Session เพื่อความปลอดภัย

การเลือกโหมดหรือเปิดหน้าเว็บไม่สั่งหุ่นจริง การเริ่มต้องคลิก Execute — REAL MOTION และยืนยันใน Dialog

## เงื่อนไขสำคัญ
1. Loop ต้องใช้ Frozen Trajectory ที่ Joint ของแขนซ้ายและขวาตำแหน่งสุดท้ายกลับมาเท่าจุดเริ่มภายใน 0.005 rad ต่อข้อเท่านั้น (Joint-Space Closed Path)
2. ถ้า W0 → Wn จบคนละ Joint Pose ระบบจะ BLOCK และไม่ Reverse หรือ Move to Initial โดยอัตโนมัติ ต้องสร้าง Return Path กลับมาที่ W0 อยู่ใน Frozen Trajectory เดียวกัน และผ่าน Validation/Ghost ใหม่ทั้งหมด
3. ทุก Cycle ผ่าน single-use Prepare Authority ใหม่, Phase4 Gate, Robot Safe State, Fresh Joint/Start Match, Frozen Fingerprint/Generation, Stop Generation และ Legacy Motion Conflict
4. การจบรอบต้องได้รับ authoritative COMPLETED ของทั้งสอง Driver พร้อม Trajectory ID ตรงกัน; ไม่ใช้เวลาคาดเดาจาก Wall-clock
5. มีช่วงพักอย่างน้อย 1 วินาทีหลังจบ Cycle ก่อนตรวจเงื่อนไขและเริ่มรอบถัดไป
6. ระหว่าง Loop บล็อก Jog/Home/Waypoint/Program/Direct Move และการเปลี่ยน Frozen Trajectory บางรายการ แต่ STOP BOTH ยังคงทำงานผ่าน Stop Path เดิม
7. Stop after current cycle จะหยุดวนหลังรอบปัจจุบันจบ ไม่หยุด Motion ปัจจุบันทันที
8. STOP BOTH / E-stop ใช้เมื่อต้องการหยุด Motion ทันที ตามวิธีหน้างาน
9. Browser ต้องเปิดอยู่และมองเห็นได้; หาก Heartbeat ขาดต่อเนื่อง 5 วินาที Backend จะเรียก Existing STOP BOTH Path ไม่รันรอบต่อไป
10. หาก Driver Feedback หายเกิน 5 วินาที ระบบใช้ STOP BOTH Path เช่นกัน
11. เปลี่ยน Fingerprint, Start State ไม่ผ่าน, Stop Generation เปลี่ยน, Error ของ Driver หรือ Controller จะหยุดการวนโดยไม่ Dispatch รอบใหม่
12. หาก Restart Backend จะไม่ Resume Loop เอง

ข้อจำกัด: Object/Environment Collision และ Acceleration Hard Limit (P4.5/P4.6/P4.20) ยังเป็นรายการ Deferred ไม่ถือว่า Loop ผ่าน Safety Certification

## วิธีใช้ในหน้าเว็บ
1. เตรียมเส้นทาง W0 → ... → W0 ให้ปิดจริง หรือเลือก Once หากเส้นทางไม่ปิด
2. Lock Grasp, Plan, Validate, Freeze และตรวจ Frozen Ghost ตามระบบเดิม
3. ที่ Step 3 · Preview & Execute เลือก Execute mode: Once, Fixed count หรือ Forever
4. ถ้าเลือก Fixed ให้ใส่จำนวน Cycles 2–999
5. ตรวจสถานะ JOINT-CLOSED กับค่าความต่างปลายทาง หากขึ้น LOOP BLOCKED อย่ากด Execute
6. ตรวจพื้นที่จริง, Fixture, Payload และ STOP/E-stop ก่อนกด Execute — REAL MOTION
7. กด OK ที่ Confirmation Dialog จึงอนุญาตให้ระบบรันหลายรอบโดยผ่าน Gate ใหม่ทุกครั้ง
8. Monitor LOOP RUNNING และ Completed k/N พร้อม Driver Status ระหว่างการทำงาน
9. ต้องการจบหลังรอบปัจจุบัน กด Stop after current cycle
10. ต้องการหยุดทันที กด STOP BOTH

อย่าเปลี่ยน Tab หรือปิด Browser ระหว่าง Loop: เมื่อ Heartbeat หาย ระบบจะหยุดด้วย Timeout

## ผลกับ Recorder
Loop ไม่ได้เปลี่ยน EXP2 Recorder หรือบันทึก Run แยกให้ทุก Cycle อัตโนมัติ หากต้องการเก็บผลทุก Cycle แยกโดยอัตโนมัติต้องเพิ่ม Multi-Cycle Recorder ในงานอื่น

## ไฟล์สำคัญ
- dual_arm_app/backend/phase5_supervised_loop.py: State machine, authoritative feedback, STOP latch, heartbeat
- dual_arm_app/backend/dual_jaka_web_backend.py: เส้นทาง API, Interlock, STOP integration
- dual_arm_app/web/index.html: Mode/Count/Stop/Progress
- dual_arm_app/web/digital_twin_phase5_execution.js: UI, Confirmation, Lease
- dual_arm_app/tests/test_phase5_supervised_loop.py: Offline unit test
- dual_arm_app/tests/test_phase5_supervised_loop_ui.py: Offline browser test

## Endpoints
- GET /api/digital-twin/phase5/loop/status — Read-only
- POST /api/digital-twin/phase5/loop/start — REAL MOTION เมื่อผู้ใช้ยืนยันเท่านั้น
- POST /api/digital-twin/phase5/loop/heartbeat — ต่ออายุ Browser Supervision, NO MOTION
- POST /api/digital-twin/phase5/loop/cancel — ยกเลิกการวนหลังรอบปัจจุบัน, NO MOTION

## การนำขึ้นระบบจริง
Backend Uvicorn ขณะพัฒนาไม่มี --reload. การแก้ Python ไม่เข้าโปรเซสที่รันอยู่จนกว่าจะ Restart. ต้องขออนุญาตผู้ใช้ก่อน Restart และก่อนการ Execute หุ่นจริง

Pilot ควรเริ่มจาก Fixed count = 2 เฉพาะ Closed Path ที่ผ่าน Validation และ Safety Check จริงก่อน จึงพิจารณา Forever

## แก้ไขปัญหา Cycle 2 ไม่เริ่ม (20 พฤศจิกายน 2569 เวลา 13:02 น.)

พบจาก Status API จริงของรอบทดลองที่ผู้ใช้เลือก Fixed count = 5:
- Loop ID: `p5loop-778157d5b46743f0931bc7bcec10d4d6`
- Mode `count`, count `5`, completed_cycles `1`, current_cycle `2`, state `FAILED`
- Loop reason เดิม: `REPEAT_EXECUTE_REJECTED: PHASE5_DUAL_TRAJECTORY_SUBMISSION_FAILED`
- Driver ล่าสุด Left/Right ได้ trajectory ใหม่ `p5-cf5ecb38cbd04c72ba30661f3f9bfe5e`, แต่จบ `ABORTED: CANCELLED_BEFORE_START` และไม่มีตัวอย่าง ServoJ dispatch
- Frozen joint path ปิดจริง: max delta `2.566794e-7 rad`; Start Match ปัจจุบันพร้อม ไม่ใช่สาเหตุเพราะ Path ไม่วนกลับ
- Backend stop_generation เพิ่มจาก 0 เป็น 1 ใน failure path

**ไม่สามารถย้อนยืนยันสาเหตุย่อย** ของ `PHASE5_DUAL_TRAJECTORY_SUBMISSION_FAILED` จาก API เดิมได้ เพราะ Loop Manager เก็บเพียง `error` และทิ้ง `response.detail` ซึ่งจะบอกว่า `LEFT/RIGHT_TRAJECTORY_ACCEPTANCE_TIMEOUT`, `INSUFFICIENT_MARGIN_BEFORE_COMMON_START` หรือ `EXECUTION_AUTHORITY_CHANGED_AFTER_DRIVER_ACCEPTANCE`. ดังนั้นยังห้ามอ้างว่า Timeout เป็นต้นเหตุที่พิสูจน์แล้ว

การแก้แบบ Fail-closed:
1. วง Loop ใช้ Window ตอบรับ Driver ที่กว้างขึ้นเฉพาะรอบที่ทำซ้ำ: `common_start_lead_s = 4.0` (จาก 2.0) และ `acceptance_timeout_s = 1.5` (จาก 0.75) เพื่อเผื่อเวลาตั้ง Servo Mode/Filter; **Once ยังคง 2.0 และ 0.75 เหมือนเดิม**
2. Validation ก่อนส่งคำสั่ง, fingerprint, Generation, phase4 gate, Stop generation, safety, start match และ post-driver-acceptance check ยังคงอยู่; ไม่มี Auto Retry ที่อาจทำให้หุ่นเคลื่อนที่ซ้ำโดยไม่ตั้งใจ
3. เมื่อเกิด rejection อีก จะเก็บ `last_rejected_execution` พร้อม cycle, `error`, `detail`, blocking reasons, binding mismatches และส่งข้อความต้นเหตุขึ้น `loop.reason` ใน status API
4. Regression ออฟไลน์ 153/153 ผ่าน รวม mock accepted late 1.9s, ความต่างการตั้งเวลา Once/Loop, postacceptance Safety Gate failure, Stop/Heartbeat และ UI. ผลอยู่ที่ `~/Downloads/phase5loop_20261120/failure_diagnosis/regression_after_second_cycle_fix.log`

**ยังไม่ได้ Restart Backend หลังแก้**: Uvicorn ที่รันอยู่จะยังใช้ Python เก่าและยังไม่มี diagnostics/timing window ใหม่นี้ ต้องขอผู้ใช้อนุญาต Restart Web Backend อย่างชัดเจนก่อน

ก่อนทดสอบจริง:
- ตรวจว่าหุ่นหยุดนิ่ง, ไม่มี error, Driver Both Ready, Frozen Path Closed และพื้นที่งานปลอดภัย
- หลัง Restart ให้ GET `/api/digital-twin/phase5/loop/status` / ตรวจ PID, readiness และไม่มี loop active
- ขออนุญาต Motion Pilot แยกจากผู้ใช้ เริ่ม Fixed count=2 ก่อน ไม่กระโดดไป 5 หรือ Forever
- ถ้ายัง FAILED ให้หยุด ไม่ Restart แบบสุ่ม และเก็บ `last_rejected_execution.detail` ก่อนแก้ครั้งต่อไป
