# MQ — Measurement Qualification สำหรับ EXP1–EXP4

วันที่เริ่มใช้: 2026-10-19  
ขอบเขต: **ระบบวัดและการบันทึกข้อมูลเท่านั้น** (ไม่ใช่การอนุมัติความปลอดภัยสำหรับการเคลื่อนที่)

## ใช้งานทันที

เปิด Terminal บน Ubuntu:

```bash
cd ~/jaka_ws
PYTHONDONTWRITEBYTECODE=1 python3 real_robot_scripts/mq_qualification.py \
  --run exp-20260911T082849318Z-bb3bd4b5 \
  --live-seconds 15 \
  --out docs/mq_reports/latest
```

ผลลัพธ์: `docs/mq_reports/latest/mq_report.md` และ `mq_report.json`  
หากขึ้น `MQ: PRE_RUN_READY` (exit code 0) แปลว่า **Offline regression + reference V3 evidence + passive live receive gate** ผ่านตามเกณฑ์ในเอกสารนี้ จึงสามารถเตรียมการเก็บ Run ใหม่ได้ แต่ยังไม่ใช่การอนุมัติขยับหุ่น

หาก `BLOCKED` (exit code 2) ต้องตรวจ failed gates ในรายงานก่อน ไม่ควรฝืนข้ามเกณฑ์หรือแก้ข้อมูล Raw ให้ PASS

คำสั่งนี้เป็นแบบ GET/Offline/read-only ต่อข้อมูลหุ่นจริง ไม่ใช้ Move/Execute/Arm/STOP/restart และไม่เปลี่ยนไฟล์ทดลองเก่า

## เกณฑ์ PRE-RUN MQ (Predeclared)

**Offline software regression:** ทดสอบ ExperimentRecorder, Port10000 framing, direct packet capture, provenance, alignment and gap rejection, phase projection, same-packet comparisons, falsification และ fail-closed gate tests รวม 68 cases ณ รอบเตรียมการ

**Historical V3 witness:** ใช้ `EXP2_CLEARANCE_V3_R01` เป็น *instrumentation qualification reference* (ไม่อ้างเป็น Final Thesis result)
- ยืนยัน Run ID ตรงกัน, execution completed, V3 raw schema
- SHA256 provenance ของ `raw.json`, `manifest.json`, URDF ต้องตรงกับผลวิเคราะห์
- เฉลี่ยอย่างน้อย 8 packets/วินาที/แขน (รวมอย่างน้อย 50 packets)
- field 6 joint actual และ 6 joint controller-reported อยู่ใน packet เดียวกันครบ 100%
- `received_monotonic_ns` และ packet sequence เพิ่มขึ้นเสมอ (strictly monotonic); `received_at_ms` อนุญาตค่าเท่ากันภายในมิลลิวินาทีเดียว แต่ห้ามย้อนหลัง
- ช่องว่างระหว่าง packet ติดต่อกันไม่เกิน 250 ms/แขน
- Analysis V3 same-packet = AVAILABLE
- Phase comparison = AVAILABLE/PARTIAL พร้อม coverage >= 95% และ >=100 candidate samples

**Live passive observation:** ตรวจ GET `/api/digital-twin/joints` ต่อเนื่อง 15s
- Source = `jaka_port10000_actual_feedback` ตลอดช่วงที่วัด
- เห็น receive timestamp ต่างกันไม่น้อยกว่า 7 ครั้ง/วินาที/แขน
- ช่วงห่างของ timestamp ที่สังเกตได้ <= 300 ms
- Feedback fresh (age <=400ms) อย่างน้อย 98% ของ snapshot
- ROS Robot/Joint status LIVE ทั้งสองแขนตอนสิ้นสุดการตรวจ
- Experiment recorder = IDLE

**การตีความ:** ค่าข้างต้นเป็นเกณฑ์ *screening data quality* สำหรับโครงการ ไม่ใช่ค่ารับรองจากผู้ผลิต ไม่รับประกัน packet loss เป็นศูนย์ และการสังเกต API snapshots ไม่ใช่การพิสูจน์ว่าได้ครบทุก packet ใน backend callback

## ระหว่างบันทึก EXP1–EXP4

1. กำหนด Experiment ID/Condition/Repeat ID พร้อมบันทึก SHA code, Path, Calibration, Grasp, Speed, Fixture, Sampling และ Software/Driver Config ให้ตรงกันทุก repeat (ห้ามปรับบาง run แบบเงียบ ๆ)
2. ตรวจ Plan + Phase4, frozen artifact และ run readiness ตามขอบเขตที่ระบบรองรับ; ตรวจพื้นที่จริงและ E-stop ก่อนสั่งหุ่น (MQ ไม่แทน Safety)
3. ใช้ Recorder ในหน้าเว็บ **ARMED** ผูกกับ Frozen Artifact เดียวกับที่จะ Execute
4. ให้ผู้ควบคุมดำเนินการ Real Execute จาก UI ตาม Gate และคำยืนยันของระบบ ห้ามใช้ script MQ เป็นตัวสั่งการ
5. หลังจบต้องได้ Recorder `CLOSED` / Run `COMPLETED` (หรือเก็บ FAILED/ABORTED เป็นหลักฐานแยก), `raw.json` และ `manifest.json` ห้ามแก้ raw
6. Analyze ในระบบให้มี `exp1_analysis.json` และ `exp2_analysis.json` ตามชนิดการทดลอง ตรวจ provenance ก่อนใช้ผล
7. รัน MQ Quality Gate กับ **Run ID ใหม่**:

```bash
cd ~/jaka_ws
PYTHONDONTWRITEBYTECODE=1 python3 real_robot_scripts/mq_qualification.py \
  --run <RUN_ID> --live-seconds 0 --skip-offline-tests \
  --out docs/mq_reports/<RUN_ID>
```

ผลที่ผ่านคือ `PRE_RUN_READY` จากส่วน `saved_run_gate` ซึ่งในบริบท Run ใหม่นำมาใช้คัดกรอง Evidence V3 ได้ หากผลเป็น `BLOCKED` ให้เก็บ Raw ไว้ แต่ทำเครื่องหมาย `DATA_QUALITY_REJECTED` หรือสอบสวนตาม Exclusion Criteria ที่ประกาศก่อนทดลอง ไม่ลบเพราะ Error สูงหรือ Task fail

**ข้อจำกัดสำคัญ:** script ตัวนี้ใช้ schema V3 แบบ dual-arm Port10000 สำหรับการทดลองการเคลื่อนที่; EXP4 ยังต้องตรวจ task success, payload/fixture และคุณภาพข้อมูลเชิงงานเพิ่มต่างหาก ไม่ใช่ใช้ MQ ตัวนี้แทนทั้งหมด

## เกณฑ์ที่ MQ ไม่ได้ตรวจหรือไม่สามารถยืนยันได้

- ค่าจาก Host Receive Timestamp **ไม่ใช่เวลา Controller Acquisition**
- Phase-Matched Relative Pose **ไม่ใช่การวัดสองแขนในเวลาโลกเดียวกัน**
- Joint Encoder + FK **ไม่ใช่ External Metrology**
- ไม่มี F/T Sensor ที่รับรองแรงจับระหว่างการยก
- อุปกรณ์จับชิ้นงานจริง, สิ่งกีดขวาง, น้ำหนักบรรทุก, E-stop, Interlock, collision checks ที่ถูก defer ต้องได้รับการตรวจ/ตัดสินใจโดยผู้ควบคุมแยกต่างหาก
- การรับข้อมูลสำเร็จก่อนทดลองไม่ได้รับประกันว่าช่วง Motion จะไม่มี Network/Driver stutter; ต้องตรวจทุก run เพิ่ม

## สำรองและส่งต่อให้ Windows

หลังจบแต่ละ Run ให้รักษา `manifest.json`, `raw.json`, `aligned.json`, ผล Analysis, CSV, รูป Plot และ `mq_report.json` พร้อมชื่อ Run ID และ hash ต่อเนื่อง

**ระวัง Working Tree ปัจจุบัน:** มี 46 tracked-deleted files และชุดทดลองเก่าที่เป็น untracked อย่า `git clean`, `git add .`, หรือ `git reset --hard` ก่อน audit; อย่า Push Raw Dataset ที่ยังไม่คัดและยังไม่ทราบขนาด/ข้อมูลส่วนตัว

## Files

- `real_robot_scripts/mq_qualification.py` — MQ CLI แบบ no motion
- `dual_arm_app/tests/test_mq_qualification.py` — synthetic fail-closed test suite
- `docs/mq_reports/latest/mq_report.md` — ผล Qualification ล่าสุด
- `docs/experiment_preflight_20261019.md` — audit รอบแรก
