# EXP2 การวัดช่วงเวลาเริ่มและหยุดจาก feedback ที่โฮสต์

ใช้ worktree ออฟไลน์ที่ฐาน `0926fed` สำหรับ replay และวิเคราะห์ไฟล์ ผล EXP2 ในคู่มือนี้หมายถึงช่วงเวลา start/settle ส่วนไฟล์เดิมชื่อ `exp2_analysis.json` เป็นผล rigid grasp ของ EXP3 ในวิทยานิพนธ์

## ทดสอบและประมวลผลออฟไลน์

รันจากราก worktree โดยใช้ไดเรกทอรีผลลัพธ์ใหม่สำหรับแต่ละครั้ง ตัวอย่างการตรวจสอบ

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python3 -m pytest -q --basetemp=.pytest_cache/exp2-local dual_arm_app/tests/test_exp*.py dual_arm_app/tests/test_host_endpoints_offline.py dual_arm_app/tests/test_port10000_actual_feedback.py dual_arm_app/tests/test_driver_port10000_rate_config.py dual_arm_app/tests/test_phase5_servo_timing_contention.py dual_arm_app/tests/test_mq_qualification.py
node --test dual_arm_app/tests/exp2_browser_performance.test.js
python3 tools/verify_campaign_a.py
python3 tools/verify_exp4.py
```

ผลตรวจของ worktree นี้ที่ฐาน `0926fed` ผ่าน pytest 131 รายการ และ Node 1 รายการ SHA256 ตรงกับรายการต้นฉบับ Campaign A 140/140 ไฟล์ และ EXP4 77/77 ไฟล์ การทดสอบจำลองยืนยันว่าปิด run ได้ทั้งเมื่อ settle และเมื่อหมดเวลาโดยไม่มี poll, callback รับ packet คืนค่าได้ระหว่างรอเขียนดิสก์ และค่าตัวเลข EXP1/EXP3 ตรงกับรอบจำลองที่ปิดตัวเลือกนี้ พร้อมใช้ตรวจไฟล์และ replay ออฟไลน์ การทำงานกับหุ่นยนต์จริงยังไม่ได้ทดสอบใน worktree นี้

สำหรับ run ที่บันทึกแล้ว ให้แทน `RUN/raw.json` ด้วยไฟล์จริงและส่งออกไปยังพื้นที่ทำงานใหม่ ตัวเลือก PNG ใช้ Matplotlib แบบ Agg และแสดงจุดที่วัดจริงทั้งหกข้อต่อ แถบสีเขียวและส้มแสดงช่วง start และ settle ที่ผ่านเกณฑ์ CSV เก็บมุมเป็นเรเดียนและ timestamp เต็มเป็น ns

```bash
python3 tools/exp2_dispatch_summary.py RUN/raw.json
PYTHONPATH=. python3 -m tools.exp2_event_zoom RUN/raw.json start_stop_zoom.png
PYTHONPATH=. python3 -m tools.exp2_batch_report dual_arm_app/experiment_runs experiment_start_stop.csv --label-prefix CA_PATH_A_
python3 tools/exp2_host_endpoints.py RUN/raw.json --samples-csv samples.csv --plot-png samples.png
python3 -m tools.exp2_endpoint_replay events.jsonl replay-runs
python3 tools/exp2_host_performance.py host.jsonl --samples 60 --interval-s 1
```

Replay รับหนึ่ง JSON object ต่อบรรทัด มี `event`, `host_ns` เรียงเวลา, `wall_ms` และ `data` โดย `arm` รับ argument ของ recorder พร้อม frozen artifact, Phase4 report และ metadata, `bind` รับ accepted execution, `packet` รับ side, actual_joints_rad และเวลา receive ทั้งสอง clock, `observe` รับ execution/driver_feedback และ `tick` เรียก finalize โดยไม่ต้องมี packet ใหม่ ดูตัวอย่างที่รันทดสอบได้ใน `dual_arm_app/tests/test_exp2_endpoint_capture.py` รหัสออก 1 หมายถึงยังปิดไม่สำเร็จหรือ recorder รายงานข้อผิดพลาด เช่น timeout

## หน้าต่างเก็บหลักฐานและเกณฑ์อ่านผล

เปิด **Capture EXP2 start and settle feedback** ก่อน Arm Recorder หรือกำหนด `endpoint_evidence` เป็น object ใน metadata ของการ arm ค่าเริ่มต้นเท่ากับ `{"pre_ms":1000,"post_ms":1000,"timeout_ms":5000,"hold_ms":100,"max_gap_ms":250,"angle_deg":0.05,"speed_deg_s":0.5}` เมื่อละ field นี้หรือส่ง false จะใช้การปิด run แบบเดิม ตัวเลือกในหน้าเว็บใช้ค่าตั้งต้น

- ระหว่าง ARMED เก็บข้อมูลย้อนหลังและหนึ่งจุดสำหรับคร่อมขอบหน้าต่าง จากนั้นเก็บทั้งสองแขนต่อเนื่องจนถึง terminal ที่ authoritative ของ trajectory เดียวกัน เวลาของ bind/terminal คือเวลาที่ backend สังเกตเหตุการณ์ การ arm สั้นเกินไปจะทำให้ baseline ไม่ครบ
- เมื่อเข้า FINALIZING จำนวน observation และ actual_packets เดิมหยุดเพิ่ม ช่องใหม่อยู่ใน `raw.json.endpoint_evidence` รุ่น `exp2-host-endpoints/v1` หลังครบ post_ms ทั้งสองแขนต้องมีตำแหน่งใน goal band และความเร็วต่ำต่อเนื่องตาม hold_ms จึงหยุดรับ tail หากเลย timeout จะปิดพร้อม TIMEOUT และแสดง error โดยคงผล COMPLETED/ABORTED/FAILED เดิม
- หลัง backend สังเกต terminal ที่ authoritative แล้ว worker ของ run นั้นจะปิดและเขียนไฟล์เองเมื่อทั้งสองแขนผ่านเกณฑ์ หรือเมื่อครบ timeout_ms แม้ไม่มี packet เพิ่ม ไม่ต้องเปิดหน้าเว็บ เรียก GET หรือส่ง observe ซ้ำ callback รับ packet เพียงเก็บข้อมูลและแจ้ง worker เมื่อผ่านเกณฑ์ ส่วนการเขียนไฟล์และวิเคราะห์เกิดบน worker
- หากเขียนไม่สำเร็จ recorder จะรายงาน error และคงสถานะ FINALIZING พร้อมข้อมูลที่ตรึงไว้ กด Refresh Recorder State เพื่อลองเขียนอีกครั้ง การปิดโปรเซสหรือเครื่องดับก่อนเขียน raw สำเร็จอาจทำให้ข้อมูลในหน่วยความจำสูญหาย เวลา timeout จำกัดช่วงรับ tail ส่วนเวลาปิดไฟล์ขึ้นกับดิสก์และการวิเคราะห์
- ปรับ threshold จากช่วงหยุดนิ่งของทั้งสองแขนก่อนประเมินผล ใช้ `stationary_max_deviation_deg` เทียบกับ planned start และ `stationary_max_speed_deg_s` ดู noise/offset ตรวจจุดวัดบนกราฟและบันทึกค่าที่เลือกไว้ล่วงหน้า ค่าเริ่มต้นมีไว้ตรวจวินิจฉัย สามารถ replay ด้วย `--angle-deg`, `--speed-deg-s`, `--hold-ms`, `--max-gap-ms` การเปลี่ยนเกณฑ์ภายหลังเพิ่มความยาวข้อมูลที่บันทึกแล้วไม่ได้
- Start ใช้คู่ตัวอย่างคร่อมการออกจาก start band ที่ยืนยันต่อเนื่อง ส่วน settle ใช้คู่ตัวอย่างที่เข้าเงื่อนไข goal band และความเร็วต่ำในช่วงนิ่งสุดท้าย จังหวะเริ่ม/หยุดทางกลจริงอาจอยู่ต่างจากเหตุการณ์ที่นิยามด้วย threshold ข้อมูลขาด, clock ซ้ำ/ย้อน, ลำดับ receiver ขาด/รีเซ็ต หรือ baseline/tail สั้นจะแสดงสาเหตุและช่วงที่สรุปไม่ได้
- ช่วงผลต่าง R−L เท่ากับ [R ต่ำ − L สูง, R สูง − L ต่ำ] หน่วย ms ค่าบวกทั้งช่วงหมายถึงโฮสต์เห็นขวาช้ากว่า ค่าลบทั้งช่วงหมายถึงเห็นขวาเร็วกว่า หากคร่อมหรือแตะศูนย์ ลำดับยังแยกไม่ได้ หลักฐานเก่าวัด tail เพิ่มย้อนหลังไม่ได้

## การวัดภาระของระบบ

ใช้ `received_monotonic_ns` ณ จุดรับ/parse, `recorder_delivery_ns` ณ ทางเข้า callback และ `recorder_append_ns` เมื่อได้ lock ก่อน append ในโฮสต์เดียวกัน `receive_gaps_since_arm` สรุปจำนวน ผลรวม และค่าสูงสุดของ gap บวกตั้งแต่ arm รวมช่วง pre ที่เลื่อนทิ้งแล้ว ส่วนรายงาน CLI สรุปเฉพาะจุดที่เก็บไว้ `received_at_ms` ใช้จับคู่บริบท wall clock เท่านั้นใน EXP2 นี้ เลข sequence เป็นตัวนับใน receiver การขาดลำดับเป็นสัญญาณให้ตรวจหลักฐาน จำนวน controller drops, queue depth, controller acquisition time, servo deadline ราย tick และ send-request time ราย tick ยังไม่มีการวัด

สำหรับ run ที่เปิด EXP2 ให้ดู `raw.json` ที่ `endpoint_evidence.driver_dispatch_summary.arms.left/right` หรืออ่านผล JSON ด้วย `tools/exp2_dispatch_summary.py` ตามตัวอย่างข้างต้น แหล่งข้อมูลคือ driver feedback ของ terminal เดียวกัน: `dispatch_timing` เก็บจำนวนตัวอย่าง สถิติ lateness และ missed_cycle_count, `servo_j_call_duration` เก็บจำนวนครั้ง เวลาเรียกและ overrun ส่วน `servo_stream` เก็บเวลาส่งและเวลากลับครั้งแรก/สุดท้ายบน host (Unix ns) กับ command_period_ms แขนที่ขาดข้อมูลแสดง UNAVAILABLE และแขนที่มีเพียงบางกลุ่มแสดง PARTIAL ไฟล์เดิมที่ไม่มีช่องนี้อ่านได้เป็น UNAVAILABLE ค่า missed cycle เป็นตัวนับของ driver ข้อมูลชุดนี้บอกเวลาราย tick, เวลา acquisition ของ controller และจำนวน packet ที่ตกหล่นจริงไม่ได้

Linux collector อ่าน CPU ticks รวมทุก core เรียง user/nice/system/idle/iowait/irq/softirq/steal, MemTotal/MemAvailable (KiB) จำนวน sector สะสมที่อ่าน/เขียนแยกตาม device ใน /proc/diskstats (512 byte ต่อ sector หลีกเลี่ยงการรวม disk กับ partition ซ้ำกัน) และตัวนับ I/O สะสมจาก /proc ของ PID ที่เลือก ใช้ผลต่างระหว่างแถวหารด้วยช่วงเวลาเพื่อหา rate ค่าเริ่มต้นวัด I/O ของ collector เอง ระบุ `--pid` เมื่อต้องการวัด process อื่นที่มีสิทธิ์อ่าน แต่ละแถวมีเวลา monotonic ก่อน/หลังอ่านและ wall ms สำหรับเทียบเวลาคร่าว ๆ

สำหรับ browser ให้เรียกเนื้อหาของ `tools/exp2_browser_performance.js` เป็น DevTools Snippet เฉพาะ tab และช่วงที่ต้องการ แล้วใช้ `exp2Performance.download("json")` หรือ `exp2Performance.download("csv")` เพื่อหยุดและดาวน์โหลด ใช้ JSON เพื่อเก็บ clock bridge ต้น/ปลาย, timeOrigin, ชนิด observer ที่รองรับ, จำนวน fetch/XHR ที่เริ่มระหว่าง request อื่น และจำนวน event ที่เกินเพดาน 20,000 CSV เก็บแถวเวลา frame, long task และ resource ตามที่ browser เปิดเผย การซ้อนทับใน browser บอกช่วงรอ request ซึ่งรวม network และ server งานใน tab เบื้องหลังอาจถูก throttle การวัดรอบใหม่เริ่มจากโหลด snippet ในหน้าใหม่ ควรเก็บทั้งช่วงปิดและเปิดเครื่องมือเพื่อเทียบ overhead

## Pilot เฉพาะรอบที่ได้รับอนุมัติแยกต่างหาก

1. ตรวจผล offline, checksum ข้อมูลตุลาคม และทบทวน checklist เริ่ม/หยุดกับผู้ควบคุมก่อนนำรุ่นนี้ไปใช้ในรอบที่อนุมัติ ใช้ speed/profile เดิมและคาบคำสั่ง servo **24 ms** เหมือนรอบอ้างอิง
2. เก็บ baseline ตอนแขนอยู่นิ่งนานกว่า pre_ms อย่างน้อยหนึ่งคาบ feedback เปิดบันทึก EXP2 ใน recorder สำหรับรอบนั้น ใช้ขั้นตอนควบคุมปกติที่ผ่านการอนุมัติ
3. รอ terminal และสถานะ CLOSED ก่อนจบรอบ หากยัง FINALIZING ให้ใช้ Refresh Recorder State ตรวจ error และไฟล์ raw ก่อนพิจารณาเริ่มรอบถัดไป
4. ให้ผ่านด้านคุณภาพข้อมูลเฉพาะรอบ COMPLETED ที่ baseline/tail ครบทั้งสองแขน มี start และ settle ครบ และไม่มี gap/reset/timeout ที่ทำให้ช่วงเวลาขาด กำหนด tolerance T (ms) ตามข้อกำหนดการทดลองล่วงหน้า หากต้องการอ้างว่าผล host อยู่ใน tolerance ทั้งช่วง start และ stop ต้องอยู่ภายใน [−T,+T] การคร่อมศูนย์เพียงอย่างเดียวไม่พอ หากช่วงกว้างกว่าเกณฑ์ให้รายงานความละเอียดไม่พอ
5. บันทึก period/gap จริง, threshold, ช่วง R−L และภาระระบบประกอบ การเทียบ browser กับ backend ผ่าน wall bridge มีความไม่แน่นอนจากคนละ clock, scheduling และ network หลักฐานนี้รองรับข้อสรุปบน host receive clock เท่านั้น การยืนยัน physical synchronization ต้องมีแหล่งเวลา acquisition หรือเครื่องมือวัดอิสระที่ตรวจสอบได้

## ตารางสรุปและกราฟเฉพาะจุดเริ่ม–หยุด (Offline)

ใช้คำสั่ง: PYTHONPATH=. python3 -m tools.exp2_batch_report RUN_ROOT output.csv --resolution-ms T --plots-dir PLOT_DIR

สร้างตาราง L/R start และ settle แบบช่วงเวลานับจาก Backend สังเกต Bind/Terminal พร้อม R−L และความกว้างช่วงความไม่แน่นอน ถ้าไม่กำหนด --resolution-ms จะไม่อ้างว่าความละเอียดเพียงพอ เพราะไม่มี Requirement ล่วงหน้า ถ้าบันทึก Evidence เวอร์ชันใหม่จะสร้างกราฟภาพรวมและภาพซูม 4 ช่องที่แสดงจุด Feedback จริงของแขนซ้าย/ขวาก่อน–หลังเหตุการณ์ พร้อม Angle Threshold และ Speed Threshold (ฝั่ง Stop) ห้ามใช้กราฟเหล่านี้อ้างว่าเวลาของ Controller หรือความผิดพลาดทางกายภาพยืนยันแล้ว การวิเคราะห์กับ Legacy Raw อาจรายงาน INCOMPLETE แม้เคยอนุมาน Start ได้จากวิธีอื่น เพราะหลักฐานก่อนเริ่มและหลังหยุดไม่ครบตามนิยามใหม่
