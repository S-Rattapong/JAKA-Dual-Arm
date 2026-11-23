# คู่มือ Validation V1–V5 ของ Dual JAKA A12

เลื่อนไป **ท้ายหน้า HMI** ใต้หัวข้อ **Legacy utilities · Utility Bay** แล้วกดเปิด Dropdown **Validation · V1–V5** ซึ่งแยกต่างหากจาก Digital Twin viewport และ Operator Panel เดิม หน้าแรกมีการทดลอง V1–V5 เลือกวิธีแล้วทำตามสี่ขั้น **Configure → Capture / import → Inspect → Analyze / export** หากหน้าเว็บแสดงว่า Validation API ยังไม่พร้อม แสดงว่าโปรเซส backend ที่ให้บริการอยู่ยังไม่ได้โหลดเราเตอร์รุ่นนี้ การ refresh โหลดได้เฉพาะไฟล์เว็บ

## เตรียมและบันทึก

1. ระบุจำนวนครั้ง เกณฑ์ หน่วย หลักฐานที่มาของเกณฑ์ และ uncertainty ก่อนเก็บตัวอย่าง ช่องเกณฑ์ว่างทั้งหมดใช้สำหรับวัดโดยยังไม่ตัดสินรับงาน สำหรับ run เก่า ให้ใส่เวลาตามเอกสารเดิมที่กำหนดเกณฑ์ก่อนเก็บข้อมูล เวลาในฟอร์มใช้เขตเวลาของเบราว์เซอร์
2. เมื่อบันทึก configuration ระบบสร้าง session ID และ snapshot ของ URDF, model metadata, calibration, hash, revision และข้อมูลที่กรอก หากเปลี่ยน Tool, probe, เกณฑ์ หรือ calibration ให้เริ่ม session ใหม่
3. สำหรับ V1/V2 เลือก saved run หรือเลือกไฟล์ campaign **raw.json** ในเครื่อง V1 รองรับการ capture แขนที่เลือกด้วย การนำเข้า run จะตรวจทุกแขนที่บันทึกมา V3–V5 ใช้ข้อมูลหน้างานจาก receiver เดิม เมื่อผู้ปฏิบัติงานจัดตำแหน่งและหยุดนิ่งแล้วจึงกด **Capture stationary window**
4. อ่านแขน socket และเลข trial ก่อนกดบันทึก ขั้น Inspect แสดงรายการ จำนวน packet ข้อมูล trial ผลตรวจการนิ่ง และแพ็กเก็ตล่าสุด เปิด **Analyze** เพื่อตรวจว่าหลักฐานครบหรือยัง ใช้ **เปิดงานล่าสุด** หลัง refresh หรือเลือก session ID ที่เคยบันทึก
5. **Exclude** เก็บตัวอย่างไว้พร้อมเหตุผลและนำออกจากการวิเคราะห์ **Retry** เก็บตัวอย่างเดิมพร้อมเหตุผลและกลับไปเก็บใหม่ **Unexclude** คืนตัวอย่างได้เมื่อช่อง trial ยังว่างและลำดับการจับคู่ถูกต้อง การเปลี่ยน trial ทำให้ต้องวิเคราะห์ใหม่

สามารถเปิดหัวข้อ **Stationary feedback quality** ในขั้น Configure ของ V1/V3/V4/V5 เพื่อปรับหน้าต่างเวลา จำนวนแพ็กเก็ต อายุข้อมูล ช่องว่างแพ็กเก็ต Joint drift และความเร็วข้อต่อได้ก่อนเริ่ม session โดยค่าที่ใช้ถูกเก็บใน configuration และใน raw sample แต่ละ Capture ค่าคัดข้อมูลนี้ไม่ใช่เกณฑ์รับงานหรือการรับรองความปลอดภัยของหุ่น การตั้งค่าที่ผ่อนเกณฑ์มากขึ้นอาจทำให้คุณภาพผลลดลง ต้องใช้เหตุผลจาก noise ของเครื่องมือและสภาพทดลองจริง

เกณฑ์คัดหน้าต่างนิ่งที่ใช้ในรุ่นนี้โดยค่าเริ่มต้นคืออย่างน้อย 1 วินาทีและ 6 packet sequence ที่ต่างกันต่อแขน แพ็กเก็ตล่าสุดอายุไม่เกิน 250 ms ช่องว่างระหว่างแพ็กเก็ตไม่เกิน 250 ms การเปลี่ยนมุมสูงสุดต่อข้อไม่เกิน 0.002 rad และความเร็วจาก finite difference ไม่เกิน 0.02 rad/s ต้องมี cached state สดของแขนที่เก็บ ระบบตรวจ motion state และความเร็ว JointState ที่มีด้วย ค่าคัดข้อมูลเหล่านี้แยกจากเกณฑ์รับงานของผู้ใช้ ใน V3 หน้าต่างของทั้งสองแขนต้องครอบคลุมช่วงร่วมอย่างน้อย 1 วินาที การเก็บครั้งถัดไปใช้หน้าต่างใหม่

## V1 · FK cross-check

ตรวจ **ความสอดคล้องระหว่างค่าควบคุมกับโมเดล** ใช้ actual_joints_rad ของแพ็กเก็ตเดียวกับ controller pose ข้อต่อผ่านการแปลงจากองศาเป็นเรเดียนแล้ว ใช้ J1–J6 ของแขนนั้นคำนวณ FK โดย URDF มี world-to-base อยู่แล้ว

ก่อนเปรียบเทียบต้องตรวจหลักฐานหน้างานของ **แต่ละแขนที่ใช้** ว่า actual_position หรือ position ที่เลือกแสดง TCP ใด หมายเลข Tool/TCP, offset และ RPY จาก J6 ไป TCP, หน่วยตำแหน่งและมุม, ลำดับการหมุน, Base/User/World และ ID ของเฟรมตรงกับโมเดลที่แสดง สำหรับ User frame ใส่ pose ของ User ใน World ด้วย รุ่นนี้รองรับ fixed-axis roll/pitch/yaw ที่ประกอบเป็น Rz·Ry·Rx เมื่อยืนยันว่าใช้ J6 ตรงกันจึงกรอก offset ศูนย์ได้ ค่า VALID ใน field_validation ยืนยันการอ่านฟิลด์เท่านั้น เอกสารของ campaign ระบุความหมาย controller raw ว่ายังไม่ยืนยัน

ผล signed XYZ คือ FK ลบ controller ใน World หน่วย mm พร้อม norm และมุมจากการหมุนสัมพัทธ์ SO(3) หน่วย deg ดู mean/RMS/max และกราฟตามเวลา เลือกเกณฑ์ position norm หรือ orientation ได้ ผล relative pose ระหว่างแขนแสดง UNKNOWN เมื่อยังไม่มีหลักฐานจับคู่ข้ามแขน การทดสอบนี้ให้หลักฐานความสอดคล้องทางจลนศาสตร์ การอ้างความแม่นยำทางกายภาพต้องมีการวัดอิสระ

## V2 · Host time shift

เลือก original run ที่มี planned.samples ต่อแขน, actual_packets และ execution.start_time_unix_ns กำหนดขอบล่าง ขอบบน และ step ของ τ (ms) ช่วงต้องครอบคลุมศูนย์ เทียบ Actual(t) กับ Plan(t−τ) ค่า τ บวกหมายถึง Actual ตามหลัง กริดทุกค่าใช้ packet ชุดเดียวกันต่อแขน ประเมินที่เวลา receipt จริง ใช้ interpolation เฉพาะของแผนในช่วงที่มีหลักฐาน ค่า gap เริ่มต้น 250 ms ไม่มีการเติมข้อมูล actual ระหว่างช่วงที่ packet หาย

ใช้ host monotonic receipt ที่ผูกกับ wall clock ของแพ็กเก็ตแรกเมื่อมี monotonic ครบ หากความต่างระหว่างนาฬิกาเปลี่ยนเกิน 10 ms ระบบจะปฏิเสธชุดนั้น ถ้าไม่มี monotonic ตลอดชุดจะระบุนาฬิกาที่ใช้ในผล อ่าน RMSE ก่อนและหลังเลื่อน residual RMSE ของทั้ง 6 ข้อ ค่า τ ซ้าย/ขวา และ Δτ = τR−τL สรุประหว่าง run แสดง SD แบบตัวอย่างเมื่อมีอย่างน้อยสอง run กราฟแสดง RMSE ตาม τ และ overlay แผนเดิม แผนที่เลื่อน และ actual กราฟแบนหรือ optimum ชิดขอบทำให้การระบุ delay ไม่น่าเชื่อถือ ใช้ผลนี้อธิบาย effective host-timed delay ซึ่งรวมการส่งข้อมูล การวัด controller lag และ EXP2 start/stop ต้องใช้หลักฐานคนละแบบ

## V3 · Surface gap

กำหนดจุดหนึ่งจุด (m) และ normal ของผิวแต่ละข้างใน tool frame ชี้ normal ซ้ายไปทิศช่องว่างที่นิยามว่าเป็นบวก ระบุว่าจะใช้ J6 หรือ mounted tool หากใช้ mounted ให้ระบุ Tool ID, ตำแหน่ง origin ใน J6 และ RPY (rad) พร้อมแหล่งอ้างอิง ระบุค่ามุมคลาดจากขนานที่ยอมรับได้ เกจยี่ห้อ/ชนิด ความละเอียด และ uncertainty

ใส่ pose ID, ทิศทาง, planned gap, trial แล้วบันทึกหน้าต่างนิ่งทั้งสองแขน กรอกระยะที่วัดจากเกจจริงหรือช่วง [low, high] ในหน่วย mm จะเติมค่าภายหลังใน Inspect ก็ได้ ระบบแปลงจุดและ normal ไป World แล้วฉายเวกเตอร์ระหว่างผิวตาม normal ซ้าย ตรวจความขนานจากมุมแคบระหว่าง normal ทั้งสอง ผลคือ measured − FK ดังนั้นค่าบวกหมายถึงช่องว่างจริงใหญ่กว่าที่ FK ให้ หากเกจให้เพียงช่วง ระบบส่งออกช่วง error ทั้งช่วง ถ้าช่วงคร่อมเส้นเกณฑ์จะได้ CANNOT_EVALUATE ช่อง planned gap ใช้ระบุท่าทดลองและแยกจากค่าที่วัดจริง

## V4 · Single-arm touch

เลือกแขนและตั้ง jig ID ใส่ระยะศูนย์กลาง A–B ที่วัดอิสระ หน่วย mm ระบุเครื่องมือ/บันทึกอ้างอิงและ uncertainty ใช้ **เพิ่มชุด jig** สำหรับหลายระยะ กรอก probe center ใน tool frame (m) และหลักฐานการวัด offset เลือก J6 หรือป้อน pose ของ mounted tool

สำหรับแต่ละ repeat เก็บ A และ B ด้วยแขนเดียวกัน แยกการจัดท่าและเก็บทีละครั้ง ดู context เช่น left · Jig 1 · Socket B · Trial 3/5 การวิเคราะห์ใช้ median ของแต่ละข้อในหน้าต่างนิ่ง ทำ FK ถึง probe center คำนวณระยะ 3 มิติแล้วลบระยะอ้างอิง ส่วน repeat dispersion เป็น radial RMS รอบ centroid แยกแขน/jig/socket ถ้ามีเพียงครั้งเดียว dispersion จะว่าง เมื่อขาด B ข้อมูล A ยังคงอยู่และผลรวมจะรอคู่ที่ครบ

## V5 · Common-point agreement

ตรวจ revision แหล่งที่มาของ world calibration และนิยาม probe ของทั้งสองแขน ระบุรายการรหัส socket ของ validation และ training แยกกัน วัดจุด validation ที่กระจายในสามมิติ รุ่นนี้ตรวจอย่างน้อย 4 จุดที่ไม่อยู่ระนาบเดียวกันโดยใช้ rank ของพิกัดศูนย์กลางและ tolerance 0.1 mm พร้อมตรวจจำนวน repeat ตามที่กำหนด

แต่ละ socket และ repeat ให้เก็บ **ซ้ายก่อน** แล้วจึงเก็บ **ขวาที่ socket เดิม** โดยหน้าต่างเวลาต้องไม่ทับกัน ตารางผลให้ XYZ ของ probe ซ้ายลบขวาใน World และความยาวเวกเตอร์หน่วย mm ดู mean/RMS/max ตาม socket ผล training แสดงแยกใน summary และ export โดยไม่รวมในเกณฑ์ validation

## แปลผลและส่งออก

| สถานะ | ความหมาย |
|---|---|
| CANNOT_EVALUATE / MISSING_INFO | ตรวจรายการ issues เช่น mapping ที่ขาด คู่ที่ยังไม่ครบ ผิวไม่ขนาน หรือ delay ที่ระบุไม่ได้ กรณีข้อมูลวัดครบให้ตรวจ criterion_issues และช่วงความไม่แน่นอนของ gauge ด้วย |
| MEASURED_NO_CRITERION | มีผลการวัดและเงื่อนไขข้อมูลครบ ยังไม่มีเกณฑ์ที่กำหนด |
| PASS / FAIL | เปรียบเทียบ metric ที่เลือกกับขีดจำกัดที่บันทึกไว้ก่อนการวัด ค่าความไม่แน่นอนประกอบการตีความ ผู้ใช้ต้องรวม guard band ที่ต้องการไว้ในเกณฑ์ของตนเอง |

**Data usable** แสดงความครบของหลักฐาน ส่วน **Acceptance** ใช้เกณฑ์คนละขั้น สำหรับ V5 จุด training แยกจากข้อมูลหลัก CSV มีแถว sample (รวม archived) และแถวผลต่อ packet/trial JSON เก็บ input, ประวัติ, calibration และ URDF ฉบับจริง ส่วน PNG มาจากผลที่วิเคราะห์แล้ว V2 เลือก run สำหรับกราฟได้ ตารางหน้าเว็บแสดงผลเบื้องต้นสูงสุด 100 แถว

Session เก็บใน dual_arm_app/validation_sessions เมื่อมีการบันทึกครั้งแรก saved run อ่านเฉพาะ raw.json ใน experiment_runs ขนาดสูงสุด 64 MiB การอัปโหลดจำกัด body 48 MiB และ 30 วินาที session ที่บันทึกมีขีดจำกัด 32 MiB ข้อมูลเดิมยังอยู่เมื่อบันทึกรายการใหม่เกินขีดจำกัด แบ่ง session ตามชุดทดลองได้

วิเคราะห์ไฟล์ export ซ้ำแบบออฟไลน์จากราก workspace ด้วย Python ที่มี NumPy และ SciPy

```python
import json
from pathlib import Path
from dual_arm_app.backend.validation_engine import analyze

session = json.loads(Path("exported-session.json").read_text())
result = analyze(session)
print(result["data_status"], result["acceptance"], result["issues"])
```

ใช้ engine_version เดียวกับไฟล์ export ตัววิเคราะห์ใช้ URDF และ calibration ภายในไฟล์นั้น เวลา received_monotonic_ns/received_at_ms เป็นเวลาเครื่อง host รับแพ็กเก็ต และ sequence index เป็นลำดับตลอดอายุ receiver

พื้นฐานการทดลองมาจากบันทึกประชุม 18 พฤศจิกายน 2026 [บันทึกวิธี Validation](https://app.notion.com/p/41c9d96468e380d091d2cf15dff5e1c9)
