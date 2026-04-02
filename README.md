# Factory Robotics GUI (CustomTkinter + MQTT)

## 1) ติดตั้ง

```bash
cd /Users/pop/FINAL_PRO_ROBOT
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 2) รันแอปควบคุมโรงงาน

```bash
python factory_robot_gui.py
```

### Payload MQTT ที่แอปรับ (สถานะจากฝั่งโรงงาน)

```json
{
  "conveyor_speed_mps": 1.45,
  "counts": {"good": 120, "bad": 5, "total": 125},
  "safety_sensor": "SAFE",
  "sensor_health": "Warning",
  "arm_runtime": {"moving": 250, "idle": 90, "error": 15},
  "conveyor2_passed": true,
  "temperature_c": 32.8,
  "humidity_pct": 61.4
}
```

- ถ้า Payload ถูกเข้ารหัสด้วย Fernet (key เดียวกับใน GUI) แอปจะถอดรหัสก่อน parse JSON อัตโนมัติ
- ปุ่ม `START`, `EMERGENCY STOP`, `SEND ANGLE` จะเข้ารหัส payload ก่อน publish ผ่าน MQTT

## 3) รัน Digital Twin 5-DOF

```bash
python digital_twin_5dof.py
```

### Payload MQTT สำหรับ Digital Twin

```json
{
  "joint_deg": [10, 20, -15, 30, 45]
}
```

- Digital Twin จะอัปเดตโมเดลแบบ real-time เมื่อได้รับมุมข้อต่อผ่าน MQTT
- มีการคำนวณ Forward Kinematics เพื่อแสดงตำแหน่งปลายแขนกล `(x, y, z)` บน GUI

## 4) โครงสร้างฐานข้อมูล SQLite

ไฟล์ DB: `factory_data.db`

ตาราง `conveyor2_log` เก็บข้อมูลเมื่อ `conveyor2_passed = true`
- `timestamp`
- `temperature_c`
- `humidity_pct`
# Final_Project_2_ROBOTIC
