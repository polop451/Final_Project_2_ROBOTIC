import paho.mqtt.client as mqtt
from robodk.robolink import *
import json

# --- ตั้งค่า RoboDK ---
RDK = Robolink()
robot1 = RDK.Item('Dobot1', ITEM_TYPE_ROBOT)
robot2 = RDK.Item('Dobot2', ITEM_TYPE_ROBOT)

# --- ฟังก์ชันจัดการสำหรับ Dobot1 ---
def on_message_dobot1(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print(f"🤖 Dobot1 Move to: {data}")
        if robot1.Valid():
            robot1.MoveJ(data, blocking=False)
    except Exception as e:
        print(f"Error Dobot1: {e}")

# --- ฟังก์ชันจัดการสำหรับ Dobot2 ---
def on_message_dobot2(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print(f"🤖 Dobot2 Move to: {data}")
        if robot2.Valid():
            robot2.MoveJ(data, blocking=False)
    except Exception as e:
        print(f"Error Dobot2: {e}")

# --- Callback หลักเมื่อเชื่อมต่อสำเร็จ ---
def on_connect(client, userdata, flags, rc):
    print("✅ Connected to Broker")
    # Subscribe พร้อมกันหลาย Topic โดยใช้ wildcard หรือระบุเจาะจง
    client.subscribe("factory/dobot1/joints")
    client.subscribe("factory/dobot2/joints")

client = mqtt.Client()
client.on_connect = on_connect

# 💡 หัวใจสำคัญ: แยกฟังก์ชันตาม Topic
client.message_callback_add("factory/dobot1/joints", on_message_dobot1)
client.message_callback_add("factory/dobot2/joints", on_message_dobot2)

# เชื่อมต่อและรัน
client.connect("test.mosquitto.org", 1883, 60)
client.loop_forever()