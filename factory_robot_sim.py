"""
factory_robot_sim.py
RoboDK Simulation — mirrors factory_robot.py (real robot control)

Differences from real code:
  - DobotApiDashboard / DobotApiMove replaced by RoboDK MoveJ
  - Hardware feedback sockets replaced by simulated DI flags
  - dash.DO() replaced by sim_DO() (prints + updates sim state)
  - Rail travel simulated with a 1.5-second delay
  - Robot2 picking clears DI10 of Rail1 automatically
  - EmergencyStop sets is_running = False
"""

import paho.mqtt.client as mqtt
import json
import threading
import time
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk.robomath import transl, rotz

# =============================================================================
# --- MQTT CONFIGURATION ---
# =============================================================================
MQTT_BROKER   = "broker.emqx.io"
TOPIC_R1      = "kmutt/cpe393/robot1/command"
TOPIC_R2      = "kmutt/cpe393/robot2/command"
TOPIC_BOTH    = "kmutt/cpe393/all/command"

TOPIC_R1_JOINTS = "factory/dobot1/joints"
TOPIC_R2_JOINTS = "factory/dobot2/joints"
TOPIC_R1_SENSOR = "factory/dobot1/irsensor"
TOPIC_R2_SENSOR = "factory/dobot2/irsensor"

# =============================================================================
# --- RoboDK SETUP ---
# =============================================================================
RDK = Robolink()
robot1 = RDK.Item('Dobot1', ITEM_TYPE_ROBOT)
robot2 = RDK.Item('Dobot2', ITEM_TYPE_ROBOT)

if not robot1.Valid():
    raise RuntimeError("Dobot1 not found in RoboDK — check robot name in station.")
if not robot2.Valid():
    raise RuntimeError("Dobot2 not found in RoboDK — check robot name in station.")

print("RoboDK: Dobot1 and Dobot2 found.")

# ── Rail mechanisms ──────────────────────────────────────────────────────────
rail1_item = RDK.Item('Rail1')
rail2_item = RDK.Item('Rail2')

if rail1_item.Valid():
    print("RoboDK: Rail1 found.")
else:
    print("[WARN] Rail1 not found in RoboDK station — rail animation disabled.")
    rail1_item = None

if rail2_item.Valid():
    print("RoboDK: Rail2 found.")
else:
    print("[WARN] Rail2 not found in RoboDK station — rail animation disabled.")
    rail2_item = None

# Speed — matches dash1/dash2.SpeedFactor(80) in real code
robot1.setSpeed(10)
robot2.setSpeed(10)

# =============================================================================
# --- CHECKPOINTS: ROBOT #1 ---
# Joint order: [J1, J2, J3, J4, 0, 0]  (MG400 = 4 DOF, RoboDK model has 6)
# =============================================================================
R1_HOME      = [11.64,   1.36,   1.79, 207.48, 0, 0]
R1_WAIT      = [45.00,   1.36,   1.79, 207.48, 0, 0]
R1_PICK      = [45.00,   3.36,   1.79, 207.48, 0, 0]
R1_GOOD      = [17.44,  41.39,  33.53, 207.48, 0, 0]
R1_IPBAD     = [47.98,  11.87,  46.92, 207.48, 0, 0]
R1_BAD_JOINT = [-71.00, 11.87,  46.92, 207.48, 0, 0]
R1_BAD_BACK  = [11.64,  11.87,  46.92, 207.48, 0, 0]

R1_POSITIONS = {
    'home':      R1_HOME,
    'wait':      R1_WAIT,
    'pick':      R1_PICK,
    'good':      R1_GOOD,
    'ipbad':     R1_IPBAD,
    'bad_joint': R1_BAD_JOINT,
    'bad_back':  R1_BAD_BACK,
}

# =============================================================================
# --- CHECKPOINTS: ROBOT #2 ---
# =============================================================================
R2_HOME  = [58.93,  41.64, 37.43, -175.09, 0, 0]
R2_PICK  = [90.98,  52.69, 70.65, -175.09, 0, 0]
R2_PLACE = [18.39,  37.76, 26.03, -175.09, 0, 0]

R2_POSITIONS = {
    'home':  R2_HOME,
    'pick':  R2_PICK,
    'place': R2_PLACE,
}

# =============================================================================
# --- RAIL ANIMATION THREADS ---
# Increments the mechanism joint to simulate belt motion while running
# =============================================================================
_rail1_running = False
_rail2_running = False
_RAIL_STEP     = 8.0    # degrees per tick (adjust to match belt speed visually)
_RAIL_INTERVAL = 0.05   # seconds between ticks (~20 fps)
_RAIL_WRAP     = 360.0  # wrap joint back to 0 after full rotation


def _rail_animate(item, flag_fn):
    """Background thread: spin rail mechanism joint while flag_fn() is True."""
    pos = 0.0
    while True:
        if flag_fn() and item is not None and item.Valid():
            pos = (pos + _RAIL_STEP) % _RAIL_WRAP
            try:
                item.setJoints([pos])
            except Exception:
                pass
        time.sleep(_RAIL_INTERVAL)


threading.Thread(
    target=_rail_animate,
    args=(rail1_item, lambda: _rail1_running),
    daemon=True,
).start()

threading.Thread(
    target=_rail_animate,
    args=(rail2_item, lambda: _rail2_running),
    daemon=True,
).start()

# =============================================================================
# --- SIMULATED DIGITAL I/O ---
# Replaces hardware feedback sockets + DI polling
# =============================================================================
_sim_di1 = {}   # {port_index: 0/1}
_sim_di2 = {}

def get_di1(index):
    return _sim_di1.get(index, 0)

def get_di2(index):
    return _sim_di2.get(index, 0)

def _set_di1(index, val):
    _sim_di1[index] = val

def _set_di2(index, val):
    _sim_di2[index] = val


def sim_DO1(port, val):
    """Simulate Robot1 digital output (replaces dash1.DO)."""
    global _rail1_running
    state = "ON" if val else "OFF"
    label = {9: "Suction-IN", 10: "Suction-OUT", 2: "Rail1", 1: "Pusher1"}.get(port, f"DO({port})")
    print(f"[Robot1 SIM] {label} → {state}")
    if port == 2:          # Rail1 belt motor
        _rail1_running = bool(val)


def sim_DO2(port, val):
    """Simulate Robot2 digital output (replaces dash2.DO)."""
    global _rail2_running
    state = "ON" if val else "OFF"
    label = {9: "Suction-IN", 10: "Suction-OUT", 2: "Rail2", 1: "Pusher2"}.get(port, f"DO({port})")
    print(f"[Robot2 SIM] {label} → {state}")
    if port == 2:          # Rail2 belt motor
        _rail2_running = bool(val)

# =============================================================================
# --- GLOBAL STATE ---
# =============================================================================
is_running     = True
rail1_count    = 0
robot2_can_get = False
r1_busy        = False
product_lock   = threading.Lock()
mqtt_client    = None   # set after MQTT client is created below

# =============================================================================
# --- MQTT PUBLISH HELPERS ---
# =============================================================================
def publish_joints(robot_id: int, joints: list):
    """Publish joint state after every move (uses first 4 joints)."""
    if mqtt_client is None:
        return
    topic   = TOPIC_R1_JOINTS if robot_id == 1 else TOPIC_R2_JOINTS
    payload = json.dumps({
        "j1": joints[0],
        "j2": joints[1],
        "j3": joints[2],
        "j4": joints[3],
    })
    mqtt_client.publish(topic, payload)
    print(f"[MQTT] Robot{robot_id} joints → {payload}")


def publish_ir_sensor(robot_id: int, state: int):
    """Publish IR sensor state (1 = product present, 0 = cleared)."""
    if mqtt_client is None:
        return
    topic   = TOPIC_R1_SENSOR if robot_id == 1 else TOPIC_R2_SENSOR
    payload = json.dumps({"state": state})
    mqtt_client.publish(topic, payload)
    print(f"[MQTT] Robot{robot_id} irsensor → {payload}")

# =============================================================================
# --- MOVE HELPERS ---
# =============================================================================
def r1_move(name: str):
    """MoveJ Robot1 to named checkpoint, then publish joints."""
    pos = R1_POSITIONS[name]
    print(f"[Robot1] MoveJ → {name}: {pos[:4]}")
    robot1.MoveJ(pos, blocking=True)
    publish_joints(1, pos)


def r2_move(name: str):
    """MoveJ Robot2 to named checkpoint, then publish joints."""
    pos = R2_POSITIONS[name]
    print(f"[Robot2] MoveJ → {name}: {pos[:4]}")
    robot2.MoveJ(pos, blocking=True)
    publish_joints(2, pos)


def r1_moveL_xyz(x, y, z, r):
    """Cartesian linear move for Robot1 (MQTT 'move' action)."""
    pose = transl(x, y, z) * rotz(r)
    print(f"[Robot1] MoveL → x={x} y={y} z={z} r={r}")
    robot1.MoveL(pose, blocking=True)
    joints = robot1.Joints().list()
    publish_joints(1, joints)


def r2_moveL_xyz(x, y, z, r):
    """Cartesian linear move for Robot2 (MQTT 'move' action)."""
    pose = transl(x, y, z) * rotz(r)
    print(f"[Robot2] MoveL → x={x} y={y} z={z} r={r}")
    robot2.MoveL(pose, blocking=True)
    joints = robot2.Joints().list()
    publish_joints(2, joints)

# =============================================================================
# --- ROBOT 1 SEQUENCE (MQTT triggered) ---
# =============================================================================
def run_robot1_sequence(is_good):
    global r1_busy, rail1_count

    if r1_busy:
        print("[Robot1] Busy — ignoring command.")
        return

    if is_good:
        with product_lock:
            count = rail1_count
        if count >= 2:
            print(f"[Robot1] GOOD blocked — Rail1 full ({count}/2).")
            return
        if get_di1(10) == 1:
            print("[Robot1] GOOD blocked — product still at Rail1 end (DI10=1).")
            return

    r1_busy = True
    try:
        print(f"[Robot1] Sequence: {'GOOD' if is_good else 'BAD'}")

        r1_move('home')
        r1_move('wait')
        r1_move('pick')

        # Suction ON — pick product
        sim_DO1(9, 1)
        time.sleep(0.2)
        sim_DO1(9, 0)

        if is_good:
            r1_move('good')
        else:
            r1_move('ipbad')
            r1_move('bad_joint')

        # Suction OFF — release product
        sim_DO1(10, 1)
        time.sleep(0.5)
        sim_DO1(10, 0)

        if is_good:
            with product_lock:
                rail1_count += 1
            print(f"[Robot1] Released GOOD product. Rail1 count: {rail1_count}")
        else:
            r1_move('bad_back')

        r1_move('home')
        print("[Robot1] Back at HOME.")

    except Exception as e:
        print(f"[Robot1] Error: {e}")
    finally:
        r1_busy = False

# =============================================================================
# --- RAIL 1 LOOP ---
# Simulates belt movement with a delay instead of hardware DI polling
# Publishes IR sensor state changes on factory/dobot1/irsensor
# =============================================================================
def rail1_loop():
    global rail1_count, robot2_can_get

    while True:
        if is_running:
            with product_lock:
                count = rail1_count

            if count > 0:
                print(f"[Rail1] Product on belt ({count}). Running rail...")
                sim_DO1(2, 1)   # Rail 1 ON

                # Simulate belt travel — product reaches end after 1.5 s
                _set_di1(10, 0)
                time.sleep(1.5)
                _set_di1(10, 1)

                publish_ir_sensor(1, 1)
                print("[Rail1] Product at end. Stopping rail.")
                sim_DO1(2, 0)   # Rail 1 OFF

                robot2_can_get = True
                print("[Rail1] robot2_can_get = True")

                # Wait until Robot2 picks the product (clears DI10)
                while get_di1(10) == 1:
                    time.sleep(0.05)

                publish_ir_sensor(1, 0)
                print("[Rail1] Product taken. Rail1 ready.")
                with product_lock:
                    rail1_count -= 1
                print(f"[Rail1] Rail1 count: {rail1_count}")

        time.sleep(0.1)

# =============================================================================
# --- ROBOT 2 + RAIL 2 LOOP (autonomous) ---
# Publishes IR sensor state changes on factory/dobot2/irsensor
# =============================================================================
def robot2_and_rail2_loop():
    global robot2_can_get

    print("[Robot2] Moving to HOME...")
    r2_move('home')
    print("[Robot2] At HOME — waiting...")

    while True:
        if is_running and robot2_can_get:
            print("[Robot2] Signal received! Starting pick sequence...")
            try:
                r2_move('pick')

                # Suction ON
                sim_DO2(9, 1)
                time.sleep(0.2)
                sim_DO2(9, 0)

                # Picking from Rail1 end — clear DI10 of Rail1
                _set_di1(10, 0)

                r2_move('place')

                # Suction OFF — release onto Rail 2
                sim_DO2(10, 1)
                time.sleep(0.5)
                sim_DO2(10, 0)

                print("[Robot2] Product placed on Rail2. Starting Rail2...")
                sim_DO2(2, 1)   # Rail 2 ON

                # Simulate belt travel
                _set_di2(10, 0)
                time.sleep(1.5)
                _set_di2(10, 1)

                publish_ir_sensor(2, 1)
                print("[Robot2] Product at end of Rail2. Stopping Rail2.")
                sim_DO2(2, 0)   # Rail 2 OFF

                publish_ir_sensor(2, 0)

                r2_move('home')
                print("[Robot2] Back at HOME.")

            except Exception as e:
                print(f"[Robot2] Error: {e}")
            finally:
                robot2_can_get = False
                print("[Robot2] robot2_can_get = False. Waiting...")

        time.sleep(0.1)

# =============================================================================
# --- MQTT ---
# =============================================================================
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"MQTT Connected! rc={rc}")
    client.subscribe(TOPIC_R1)
    client.subscribe(TOPIC_R2)
    client.subscribe(TOPIC_BOTH)


def on_message(client, userdata, msg):
    global is_running
    try:
        data  = json.loads(msg.payload.decode())
        topic = msg.topic
        print(f"[MQTT] {topic} → {data}")

        if topic == TOPIC_R1:
            action = data.get('action')
            if action == "process":
                is_good = (data.get('result') == "good")
                threading.Thread(
                    target=run_robot1_sequence,
                    args=(is_good,),
                    daemon=True,
                ).start()
            elif action == "move":
                threading.Thread(
                    target=r1_moveL_xyz,
                    args=(data['x'], data['y'], data['z'], data.get('r', 0)),
                    daemon=True,
                ).start()
            elif action == "Pusher":
                sim_DO1(1, 1 if data.get('status') == "ON" else 0)

        elif topic == TOPIC_R2:
            action = data.get('action')
            if action == "move":
                threading.Thread(
                    target=r2_moveL_xyz,
                    args=(data['x'], data['y'], data['z'], data.get('r', 0)),
                    daemon=True,
                ).start()
            elif action == "Pusher":
                sim_DO2(1, 1 if data.get('status') == "ON" else 0)

        elif topic == TOPIC_BOTH:
            if data.get('action') == "stop":
                is_running = False
                print("[SIM] EMERGENCY STOP — simulation halted!")

    except Exception as e:
        print(f"[MQTT] Error processing message: {e}")

# =============================================================================
# --- START EVERYTHING ---
# =============================================================================
threading.Thread(target=rail1_loop,            daemon=True).start()
threading.Thread(target=robot2_and_rail2_loop, daemon=True).start()

# Robot1 → HOME on startup
print("[Robot1] Moving to HOME...")
robot1.MoveJ(R1_POSITIONS['home'], blocking=True)
publish_joints(1, R1_POSITIONS['home'])
print("[Robot1] At HOME — waiting for MQTT command...")

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, 1883, 60)

print("=" * 50)
print(f"Robot1 topic   : {TOPIC_R1}")
print(f"Robot2 topic   : {TOPIC_R2}")
print(f"Both topic     : {TOPIC_BOTH}")
print(f"R1 joints pub  : {TOPIC_R1_JOINTS}")
print(f"R2 joints pub  : {TOPIC_R2_JOINTS}")
print(f"R1 sensor pub  : {TOPIC_R1_SENSOR}")
print(f"R2 sensor pub  : {TOPIC_R2_SENSOR}")
print("=" * 50)

mqtt_client.loop_forever()
