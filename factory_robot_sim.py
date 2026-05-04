"""
factory_robot_sim.py
RoboDK Simulation — mirrors factory_robot.py (real robot control)

Differences from real code:
  - DobotApiDashboard / DobotApiMove replaced by RoboDK MoveJ / MoveL
  - Hardware feedback sockets replaced by simulated DI flags
  - dash.DO() replaced by sim_DO() (prints + updates sim state)
  - Rail travel simulated with a 1.5-second delay
  - Robot2 picking clears DI10 of Rail1 automatically
  - pause_event / emergency_shutdown mirror real pause/resume/stop logic
  - r1_pick_override supports Cartesian pick-position override via MQTT
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

TOPIC_R1_JOINTS  = "factory/dobot1/joints"
TOPIC_R2_JOINTS  = "factory/dobot2/joints"
TOPIC_R1_SENSOR  = "factory/dobot1/irsensor"
TOPIC_R2_SENSOR  = "factory/dobot2/irsensor"
TOPIC_SYS_STATUS = "factory/system/status"
TOPIC_2DOF_JOINTS = "factory/2dof/joints"

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
# \u2500\u2500 2DOF arm (1 active joint) \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
robot_2dof = RDK.Item('2DOF-ROBOT', ITEM_TYPE_ROBOT)
if not robot_2dof.Valid():
    robot_2dof = RDK.Item('2DOF_ROBOT', ITEM_TYPE_ROBOT)
if robot_2dof.Valid():
    print("RoboDK: 2DOF-ROBOT found.")
    robot_2dof.setSpeed(50)
else:
    print("[WARN] 2DOF-ROBOT not found in RoboDK station \u2014 2DOF motion disabled.")
    robot_2dof = None
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
R1_CHECK1    = [45.00,   3.36,   1.79, 207.48, 0, 0]   # sim: same area as pick

R1_POSITIONS = {
    'home':      R1_HOME,
    'wait':      R1_WAIT,
    'pick':      R1_PICK,
    'good':      R1_GOOD,
    'ipbad':     R1_IPBAD,
    'bad_joint': R1_BAD_JOINT,
    'bad_back':  R1_BAD_BACK,
    'check1':    R1_CHECK1,
}

# =============================================================================
# --- CHECKPOINTS: ROBOT #2 ---
# =============================================================================
R2_HOME    = [ 58.93,  24.90,  37.43, -175.09, 0, 0]
R2_WAIT    = [135.65,  24.90,  22.52, -175.09, 0, 0]
R2_GOOD    = [136.22,  40.56,  43.40, -175.09, 0, 0]
R2_OFFSET  = [ 90.00,  24.90,  22.52, -175.09, 0, 0]
R2_PICK    = [ 92.00,  54.06,  72.68, -175.09, 0, 0]
R2_TOPLACE = [ 92.00,  24.90, 120.00, -175.09, 0, 0]
R2_PLACE   = [ 18.39,  37.76,  26.03, -175.09, 0, 0]

R2_POSITIONS = {
    'home':    R2_HOME,
    'wait':    R2_WAIT,
    'good':    R2_GOOD,
    'offset':  R2_OFFSET,
    'pick':    R2_PICK,
    'toplace': R2_TOPLACE,
    'place':   R2_PLACE,
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
# --- 2DOF ARM SEQUENCE ---
# Triggered each time the Rail1 IR sensor detects a product.
# Rotates the single joint: 0\u00b0 \u2192 -45\u00b0 \u2192 +45\u00b0 \u2192 0\u00b0
# =============================================================================
def run_2dof_sequence():
    """Sweep the 2DOF arm \u221245\u00b0 \u2192 +45\u00b0 \u2192 0\u00b0 (1-joint robot)."""
    if robot_2dof is None or not robot_2dof.Valid():
        print("[2DOF] Robot not available \u2014 skipping sequence.")
        return
    try:
        print("[2DOF] Rotating to -45\u00b0")
        robot_2dof.MoveJ([-45], blocking=True)
        print("[2DOF] Rotating to +45\u00b0")
        robot_2dof.MoveJ([45], blocking=True)
        print("[2DOF] Returning to 0\u00b0")
        robot_2dof.MoveJ([0], blocking=True)
        print("[2DOF] Sequence complete.")
    except Exception as e:
        print(f"[2DOF] Error during sequence: {e}")
# =============================================================================
# --- GLOBAL STATE ---
# =============================================================================
is_running      = True
rail1_count     = 0
robot2_can_get  = False
r1_busy         = False
product_lock    = threading.Lock()
mqtt_client     = None   # set after MQTT client is created below

pause_event = threading.Event()
pause_event.set()   # start in RUNNING state

r1_pick_lock     = threading.Lock()
r1_pick_override = None

# Real IR sensor state for Rail1 — updated via MQTT subscription
_rail1_ir_state = 0           # 0 = clear, 1 = product at end
_rail1_ir_event = threading.Event()   # pulsed whenever IR state changes

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

def publish_system_status(status: str):
    """Publish overall system status (running / paused / stopped)."""
    if mqtt_client is None:
        return
    payload = json.dumps({"status": status})
    mqtt_client.publish(TOPIC_SYS_STATUS, payload)
    print(f"[MQTT] System status \u2192 {payload}")
# =============================================================================
# --- MOVE HELPERS ---
# =============================================================================
def r1_move(name: str):
    """MoveJ Robot1 to named checkpoint, then publish real joint angles."""
    pos = R1_POSITIONS[name]
    print(f"[Robot1] MoveJ \u2192 {name}: {pos[:4]}")
    robot1.MoveJ(pos, blocking=True)
    publish_joints(1, robot1.Joints().list())


def r1_move_pick():
    """
    Move Robot1 to the pick position — uses Cartesian MoveL override if set
    via MQTT, otherwise falls back to the default 'pick' joint checkpoint.
    """
    with r1_pick_lock:
        pos = r1_pick_override
    if pos is not None:
        x, y, z, r = pos
        print(f"[Robot1] MoveL (override) \u2192 x={x} y={y} z={z} r={r}")
        try:
            # Use FK at default pick joints to get the correct tool orientation,
            # then override only the Cartesian position.  Using transl()*rotz()
            # alone gives identity orientation which Dobot IK cannot solve.
            pose = robot1.SolveFK(R1_POSITIONS['pick'])
            pose[0, 3] = x
            pose[1, 3] = y
            pose[2, 3] = z
            if r != 0:
                pose = pose * rotz(r)
            robot1.MoveL(pose, blocking=True)
        except Exception as e:
            print(f"[Robot1] MoveL failed ({e}) \u2014 falling back to joint pick")
            robot1.MoveJ(R1_POSITIONS['pick'], blocking=True)
    else:
        jpos = R1_POSITIONS['pick']
        print(f"[Robot1] MoveJ \u2192 pick: {jpos[:4]}")
        robot1.MoveJ(jpos, blocking=True)
    publish_joints(1, robot1.Joints().list())


def r2_move(name: str):
    """MoveJ Robot2 to named checkpoint, then publish real joint angles."""
    pos = R2_POSITIONS[name]
    print(f"[Robot2] MoveJ \u2192 {name}: {pos[:4]}")
    robot2.MoveJ(pos, blocking=True)
    publish_joints(2, robot2.Joints().list())


def r1_moveL_xyz(x, y, z, r):
    """Cartesian linear move for Robot1 (MQTT 'move' action)."""
    print(f"[Robot1] MoveL → x={x} y={y} z={z} r={r}")
    try:
        pose = robot1.SolveFK(R1_POSITIONS['home'])
        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z
        if r != 0:
            pose = pose * rotz(r)
        robot1.MoveL(pose, blocking=True)
    except Exception as e:
        print(f"[Robot1] MoveL failed ({e})")
    publish_joints(1, robot1.Joints().list())


def r2_moveL_xyz(x, y, z, r):
    """Cartesian linear move for Robot2 (MQTT 'move' action)."""
    print(f"[Robot2] MoveL → x={x} y={y} z={z} r={r}")
    try:
        pose = robot2.SolveFK(R2_POSITIONS['home'])
        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z
        if r != 0:
            pose = pose * rotz(r)
        robot2.MoveL(pose, blocking=True)
    except Exception as e:
        print(f"[Robot2] MoveL failed ({e})")
    publish_joints(2, robot2.Joints().list())

# =============================================================================
# --- EMERGENCY SHUTDOWN ---
# =============================================================================
def emergency_shutdown():
    global is_running

    print("=" * 50)
    print("[SHUTDOWN] Emergency stop received. Shutting down simulation...")

    is_running = False
    pause_event.set()   # unblock any waiting threads

    print("[SHUTDOWN] Turning off all simulated outputs...")
    sim_DO1(2, 0);  sim_DO1(9, 0);  sim_DO1(10, 0);  sim_DO1(1, 0)
    sim_DO2(2, 0);  sim_DO2(9, 0);  sim_DO2(10, 0);  sim_DO2(1, 0)

    print("[SHUTDOWN] Publishing final status...")
    try:
        publish_system_status("stopped")
    except Exception:
        pass

    import time as _t
    _t.sleep(0.5)

    print("[SHUTDOWN] Disconnecting MQTT...")
    try:
        mqtt_client.disconnect()
        mqtt_client.loop_stop()
    except Exception:
        pass

    print("[SHUTDOWN] Simulation stopped.")
    print("=" * 50)
    import os
    os._exit(0)


# =============================================================================
# --- PAUSE / RESUME ---
# =============================================================================
def do_pause():
    """Block all loops and stop both rails."""
    if not pause_event.is_set():
        print("[System] Already paused.")
        return

    pause_event.clear()

    sim_DO1(2, 0)   # stop Rail1
    sim_DO2(2, 0)   # stop Rail2

    publish_system_status("paused")
    print("[System] PAUSED \u2014 rails stopped, loops blocked.")


def do_resume():
    """Unblock all loops."""
    if pause_event.is_set():
        print("[System] Already running.")
        return

    pause_event.set()

    publish_system_status("running")
    print("[System] RESUMED \u2014 loops unblocked.")


# =============================================================================
# --- ROBOT 1 SEQUENCE (MQTT triggered) ---
# =============================================================================
def run_robot1_sequence(is_good):
    global r1_busy, rail1_count

    if not pause_event.is_set():
        print("[Robot1] System paused — ignoring new sequence request.")
        return

    if r1_busy:
        print("[Robot1] Busy, ignoring.")
        return

    if is_good:
        with product_lock:
            count = rail1_count
        if count >= 2:
            print(f"[Robot1] GOOD blocked — Rail1 full ({count}/2 products).")
            return
        if _rail1_ir_state == 1:
            print("[Robot1] GOOD blocked — product still at end of Rail1 (IR=1).")
            return

    r1_busy = True
    try:
        print(f"[Robot1] Sequence: {'GOOD' if is_good else 'BAD'}")

        pause_event.wait()
        if not is_running: return
        r1_move('home')

        pause_event.wait()
        if not is_running: return
        r1_move('wait')

        pause_event.wait()
        if not is_running: return
        r1_move_pick()

        pause_event.wait()
        if not is_running: return
        time.sleep(1)
        sim_DO1(9, 1)
        time.sleep(1)
        sim_DO1(9, 0)

        if is_good:
            pause_event.wait()
            if not is_running: return
            r1_move('good')
        else:
            pause_event.wait()
            if not is_running: return
            r1_move('ipbad')
            pause_event.wait()
            if not is_running: return
            r1_move('bad_joint')

        pause_event.wait()
        if not is_running: return
        time.sleep(1)
        sim_DO1(10, 1)
        time.sleep(1)
        sim_DO1(10, 0)

        if is_good:
            with product_lock:
                rail1_count += 1
            print(f"[Robot1] Released GOOD product. Rail1 count: {rail1_count}")
        else:
            pause_event.wait()
            if not is_running: return
            r1_move('bad_back')

        pause_event.wait()
        if not is_running: return
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

    while is_running:
        pause_event.wait()
        if not is_running:
            break

        with product_lock:
            count = rail1_count

        if count > 0:
            print(f"[Rail1] Product on belt ({count}). Running rail...")
            sim_DO1(2, 1)   # Rail 1 ON

            # Wait for REAL IR sensor = 1 from MQTT (product reaches end)
            print("[Rail1] Waiting for IR sensor = 1 from MQTT...")
            _rail1_ir_event.clear()
            while is_running and _rail1_ir_state != 1:
                pause_event.wait()
                _rail1_ir_event.wait(timeout=0.1)
                _rail1_ir_event.clear()

            if not is_running:
                break

            print("[Rail1] IR = 1 detected. Stopping rail.")
            sim_DO1(2, 0)   # Rail 1 OFF

            # 2DOF arm sweeps when IR = 1
            run_2dof_sequence()

            # Robot2 will start when it receives {"action":"trigger"} via MQTT
            print("[Rail1] 2DOF done. Waiting for external MQTT trigger to start Robot2...")

            # Wait until real IR sensor = 0 (product picked by Robot2)
            print("[Rail1] Waiting for IR sensor = 0 (product taken)...")
            _rail1_ir_event.clear()
            while is_running and _rail1_ir_state != 0:
                pause_event.wait()
                _rail1_ir_event.wait(timeout=0.1)
                _rail1_ir_event.clear()

            if not is_running:
                break

            print("[Rail1] Product taken. Rail1 ready for next.")
            with product_lock:
                rail1_count -= 1
            print(f"[Rail1] Rail1 count: {rail1_count}")

        time.sleep(0.1)

    print("[Rail1] Loop exited.")

# =============================================================================
# --- ROBOT 2 + RAIL 2 LOOP (autonomous) ---
# Publishes IR sensor state changes on factory/dobot2/irsensor
# =============================================================================
def robot2_and_rail2_loop():
    global robot2_can_get

    print("[Robot2] Moving to HOME...")
    r2_move('home')
    print("[Robot2] At HOME — waiting...")

    while is_running:
        pause_event.wait()
        if not is_running:
            break

        if robot2_can_get:
            print("[Robot2] Signal received! Starting pick sequence...")
            try:

                pause_event.wait()
                if not is_running: return
                r2_move('home')

                pause_event.wait()
                if not is_running: return
                r2_move('pick')

                pause_event.wait()
                if not is_running: return
                time.sleep(1); sim_DO2(9, 1); time.sleep(1); sim_DO2(9, 0)

                pause_event.wait()
                if not is_running: return
                r2_move('home')

                pause_event.wait()
                if not is_running: return
                r2_move('place')

                pause_event.wait()
                if not is_running: return
                time.sleep(1); sim_DO2(10, 1); time.sleep(1); sim_DO2(10, 0)

                print("[Robot2] Product placed on Rail2. Starting Rail2...")
                sim_DO2(2, 1)   # Rail 2 ON

                # Simulate belt travel
                _set_di2(10, 0)
                time.sleep(1.5)
                _set_di2(10, 1)

                publish_ir_sensor(2, 1)
                print("[Robot2] Product reached end of Rail2. Stopping Rail2.")
                sim_DO2(2, 0)   # Rail 2 OFF
                publish_ir_sensor(2, 0)

                pause_event.wait()
                if not is_running: return
                r2_move('home')
                print("[Robot2] Back at HOME.")

            except Exception as e:
                print(f"[Robot2] Error: {e}")
            finally:
                robot2_can_get = False
                print("[Robot2] robot2_can_get = False. Waiting...")

        time.sleep(0.1)

    print("[Robot2] Loop exited.")

# =============================================================================
# --- MQTT ---
# =============================================================================
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"MQTT Connected! rc={rc}")
    client.subscribe(TOPIC_R1)
    client.subscribe(TOPIC_R2)
    client.subscribe(TOPIC_BOTH)
    client.subscribe(TOPIC_R1_SENSOR)   # read real IR sensor from hardware


def on_message(client, userdata, msg):
    global r1_pick_override, _rail1_ir_state, robot2_can_get
    try:
        data  = json.loads(msg.payload.decode())
        topic = msg.topic
        print(f"[MQTT] {topic} → {data}")

        if topic == TOPIC_R1_SENSOR:
            _rail1_ir_state = int(data.get('state', 0))
            _rail1_ir_event.set()   # wake up rail1_loop
            print(f"[IR] Rail1 sensor = {_rail1_ir_state}")
            return

        if topic == TOPIC_R1:
            action = data.get('action')

            if action == "process":
                # Optional pick-position override (Cartesian x,y,z,r)
                if all(k in data for k in ('x', 'y', 'z', 'r')):
                    with r1_pick_lock:
                        r1_pick_override = [data['x'], data['y'], data['z'], data['r']]
                        print(f"[MQTT] Pick override set to {r1_pick_override}")
                else:
                    with r1_pick_lock:
                        r1_pick_override = None
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

            elif action == "pick":
                # Sent by rail1_loop after the 2DOF arm completes its sequence
                robot2_can_get = True
                print("[Robot2] Pick signal received via MQTT.")

            elif action == "trigger":
                # Manual trigger: {"action": "trigger"} on kmutt/cpe393/robot2/command
                robot2_can_get = True
                print("[Robot2] Trigger received — starting pick sequence.")

            elif action == "Pusher":
                sim_DO2(1, 1 if data.get('status') == "ON" else 0)

        elif topic == TOPIC_BOTH:
            action = data.get('action')

            if action == "stop":
                threading.Thread(target=emergency_shutdown, daemon=True).start()

            elif action == "pause":
                threading.Thread(target=do_pause, daemon=True).start()

            elif action == "resume":
                threading.Thread(target=do_resume, daemon=True).start()

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
publish_joints(1, robot1.Joints().list())
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
print(f"2DOF joints pub: {TOPIC_2DOF_JOINTS}")
print(f"Sys status pub : {TOPIC_SYS_STATUS}")
print("=" * 50)
print("Commands (publish to kmutt/cpe393/all/command):")
print('  Stop   → {"action": "stop"}')
print('  Pause  → {"action": "pause"}')
print('  Resume → {"action": "resume"}')
print("=" * 50)
print("Robot1 process commands (publish to kmutt/cpe393/robot1/command):")
print('  Good (default pick) → {"action": "process", "result": "good"}')
print('  Bad  (default pick) → {"action": "process", "result": "bad"}')
print('  Good (custom pick)  → {"action": "process", "result": "good", "x": 240, "y": 260, "z": -30, "r": 360}')
print('  Bad  (custom pick)  → {"action": "process", "result": "bad",  "x": 240, "y": 260, "z": -30, "r": 360}')
print("=" * 50)

mqtt_client.loop_forever()
