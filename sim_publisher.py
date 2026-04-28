"""
sim_publisher.py
GUI MQTT Publisher — ส่งคำสั่งควบคุม factory_robot_sim.py

Topics publish:
  kmutt/cpe393/robot1/command
  kmutt/cpe393/robot2/command
  kmutt/cpe393/all/command

Topics subscribe (feedback):
  factory/dobot1/joints   factory/dobot2/joints
  factory/dobot1/irsensor factory/dobot2/irsensor
"""

import json
import queue
import threading
import time
from datetime import datetime

import customtkinter as ctk
import paho.mqtt.client as mqtt

# =============================================================================
# --- CONFIG ---
# =============================================================================
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT   = 1883

TOPIC_R1   = "kmutt/cpe393/robot1/command"
TOPIC_R2   = "kmutt/cpe393/robot2/command"
TOPIC_BOTH = "kmutt/cpe393/all/command"

SUB_R1_JOINTS = "factory/dobot1/joints"
SUB_R2_JOINTS = "factory/dobot2/joints"
SUB_R1_IR     = "factory/dobot1/irsensor"
SUB_R2_IR     = "factory/dobot2/irsensor"

# =============================================================================
# --- COLOUR PALETTE ---
# =============================================================================
C_BG        = "#1a1a2e"
C_CARD      = "#16213e"
C_ACCENT    = "#0f3460"
C_GOOD      = "#2ecc71"
C_BAD       = "#e74c3c"
C_WARN      = "#f39c12"
C_BLUE      = "#3498db"
C_TEXT      = "#ecf0f1"
C_MUTED     = "#7f8c8d"
C_STOP      = "#c0392b"
C_STOP_HV   = "#e74c3c"
C_RAIL_ON   = "#27ae60"
C_RAIL_OFF  = "#2c3e50"

# =============================================================================
# --- MAIN WINDOW ---
# =============================================================================
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class SimPublisherGUI(ctk.CTk):

    def __init__(self):
        super().__init__()
        self.title("Sim Publisher — Robot Commander  |  CPE393")
        self.geometry("1100x760")
        self.minsize(960, 680)
        self.configure(fg_color=C_BG)

        self._msg_q: queue.Queue = queue.Queue()
        self._mqtt: mqtt.Client | None = None
        self._connected = False

        # feedback state
        self._r1_joints = [0.0, 0.0, 0.0, 0.0]
        self._r2_joints = [0.0, 0.0, 0.0, 0.0]
        self._r1_ir = 0
        self._r2_ir = 0

        self._build_ui()
        self._mqtt_connect()
        self.after(150, self._drain_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ═════════════════════════════════════ UI BUILD ══════════════════════════

    def _build_ui(self):
        # ── Top bar ──────────────────────────────────────────────────────────
        topbar = ctk.CTkFrame(self, fg_color=C_ACCENT, corner_radius=0, height=52)
        topbar.pack(fill="x", side="top")
        topbar.pack_propagate(False)

        ctk.CTkLabel(
            topbar,
            text="  🤖  Sim Publisher — Robot Commander",
            font=ctk.CTkFont(size=17, weight="bold"),
            text_color=C_TEXT,
        ).pack(side="left", padx=16, pady=10)

        # MQTT status dot + label
        self._dot = ctk.CTkLabel(topbar, text="●", font=ctk.CTkFont(size=20),
                                  text_color="#e74c3c")
        self._dot.pack(side="right", padx=(0, 6), pady=10)
        self._conn_lbl = ctk.CTkLabel(topbar, text="Disconnected",
                                       font=ctk.CTkFont(size=12), text_color=C_MUTED)
        self._conn_lbl.pack(side="right", padx=(0, 4), pady=10)

        # ── Main body ─────────────────────────────────────────────────────────
        body = ctk.CTkFrame(self, fg_color=C_BG)
        body.pack(fill="both", expand=True, padx=14, pady=(10, 6))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=0)

        # Left column: R1 + R2 cards
        left = ctk.CTkFrame(body, fg_color=C_BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        left.rowconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)

        self._build_robot_card(left, robot_id=1, row=0)
        self._build_robot_card(left, robot_id=2, row=1)

        # Right column: feedback + log
        right = ctk.CTkFrame(body, fg_color=C_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        right.rowconfigure(0, weight=0)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self._build_feedback_card(right)
        self._build_log_card(right)

        # Bottom: E-STOP bar
        self._build_estop_bar(body)

    # ── Robot card ───────────────────────────────────────────────────────────
    def _build_robot_card(self, parent, robot_id: int, row: int):
        color = "#1a3a5c" if robot_id == 1 else "#2d1b4e"
        card = ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=12)
        card.grid(row=row, column=0, sticky="nsew", pady=(0 if row else 0, 6))
        card.columnconfigure(0, weight=1)

        hdr = ctk.CTkFrame(card, fg_color=color, corner_radius=8, height=36)
        hdr.pack(fill="x", padx=8, pady=(8, 4))
        hdr.pack_propagate(False)
        ctk.CTkLabel(
            hdr,
            text=f"  DOBOT {robot_id}  ·  MG400",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=C_TEXT,
        ).pack(side="left", padx=10, pady=6)

        inner = ctk.CTkFrame(card, fg_color="transparent")
        inner.pack(fill="both", expand=True, padx=10, pady=(2, 8))

        if robot_id == 1:
            self._build_r1_controls(inner)
        else:
            self._build_r2_controls(inner)

    def _build_r1_controls(self, parent):
        # Row 0: Process buttons
        row0 = ctk.CTkFrame(parent, fg_color="transparent")
        row0.pack(fill="x", pady=(4, 2))
        ctk.CTkLabel(row0, text="Process:", font=ctk.CTkFont(size=12),
                     text_color=C_MUTED, width=70).pack(side="left")
        ctk.CTkButton(
            row0, text="✔  GOOD", width=130, height=36,
            fg_color=C_GOOD, hover_color="#27ae60", text_color="white",
            font=ctk.CTkFont(size=13, weight="bold"),
            command=lambda: self._send(TOPIC_R1, {"action": "process", "result": "good"}),
        ).pack(side="left", padx=(4, 6))
        ctk.CTkButton(
            row0, text="✘  BAD", width=130, height=36,
            fg_color=C_BAD, hover_color="#c0392b", text_color="white",
            font=ctk.CTkFont(size=13, weight="bold"),
            command=lambda: self._send(TOPIC_R1, {"action": "process", "result": "bad"}),
        ).pack(side="left")

        # Row 1: Pusher
        row1 = ctk.CTkFrame(parent, fg_color="transparent")
        row1.pack(fill="x", pady=2)
        ctk.CTkLabel(row1, text="Pusher:", font=ctk.CTkFont(size=12),
                     text_color=C_MUTED, width=70).pack(side="left")
        ctk.CTkButton(
            row1, text="ON", width=80, height=32,
            fg_color=C_RAIL_ON, hover_color="#219a52",
            command=lambda: self._send(TOPIC_R1, {"action": "Pusher", "status": "ON"}),
        ).pack(side="left", padx=(4, 4))
        ctk.CTkButton(
            row1, text="OFF", width=80, height=32,
            fg_color=C_RAIL_OFF, hover_color="#3d5166",
            command=lambda: self._send(TOPIC_R1, {"action": "Pusher", "status": "OFF"}),
        ).pack(side="left")

        # Row 2: Cartesian move
        self._r1_xyz = self._build_xyzr_row(parent, robot_id=1)

    def _build_r2_controls(self, parent):
        # Row 0: Pusher
        row0 = ctk.CTkFrame(parent, fg_color="transparent")
        row0.pack(fill="x", pady=(4, 2))
        ctk.CTkLabel(row0, text="Pusher:", font=ctk.CTkFont(size=12),
                     text_color=C_MUTED, width=70).pack(side="left")
        ctk.CTkButton(
            row0, text="ON", width=80, height=32,
            fg_color=C_RAIL_ON, hover_color="#219a52",
            command=lambda: self._send(TOPIC_R2, {"action": "Pusher", "status": "ON"}),
        ).pack(side="left", padx=(4, 4))
        ctk.CTkButton(
            row0, text="OFF", width=80, height=32,
            fg_color=C_RAIL_OFF, hover_color="#3d5166",
            command=lambda: self._send(TOPIC_R2, {"action": "Pusher", "status": "OFF"}),
        ).pack(side="left")

        # Row 1: Cartesian move
        self._r2_xyz = self._build_xyzr_row(parent, robot_id=2)

    def _build_xyzr_row(self, parent, robot_id: int):
        """Returns dict of Entry widgets for x, y, z, r."""
        frame = ctk.CTkFrame(parent, fg_color=C_ACCENT, corner_radius=8)
        frame.pack(fill="x", pady=(6, 2), ipady=4)

        ctk.CTkLabel(frame, text="Move (mm/°):", font=ctk.CTkFont(size=11),
                     text_color=C_MUTED).grid(row=0, column=0, padx=(8, 4), pady=4)

        entries = {}
        for col, lbl in enumerate(["x", "y", "z", "r"], start=1):
            ctk.CTkLabel(frame, text=lbl, font=ctk.CTkFont(size=11),
                         text_color=C_TEXT, width=14).grid(row=0, column=col * 2 - 1,
                                                            padx=(6, 0))
            e = ctk.CTkEntry(frame, width=70, height=28,
                             font=ctk.CTkFont(size=12), placeholder_text="0")
            e.grid(row=0, column=col * 2, padx=(2, 0), pady=4)
            entries[lbl] = e

        topic = TOPIC_R1 if robot_id == 1 else TOPIC_R2

        def _send_move():
            try:
                payload = {
                    "action": "move",
                    "x": float(entries["x"].get() or 0),
                    "y": float(entries["y"].get() or 0),
                    "z": float(entries["z"].get() or 0),
                    "r": float(entries["r"].get() or 0),
                }
                self._send(topic, payload)
            except ValueError:
                self._log("⚠  ค่า x/y/z/r ต้องเป็นตัวเลข", color=C_WARN)

        ctk.CTkButton(
            frame, text="Send", width=60, height=28,
            fg_color=C_BLUE, hover_color="#2980b9",
            command=_send_move,
        ).grid(row=0, column=9, padx=(8, 8), pady=4)

        return entries

    # ── Feedback card ─────────────────────────────────────────────────────────
    def _build_feedback_card(self, parent):
        card = ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=12)
        card.grid(row=0, column=0, sticky="ew", pady=(0, 6))

        ctk.CTkLabel(card, text="  Live Feedback",
                     font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=C_TEXT).pack(anchor="w", padx=12, pady=(8, 2))

        grid = ctk.CTkFrame(card, fg_color="transparent")
        grid.pack(fill="x", padx=12, pady=(0, 8))
        grid.columnconfigure((0, 1, 2, 3), weight=1)

        # Joint labels — Robot 1
        ctk.CTkLabel(grid, text="Dobot1 Joints",
                     font=ctk.CTkFont(size=11, weight="bold"),
                     text_color=C_MUTED).grid(row=0, column=0, columnspan=4,
                                               sticky="w", pady=(2, 0))
        self._r1j_lbl = []
        for i in range(4):
            lbl = ctk.CTkLabel(grid, text=f"J{i+1}: —",
                               font=ctk.CTkFont(size=11), text_color=C_TEXT,
                               fg_color=C_ACCENT, corner_radius=6, width=100)
            lbl.grid(row=1, column=i, padx=2, pady=2, sticky="ew")
            self._r1j_lbl.append(lbl)

        # Joint labels — Robot 2
        ctk.CTkLabel(grid, text="Dobot2 Joints",
                     font=ctk.CTkFont(size=11, weight="bold"),
                     text_color=C_MUTED).grid(row=2, column=0, columnspan=4,
                                               sticky="w", pady=(6, 0))
        self._r2j_lbl = []
        for i in range(4):
            lbl = ctk.CTkLabel(grid, text=f"J{i+1}: —",
                               font=ctk.CTkFont(size=11), text_color=C_TEXT,
                               fg_color=C_ACCENT, corner_radius=6, width=100)
            lbl.grid(row=3, column=i, padx=2, pady=2, sticky="ew")
            self._r2j_lbl.append(lbl)

        # IR sensor indicators
        ir_row = ctk.CTkFrame(card, fg_color="transparent")
        ir_row.pack(fill="x", padx=12, pady=(0, 8))

        ctk.CTkLabel(ir_row, text="IR Sensor  Rail1:",
                     font=ctk.CTkFont(size=11), text_color=C_MUTED).pack(side="left")
        self._ir1_lbl = ctk.CTkLabel(ir_row, text="  ○ CLEAR  ",
                                      font=ctk.CTkFont(size=11, weight="bold"),
                                      fg_color=C_RAIL_OFF, corner_radius=8,
                                      text_color="white")
        self._ir1_lbl.pack(side="left", padx=6)

        ctk.CTkLabel(ir_row, text="Rail2:",
                     font=ctk.CTkFont(size=11), text_color=C_MUTED).pack(side="left", padx=(10, 0))
        self._ir2_lbl = ctk.CTkLabel(ir_row, text="  ○ CLEAR  ",
                                      font=ctk.CTkFont(size=11, weight="bold"),
                                      fg_color=C_RAIL_OFF, corner_radius=8,
                                      text_color="white")
        self._ir2_lbl.pack(side="left", padx=6)

    # ── Log card ──────────────────────────────────────────────────────────────
    def _build_log_card(self, parent):
        card = ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=12)
        card.grid(row=1, column=0, sticky="nsew", pady=0)
        card.rowconfigure(1, weight=1)
        card.columnconfigure(0, weight=1)

        hdr = ctk.CTkFrame(card, fg_color="transparent")
        hdr.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 2))
        ctk.CTkLabel(hdr, text="Log",
                     font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=C_TEXT).pack(side="left")
        ctk.CTkButton(hdr, text="Clear", width=56, height=24,
                      fg_color=C_ACCENT, hover_color="#1a5276",
                      font=ctk.CTkFont(size=11),
                      command=self._clear_log).pack(side="right")

        self._log_box = ctk.CTkTextbox(
            card, fg_color="#0d1117", text_color=C_TEXT,
            font=ctk.CTkFont(family="Menlo", size=11),
            corner_radius=8, wrap="none",
        )
        self._log_box.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        self._log_box.configure(state="disabled")

    # ── E-Stop bar ────────────────────────────────────────────────────────────
    def _build_estop_bar(self, parent):
        bar = ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=10, height=58)
        bar.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        bar.pack_propagate(False)

        ctk.CTkButton(
            bar,
            text="⛔  EMERGENCY STOP",
            width=260, height=44,
            fg_color=C_STOP, hover_color=C_STOP_HV,
            font=ctk.CTkFont(size=16, weight="bold"),
            text_color="white",
            corner_radius=8,
            command=self._emergency_stop,
        ).pack(side="left", padx=16, pady=7)

        ctk.CTkLabel(
            bar,
            text="ส่งคำสั่ง stop ไปที่  kmutt/cpe393/all/command\nหยุด simulation ทันที",
            font=ctk.CTkFont(size=11),
            text_color=C_MUTED,
            justify="left",
        ).pack(side="left", padx=8)

    # ═════════════════════════════════════ MQTT ══════════════════════════════

    def _mqtt_connect(self):
        self._mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._mqtt.on_connect    = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect
        self._mqtt.on_message    = self._on_message

        def _connect_bg():
            try:
                self._mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
                self._mqtt.loop_start()
            except Exception as e:
                self._msg_q.put(("log", f"MQTT connect error: {e}", C_BAD))

        threading.Thread(target=_connect_bg, daemon=True).start()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(SUB_R1_JOINTS)
            client.subscribe(SUB_R2_JOINTS)
            client.subscribe(SUB_R1_IR)
            client.subscribe(SUB_R2_IR)
            self._msg_q.put(("connected", True))
            self._msg_q.put(("log", f"Connected to {MQTT_BROKER}", C_GOOD))
        else:
            self._msg_q.put(("log", f"MQTT rc={rc}", C_BAD))

    def _on_disconnect(self, client, userdata, rc, properties=None, reasonCode=None):
        self._msg_q.put(("connected", False))
        self._msg_q.put(("log", "Disconnected from broker", C_WARN))

    def _on_message(self, client, userdata, msg):
        try:
            data  = json.loads(msg.payload.decode())
            topic = msg.topic

            if topic == SUB_R1_JOINTS:
                self._msg_q.put(("r1_joints", [data["j1"], data["j2"],
                                               data["j3"], data["j4"]]))
            elif topic == SUB_R2_JOINTS:
                self._msg_q.put(("r2_joints", [data["j1"], data["j2"],
                                               data["j3"], data["j4"]]))
            elif topic == SUB_R1_IR:
                self._msg_q.put(("r1_ir", data["state"]))
            elif topic == SUB_R2_IR:
                self._msg_q.put(("r2_ir", data["state"]))
        except Exception:
            pass

    def _send(self, topic: str, payload: dict):
        if not self._connected:
            self._log("⚠  ยังไม่ได้ต่อ MQTT", color=C_WARN)
            return
        msg = json.dumps(payload)
        self._mqtt.publish(topic, msg)
        ts = datetime.now().strftime("%H:%M:%S")
        short_topic = topic.split("/")[-2] + "/" + topic.split("/")[-1]
        self._log(f"[{ts}] ▶ {short_topic}  {msg}", color=C_BLUE)

    def _emergency_stop(self):
        self._send(TOPIC_BOTH, {"action": "stop"})
        self._log("⛔  EMERGENCY STOP sent!", color=C_BAD)

    # ═════════════════════════════════════ Queue drain ═══════════════════════

    def _drain_queue(self):
        try:
            while True:
                item = self._msg_q.get_nowait()
                kind = item[0]

                if kind == "connected":
                    self._set_connected(item[1])
                elif kind == "log":
                    self._log(item[1], color=item[2] if len(item) > 2 else C_TEXT)
                elif kind == "r1_joints":
                    joints = item[1]
                    for i, v in enumerate(joints):
                        self._r1j_lbl[i].configure(text=f"J{i+1}: {v:.1f}°")
                elif kind == "r2_joints":
                    joints = item[1]
                    for i, v in enumerate(joints):
                        self._r2j_lbl[i].configure(text=f"J{i+1}: {v:.1f}°")
                elif kind == "r1_ir":
                    state = item[1]
                    if state:
                        self._ir1_lbl.configure(text="  ● DETECT  ",
                                                 fg_color=C_WARN)
                    else:
                        self._ir1_lbl.configure(text="  ○ CLEAR  ",
                                                 fg_color=C_RAIL_OFF)
                elif kind == "r2_ir":
                    state = item[1]
                    if state:
                        self._ir2_lbl.configure(text="  ● DETECT  ",
                                                 fg_color=C_WARN)
                    else:
                        self._ir2_lbl.configure(text="  ○ CLEAR  ",
                                                 fg_color=C_RAIL_OFF)
        except queue.Empty:
            pass
        self.after(150, self._drain_queue)

    # ═════════════════════════════════════ Helpers ═══════════════════════════

    def _set_connected(self, ok: bool):
        self._connected = ok
        if ok:
            self._dot.configure(text_color=C_GOOD)
            self._conn_lbl.configure(text="Connected", text_color=C_GOOD)
        else:
            self._dot.configure(text_color=C_BAD)
            self._conn_lbl.configure(text="Disconnected", text_color=C_MUTED)

    def _log(self, text: str, color: str = C_TEXT):
        self._log_box.configure(state="normal")
        self._log_box.insert("end", text + "\n")
        self._log_box.configure(state="disabled")
        self._log_box.see("end")

    def _clear_log(self):
        self._log_box.configure(state="normal")
        self._log_box.delete("1.0", "end")
        self._log_box.configure(state="disabled")

    def _on_close(self):
        if self._mqtt:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        self.destroy()


# =============================================================================
if __name__ == "__main__":
    app = SimPublisherGUI()
    app.mainloop()
