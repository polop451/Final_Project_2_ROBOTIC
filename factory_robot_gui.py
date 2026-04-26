"""
Factory Robot Control & Monitoring GUI
CPE393 Special Topic III — Class Project 2: Robotics for Productivity

MQTT Topics
-----------
Subscribe:
  factory/status               — device status, counts, speeds, sensor, runtime
  factory/vision/workpieces    — workpiece positions (x,y,z,R) and quality
Publish:
  factory/cmd                  — commands (START / STOP / EMERGENCY_STOP / SET_JOINTS)

All published payloads are optionally Fernet-encrypted.
Incoming payloads are automatically tried for decryption first.

Expected status payload keys
-----------------------------
  arm1_status / arm2_status / belt1_status / belt2_status / ejector_status
    → "ready" | "moving" | "running" | "stopped" | "idle" | "error"
  counts  → {"total_in": N, "good_out": N, "bad_out": N, "total_out": N}
  conveyor1_speed_mps, conveyor2_speed_mps (or conveyor_speed_mps)
  intrusion_count
  safety_sensor_status  → "normal" | "detecting" | "damaged" | "unknown"
  arm1_runtime / ejector_runtime  → {"moving": s, "idle": s, "error": s}
  joint_deg  → [q1, q2, q3, q4]  (degrees, for display)
  conveyor2_entry → {"workpiece_id": N, "quality": "good"|"bad", "pickup_time": ISO}
  conveyor2_exit  → {"workpiece_id": N, "exit_time": ISO,
                      "temperature_c": F, "humidity_pct": F}

Expected vision payload keys
------------------------------
  workpieces  → [{"id": N, "x": mm, "y": mm, "z": mm, "r": deg,
                   "quality": "good"|"bad"}, ...]
  pick_order  → [id, id, ...]   (optimal sequence)
"""

import json
import os
import queue
import sqlite3
import subprocess
import sys
from datetime import datetime

import customtkinter as ctk
import paho.mqtt.client as mqtt
from cryptography.fernet import Fernet, InvalidToken
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ── Colour palette ────────────────────────────────────────────────────────────
_STATUS_COLORS = {
    "ready":     ("#2E7D32", "white"),
    "moving":    ("#1565C0", "white"),
    "running":   ("#1565C0", "white"),
    "stopped":   ("#455A64", "white"),
    "idle":      ("#455A64", "white"),
    "error":     ("#B71C1C", "white"),
    "fault":     ("#B71C1C", "white"),
    "normal":    ("#2E7D32", "white"),
    "detecting": ("#E65100", "white"),
    "damaged":   ("#B71C1C", "white"),
    "unknown":   ("#546E7A", "white"),
    "warning":   ("#F57F17", "black"),
}

_QUALITY_COLORS = {
    "good":    ("#43A047", "white"),
    "bad":     ("#E53935", "white"),
    "unknown": ("#607D8B", "white"),
}

# MG400 joint limits (deg) and human-readable labels
# Joint limits extracted from Dobot-MG400.robot model file
_MG400_LIMITS = [(-160, 160), (-25, 85), (-25, 105), (-360, 360)]
_MG400_LABELS = [
    "J1  Base Rotation",
    "J2  Upper Arm",
    "J3  Forearm",
    "J4  Tool Roll",
]

_EJ_LIMITS = [(-90, 90), (-30, 60)]
_EJ_LABELS = [
    "J1  H-Rotation",
    "J2  Pitch",
]


# ─────────────────────────────────────────────────────────────────────────────
class FactoryRobotGUI(ctk.CTk):

    def __init__(self):
        super().__init__()
        self.title("Factory Robot Control & Monitoring  —  CPE393")
        self.geometry("1440x900")
        self.minsize(1280, 800)

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # ── Runtime state ──────────────────────────────────────────────────
        self._mqtt_client = None
        self._connected = False
        self._msg_queue: queue.Queue = queue.Queue()
        self._fernet_enabled = ctk.BooleanVar(value=True)

        self._counts = {"total_in": 0, "good_out": 0, "bad_out": 0, "total_out": 0}
        self._intrusion_count = 0
        self._workpieces: list = []
        self._pick_order: list = []

        self._arm1_runtime    = {"moving": 0.0, "idle": 0.0, "error": 0.0}
        self._ejector_runtime = {"moving": 0.0, "idle": 0.0, "error": 0.0}

        self._dev_status = {k: "unknown"
                            for k in ("arm1", "arm2", "belt1", "belt2", "ejector")}

        # ── Database ───────────────────────────────────────────────────────
        self._db = sqlite3.connect("factory_data.db", check_same_thread=False)
        self._init_db()

        # ── Build UI ───────────────────────────────────────────────────────
        self._build_ui()
        self.after(100, self._process_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ═══════════════════════════════════════════ Database ═══════════════════

    def _init_db(self):
        self._db.executescript("""
            CREATE TABLE IF NOT EXISTS workpiece_log (
                id            INTEGER PRIMARY KEY AUTOINCREMENT,
                wp_id         INTEGER,
                quality       TEXT,
                pickup_time   TEXT,
                exit_time     TEXT,
                temp_c        REAL,
                humidity_pct  REAL
            );
            CREATE TABLE IF NOT EXISTS system_events (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                event     TEXT NOT NULL
            );
        """)
        self._db.commit()

    def _db_event(self, event: str):
        self._db.execute(
            "INSERT INTO system_events (timestamp, event) VALUES (?, ?)",
            (datetime.now().isoformat(timespec="seconds"), event),
        )
        self._db.commit()

    def _db_log_workpiece(self, wp_id, quality, pickup_time,
                           exit_time=None, temp_c=None, humidity_pct=None):
        self._db.execute(
            """INSERT INTO workpiece_log
               (wp_id, quality, pickup_time, exit_time, temp_c, humidity_pct)
               VALUES (?, ?, ?, ?, ?, ?)""",
            (wp_id, quality, pickup_time, exit_time, temp_c, humidity_pct),
        )
        self._db.commit()
        self.after(0, self._refresh_log_table)

    def _db_update_exit(self, wp_id, exit_time, temp_c, humidity_pct):
        self._db.execute(
            """UPDATE workpiece_log SET exit_time=?, temp_c=?, humidity_pct=?
               WHERE id = (
                   SELECT id FROM workpiece_log
                   WHERE wp_id=? AND exit_time IS NULL
                   ORDER BY id DESC LIMIT 1
               )""",
            (exit_time, temp_c, humidity_pct, wp_id),
        )
        self._db.commit()
        self.after(0, self._refresh_log_table)

    def _load_log_rows(self):
        cur = self._db.execute(
            """SELECT id, wp_id, quality, pickup_time, exit_time, temp_c, humidity_pct
               FROM workpiece_log ORDER BY id DESC LIMIT 50"""
        )
        return cur.fetchall()

    # ═══════════════════════════════════════════ UI Build ═══════════════════

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self._build_connection_bar()
        self._build_encryption_bar()
        self._build_tabs()

    # ── Connection bar ────────────────────────────────────────────────────

    def _build_connection_bar(self):
        f = ctk.CTkFrame(self, corner_radius=8)
        f.grid(row=0, column=0, padx=12, pady=(12, 4), sticky="ew")
        f.grid_columnconfigure(9, weight=1)

        def _lbl(text, col):
            ctk.CTkLabel(f, text=text).grid(row=0, column=col, padx=(12, 2), pady=8)

        _lbl("Broker IP", 0)
        self._broker_e = ctk.CTkEntry(f, width=155)
        self._broker_e.insert(0, "127.0.0.1")
        self._broker_e.grid(row=0, column=1, padx=4)

        _lbl("Port", 2)
        self._port_e = ctk.CTkEntry(f, width=65)
        self._port_e.insert(0, "1883")
        self._port_e.grid(row=0, column=3, padx=4)

        _lbl("Status Topic", 4)
        self._sub_status_e = ctk.CTkEntry(f, width=185)
        self._sub_status_e.insert(0, "factory/status")
        self._sub_status_e.grid(row=0, column=5, padx=4)

        _lbl("Vision Topic", 6)
        self._sub_vision_e = ctk.CTkEntry(f, width=215)
        self._sub_vision_e.insert(0, "factory/vision/workpieces")
        self._sub_vision_e.grid(row=0, column=7, padx=4)

        _lbl("Cmd Topic", 8)
        self._pub_e = ctk.CTkEntry(f, width=145)
        self._pub_e.insert(0, "factory/cmd")
        self._pub_e.grid(row=0, column=9, padx=4, sticky="ew")

        self._conn_btn = ctk.CTkButton(f, text="CONNECT", width=110,
                                        command=self._connect_mqtt)
        self._conn_btn.grid(row=0, column=10, padx=8)

        self._conn_lbl = ctk.CTkLabel(f, text="● Disconnected",
                                       text_color="tomato",
                                       font=ctk.CTkFont(size=13, weight="bold"))
        self._conn_lbl.grid(row=0, column=11, padx=8)

    # ── Encryption bar ────────────────────────────────────────────────────

    def _build_encryption_bar(self):
        f = ctk.CTkFrame(self, corner_radius=8)
        f.grid(row=1, column=0, padx=12, pady=4, sticky="ew")
        f.grid_columnconfigure(2, weight=1)

        ctk.CTkCheckBox(f, text="Fernet Encryption",
                         variable=self._fernet_enabled).grid(
            row=0, column=0, padx=12, pady=6
        )
        ctk.CTkLabel(f, text="Key:").grid(row=0, column=1, padx=(12, 4))
        self._key_e = ctk.CTkEntry(f, show="•")
        self._key_e.insert(0, Fernet.generate_key().decode())
        self._key_e.grid(row=0, column=2, padx=4, sticky="ew")

        ctk.CTkButton(f, text="GENERATE", width=100,
                       command=self._gen_key).grid(row=0, column=3, padx=4, pady=6)
        ctk.CTkButton(f, text="SHOW/HIDE", width=100,
                       command=self._toggle_key).grid(row=0, column=4, padx=8, pady=6)

    # ── Tab container ─────────────────────────────────────────────────────

    def _build_tabs(self):
        tabs = ctk.CTkTabview(self)
        tabs.grid(row=2, column=0, padx=12, pady=(4, 12), sticky="nsew")
        for name in ("Control & Status", "Vision & Pick Order",
                     "Analytics", "Database Log"):
            tabs.add(name)
        self._build_tab_control(tabs.tab("Control & Status"))
        self._build_tab_vision(tabs.tab("Vision & Pick Order"))
        self._build_tab_analytics(tabs.tab("Analytics"))
        self._build_tab_db(tabs.tab("Database Log"))

    # ═══════════════════════════════════════ Tab 1: Control & Status ════════

    def _build_tab_control(self, parent):
        parent.grid_columnconfigure((0, 1, 2), weight=1)
        parent.grid_rowconfigure(0, weight=1)

        left   = ctk.CTkFrame(parent, corner_radius=8)
        center = ctk.CTkFrame(parent, corner_radius=8)
        right  = ctk.CTkFrame(parent, corner_radius=8)
        left.grid  (row=0, column=0, padx=(0, 5), pady=8, sticky="nsew")
        center.grid(row=0, column=1, padx=5,      pady=8, sticky="nsew")
        right.grid (row=0, column=2, padx=(5, 0), pady=8, sticky="nsew")
        for f in (left, center, right):
            f.grid_columnconfigure(0, weight=1)

        self._build_ctrl_left(left)
        self._build_ctrl_center(center)
        self._build_ctrl_right(right)

    def _build_ctrl_left(self, p):
        ctk.CTkLabel(p, text="System Control",
                      font=ctk.CTkFont(size=18, weight="bold")).grid(
            row=0, column=0, padx=14, pady=(14, 6), sticky="w"
        )

        self._start_btn = ctk.CTkButton(
            p, text="▶  START",
            fg_color="#2E7D32", hover_color="#1B5E20",
            font=ctk.CTkFont(size=26, weight="bold"), height=80,
            command=self._send_start,
        )
        self._start_btn.grid(row=1, column=0, padx=14, pady=(4, 6), sticky="ew")

        ctk.CTkButton(
            p, text="◼  STOP",
            fg_color="#E65100", hover_color="#BF360C",
            font=ctk.CTkFont(size=20, weight="bold"), height=55,
            command=self._send_stop,
        ).grid(row=2, column=0, padx=14, pady=4, sticky="ew")

        ctk.CTkButton(
            p, text="⚠  EMERGENCY STOP",
            fg_color="#B71C1C", hover_color="#7F0000",
            font=ctk.CTkFont(size=22, weight="bold"), height=75,
            command=self._send_estop,
        ).grid(row=3, column=0, padx=14, pady=(4, 18), sticky="ew")

        ctk.CTkLabel(p, text="Device Status",
                      font=ctk.CTkFont(size=16, weight="bold")).grid(
            row=4, column=0, padx=14, pady=(6, 4), sticky="w"
        )

        self._dev_badges: dict = {}
        devices = [
            ("arm1",    "Arm 1  (Dobot MG400)"),
            ("arm2",    "Arm 2  (Ejector 2-DOF)"),
            ("belt1",   "Conveyor Belt 1"),
            ("belt2",   "Conveyor Belt 2"),
            ("ejector", "Ejector Device"),
        ]
        for ri, (key, label_text) in enumerate(devices, start=5):
            row_f = ctk.CTkFrame(p, fg_color="transparent")
            row_f.grid(row=ri, column=0, padx=14, pady=2, sticky="ew")
            row_f.grid_columnconfigure(0, weight=1)
            ctk.CTkLabel(row_f, text=label_text,
                          font=ctk.CTkFont(size=13)).grid(row=0, column=0, sticky="w")
            badge = ctk.CTkLabel(
                row_f, text="UNKNOWN",
                font=ctk.CTkFont(size=11, weight="bold"),
                fg_color="#546E7A", text_color="white",
                corner_radius=6, width=90,
            )
            badge.grid(row=0, column=1, sticky="e")
            self._dev_badges[key] = badge

        ctk.CTkButton(p, text="🖥  Open Digital Twin",
                       command=self._open_twin).grid(
            row=11, column=0, padx=14, pady=(18, 14), sticky="ew"
        )

    def _build_ctrl_center(self, p):
        ctk.CTkLabel(p, text="Arm Joint Control",
                      font=ctk.CTkFont(size=18, weight="bold")).grid(
            row=0, column=0, padx=14, pady=(14, 8), sticky="w"
        )
        p.grid_columnconfigure(0, weight=1)
        p.grid_rowconfigure(1, weight=1)

        inner = ctk.CTkTabview(p)
        inner.grid(row=1, column=0, padx=8, pady=(0, 8), sticky="nsew")
        inner.add("Dobot 1 (Arm 1)")
        inner.add("Dobot 2 (Arm 2)")
        inner.add("Ejector (2-DOF)")

        self._build_arm_sliders(
            inner.tab("Dobot 1 (Arm 1)"), "arm1", _MG400_LIMITS, _MG400_LABELS
        )
        self._build_arm_sliders(
            inner.tab("Dobot 2 (Arm 2)"), "arm2", _MG400_LIMITS, _MG400_LABELS
        )
        self._build_arm_sliders(
            inner.tab("Ejector (2-DOF)"), "ejector", _EJ_LIMITS, _EJ_LABELS
        )

    def _build_arm_sliders(self, parent, arm_id, limits, labels):
        parent.grid_columnconfigure(0, weight=1)

        var_list: list = []
        lbl_list: list = []

        for i, ((lo, hi), lbl_text) in enumerate(zip(limits, labels)):
            base_row = i * 3
            ctk.CTkLabel(parent, text=lbl_text,
                          font=ctk.CTkFont(size=13, weight="bold")).grid(
                row=base_row, column=0, padx=16, pady=(10, 0), sticky="w"
            )
            var = ctk.DoubleVar(value=0.0)
            var_list.append(var)
            ctk.CTkSlider(
                parent, from_=lo, to=hi, variable=var,
                command=lambda v, idx=i, aid=arm_id: self._on_joint_drag(aid, idx, v)
            ).grid(row=base_row + 1, column=0, padx=16, pady=3, sticky="ew")

            val_lbl = ctk.CTkLabel(
                parent,
                text=f"  range {lo}° … +{hi}°  →  0.0°",
                font=ctk.CTkFont(size=12),
                text_color="#90A4AE",
            )
            val_lbl.grid(row=base_row + 2, column=0, padx=16, pady=(0, 4), sticky="w")
            lbl_list.append(val_lbl)

        setattr(self, f"_joint_vars_{arm_id}", var_list)
        setattr(self, f"_joint_val_labels_{arm_id}", lbl_list)

        rep_row = len(limits) * 3
        ctk.CTkLabel(parent, text="Reported by robot:",
                      font=ctk.CTkFont(size=12)).grid(
            row=rep_row, column=0, padx=16, pady=(12, 0), sticky="w"
        )
        rep_lbl = ctk.CTkLabel(
            parent,
            text="  ".join(f"J{i + 1}=0.0°" for i in range(len(limits))),
            font=ctk.CTkFont(size=12),
            text_color="#42A5F5",
        )
        rep_lbl.grid(row=rep_row + 1, column=0, padx=16, pady=(0, 8), sticky="w")
        setattr(self, f"_reported_lbl_{arm_id}", rep_lbl)

        btn_f = ctk.CTkFrame(parent, fg_color="transparent")
        btn_f.grid(row=rep_row + 2, column=0, padx=16, pady=(8, 14), sticky="ew")
        btn_f.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkButton(
            btn_f, text="SEND JOINTS",
            command=lambda aid=arm_id: self._send_joints(aid),
        ).grid(row=0, column=0, padx=(0, 4), sticky="ew")
        ctk.CTkButton(
            btn_f, text="RESET TO ZERO",
            fg_color="#546E7A", hover_color="#37474F",
            command=lambda aid=arm_id: self._reset_joints(aid),
        ).grid(row=0, column=1, padx=(4, 0), sticky="ew")

    def _build_ctrl_right(self, p):
        ctk.CTkLabel(p, text="Production Dashboard",
                      font=ctk.CTkFont(size=18, weight="bold")).grid(
            row=0, column=0, columnspan=2, padx=14, pady=(14, 8), sticky="w"
        )
        p.grid_columnconfigure(1, weight=1)

        METRICS = [
            ("total_in",  "Total Entered",   "#42A5F5"),
            ("good_out",  "Good Exited",      "#66BB6A"),
            ("bad_out",   "Rejected (Bad)",   "#EF5350"),
            ("total_out", "Total Exited",     "#90A4AE"),
        ]
        self._count_labels: dict = {}
        for ri, (key, text, color) in enumerate(METRICS, start=1):
            ctk.CTkLabel(p, text=text, font=ctk.CTkFont(size=14)).grid(
                row=ri, column=0, padx=14, pady=5, sticky="w"
            )
            val_lbl = ctk.CTkLabel(p, text="0",
                                    font=ctk.CTkFont(size=30, weight="bold"),
                                    text_color=color)
            val_lbl.grid(row=ri, column=1, padx=14, pady=5, sticky="e")
            self._count_labels[key] = val_lbl

        def _speed_row(label_text, row):
            ctk.CTkLabel(p, text=label_text, font=ctk.CTkFont(size=14)).grid(
                row=row, column=0, padx=14, pady=(12, 4), sticky="w"
            )
            lbl = ctk.CTkLabel(p, text="0.000 m/s",
                                font=ctk.CTkFont(size=20, weight="bold"),
                                text_color="#42A5F5")
            lbl.grid(row=row, column=1, padx=14, pady=(12, 4), sticky="e")
            return lbl

        self._speed1_lbl = _speed_row("Belt 1 Speed", 5)
        self._speed2_lbl = _speed_row("Belt 2 Speed", 6)

        ctk.CTkLabel(p, text="Safety Intrusions",
                      font=ctk.CTkFont(size=14)).grid(
            row=7, column=0, padx=14, pady=(12, 4), sticky="w"
        )
        self._intrusion_lbl = ctk.CTkLabel(p, text="0",
                                            font=ctk.CTkFont(size=30, weight="bold"),
                                            text_color="#FF9800")
        self._intrusion_lbl.grid(row=7, column=1, padx=14, pady=(12, 4), sticky="e")

        ctk.CTkLabel(p, text="Safety Sensor",
                      font=ctk.CTkFont(size=14, weight="bold")).grid(
            row=8, column=0, padx=14, pady=(14, 4), sticky="w"
        )
        self._sensor_badge = ctk.CTkLabel(
            p, text="UNKNOWN",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color="#546E7A", text_color="white",
            corner_radius=8, width=130,
        )
        self._sensor_badge.grid(row=8, column=1, padx=14, pady=(14, 4), sticky="e")

        ctk.CTkLabel(
            p,
            text="Normal = OK  |  Detecting = Triggered  |  Damaged = Fault",
            font=ctk.CTkFont(size=11), text_color="#607D8B",
        ).grid(row=9, column=0, columnspan=2, padx=14, pady=(4, 14), sticky="w")

    # ═══════════════════════════════════ Tab 2: Vision & Pick Order ══════════

    def _build_tab_vision(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(2, weight=1)

        hdr = ctk.CTkFrame(parent, corner_radius=8)
        hdr.grid(row=0, column=0, padx=8, pady=(8, 4), sticky="ew")
        hdr.grid_columnconfigure(2, weight=1)

        self._detected_lbl = ctk.CTkLabel(
            hdr, text="Detected Workpieces: 0",
            font=ctk.CTkFont(size=18, weight="bold"),
        )
        self._detected_lbl.grid(row=0, column=0, padx=16, pady=10, sticky="w")

        ctk.CTkLabel(hdr, text="Optimal Pick Order:",
                      font=ctk.CTkFont(size=14)).grid(
            row=0, column=1, padx=(24, 6), pady=10, sticky="w"
        )
        self._pick_order_lbl = ctk.CTkLabel(
            hdr, text="—",
            font=ctk.CTkFont(size=15, weight="bold"), text_color="#42A5F5",
        )
        self._pick_order_lbl.grid(row=0, column=2, padx=6, pady=10, sticky="w")

        COL_HDRS = ["ID", "X (mm)", "Y (mm)", "Z (mm)", "R (°)",
                    "Quality", "Pick Priority"]
        chdr = ctk.CTkFrame(parent, fg_color="#1A1A2E", corner_radius=6)
        chdr.grid(row=1, column=0, padx=8, pady=(4, 2), sticky="ew")
        for ci, txt in enumerate(COL_HDRS):
            ctk.CTkLabel(chdr, text=txt,
                          font=ctk.CTkFont(size=13, weight="bold"),
                          text_color="#90A4AE").grid(
                row=0, column=ci, padx=14, pady=6, sticky="w"
            )
        chdr.grid_columnconfigure(6, weight=1)

        self._wp_scroll = ctk.CTkScrollableFrame(parent, corner_radius=8)
        self._wp_scroll.grid(row=2, column=0, padx=8, pady=(2, 8), sticky="nsew")
        self._wp_scroll.grid_columnconfigure(0, weight=1)

    def _refresh_workpiece_rows(self):
        for w in self._wp_scroll.winfo_children():
            w.destroy()
        for ri, wp in enumerate(self._workpieces):
            wp_id   = wp.get("id", "?")
            quality = str(wp.get("quality", "unknown")).lower()
            try:
                priority = self._pick_order.index(wp_id) + 1
            except ValueError:
                priority = "—"
            fg, tc = _QUALITY_COLORS.get(quality, ("#607D8B", "white"))
            bg = "#1E1E3A" if ri % 2 == 0 else "#16162A"
            row_f = ctk.CTkFrame(self._wp_scroll, fg_color=bg, corner_radius=4)
            row_f.grid(row=ri, column=0, padx=4, pady=2, sticky="ew")
            row_f.grid_columnconfigure(6, weight=1)
            for ci, val in enumerate([
                str(wp_id),
                f"{wp.get('x', 0.0):.1f}",
                f"{wp.get('y', 0.0):.1f}",
                f"{wp.get('z', 0.0):.1f}",
                f"{wp.get('r', 0.0):.1f}",
            ]):
                ctk.CTkLabel(row_f, text=val,
                              font=ctk.CTkFont(size=14)).grid(
                    row=0, column=ci, padx=14, pady=8, sticky="w"
                )
            ctk.CTkLabel(row_f, text=quality.upper(),
                          font=ctk.CTkFont(size=12, weight="bold"),
                          fg_color=fg, text_color=tc,
                          corner_radius=6, width=68).grid(
                row=0, column=5, padx=14, pady=8, sticky="w"
            )
            ctk.CTkLabel(row_f, text=str(priority),
                          font=ctk.CTkFont(size=14)).grid(
                row=0, column=6, padx=14, pady=8, sticky="w"
            )

    # ═══════════════════════════════════════════ Tab 3: Analytics ═══════════

    def _build_tab_analytics(self, parent):
        parent.grid_columnconfigure((0, 1), weight=1)
        parent.grid_rowconfigure(0, weight=1)

        def _make_pie_panel(col, title):
            f = ctk.CTkFrame(parent, corner_radius=8)
            f.grid(row=0, column=col,
                   padx=(8, 4) if col == 0 else (4, 8),
                   pady=8, sticky="nsew")
            f.grid_columnconfigure(0, weight=1)
            f.grid_rowconfigure(1, weight=1)
            ctk.CTkLabel(f, text=title,
                          font=ctk.CTkFont(size=16, weight="bold")).grid(
                row=0, column=0, padx=12, pady=(12, 4)
            )
            fig = Figure(figsize=(5.2, 3.8), dpi=100)
            fig.patch.set_facecolor("#1A1A2E")
            ax = fig.add_subplot(111)
            ax.set_facecolor("#1A1A2E")
            canvas = FigureCanvasTkAgg(fig, master=f)
            canvas.get_tk_widget().grid(row=1, column=0, padx=8,
                                         pady=(0, 12), sticky="nsew")
            return fig, ax, canvas

        self._fig_arm1, self._ax_arm1, self._canvas_arm1 = _make_pie_panel(
            0, "Arm 1 (MG400) — Runtime Distribution"
        )
        self._fig_ej, self._ax_ej, self._canvas_ej = _make_pie_panel(
            1, "Ejector Device — Runtime Distribution"
        )
        self._draw_pie(self._ax_arm1, self._canvas_arm1,
                       {"moving": 1.0, "idle": 1.0, "error": 0.0})
        self._draw_pie(self._ax_ej, self._canvas_ej,
                       {"moving": 1.0, "idle": 1.0, "error": 0.0})

    def _draw_pie(self, ax, canvas, runtime: dict):
        moving = max(float(runtime.get("moving", 0)), 0)
        idle   = max(float(runtime.get("idle",   0)), 0)
        error  = max(float(runtime.get("error",  0)), 0)
        if moving + idle + error <= 0:
            moving = idle = 1.0
        ax.clear()
        wedges, texts, autotexts = ax.pie(
            [moving, idle, error],
            labels=["Moving", "Idle", "Error"],
            autopct="%1.1f%%",
            startangle=90,
            colors=["#42A5F5", "#66BB6A", "#EF5350"],
            wedgeprops={"edgecolor": "#12122A", "linewidth": 1.5},
        )
        for t in texts:
            t.set_color("#ECEFF1")
            t.set_fontsize(11)
        for at in autotexts:
            at.set_color("white")
            at.set_fontsize(10)
        ax.set_facecolor("#1A1A2E")
        ax.axis("equal")
        canvas.draw_idle()

    # ═══════════════════════════════════════════ Tab 4: Database Log ════════

    def _build_tab_db(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(2, weight=1)

        hdr = ctk.CTkFrame(parent, corner_radius=8)
        hdr.grid(row=0, column=0, padx=8, pady=(8, 4), sticky="ew")
        hdr.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(hdr, text="Workpiece Log — Belt 2 Entries (Last 50)",
                      font=ctk.CTkFont(size=16, weight="bold")).grid(
            row=0, column=0, padx=16, pady=10, sticky="w"
        )
        ctk.CTkButton(hdr, text="↻  Refresh", width=100,
                       command=self._refresh_log_table).grid(
            row=0, column=1, padx=8, pady=8
        )

        COL_HDRS = ["Row ID", "WP ID", "Quality", "Pickup Time",
                    "Exit Time", "Temp (°C)", "Humidity (%)"]
        chdr = ctk.CTkFrame(parent, fg_color="#1A1A2E", corner_radius=6)
        chdr.grid(row=1, column=0, padx=8, pady=(4, 2), sticky="ew")
        for ci, txt in enumerate(COL_HDRS):
            ctk.CTkLabel(chdr, text=txt,
                          font=ctk.CTkFont(size=13, weight="bold"),
                          text_color="#90A4AE").grid(
                row=0, column=ci, padx=14, pady=6, sticky="w"
            )

        self._log_scroll = ctk.CTkScrollableFrame(parent, corner_radius=8)
        self._log_scroll.grid(row=2, column=0, padx=8, pady=(2, 8), sticky="nsew")
        self._log_scroll.grid_columnconfigure(0, weight=1)
        self._refresh_log_table()

    def _refresh_log_table(self):
        for w in self._log_scroll.winfo_children():
            w.destroy()
        for ri, row in enumerate(self._load_log_rows()):
            row_id, wp_id, quality, pickup, exit_t, temp, hum = row
            quality_lc = str(quality or "").lower()
            fg, tc = _QUALITY_COLORS.get(quality_lc, ("#607D8B", "white"))
            bg = "#1E1E3A" if ri % 2 == 0 else "#16162A"
            rf = ctk.CTkFrame(self._log_scroll, fg_color=bg, corner_radius=4)
            rf.grid(row=ri, column=0, padx=4, pady=1, sticky="ew")
            vals = [
                str(row_id), str(wp_id or "—"),
                (quality or "?").upper(),
                str(pickup or "—"), str(exit_t or "—"),
                f"{temp:.1f}" if temp is not None else "—",
                f"{hum:.1f}" if hum is not None else "—",
            ]
            for ci, val in enumerate(vals):
                if ci == 2:
                    w = ctk.CTkLabel(rf, text=val,
                                      font=ctk.CTkFont(size=12, weight="bold"),
                                      fg_color=fg, text_color=tc,
                                      corner_radius=4, width=65)
                else:
                    w = ctk.CTkLabel(rf, text=val, font=ctk.CTkFont(size=13))
                w.grid(row=0, column=ci, padx=12, pady=6, sticky="w")

    # ═══════════════════════════════════════════════ MQTT ═══════════════════

    def _connect_mqtt(self):
        if self._connected:
            self._disconnect_mqtt()
            return

        broker = self._broker_e.get().strip()
        try:
            port = int(self._port_e.get().strip())
        except ValueError:
            self._conn_lbl.configure(text="Invalid port", text_color="tomato")
            return
        if not broker:
            self._conn_lbl.configure(text="Broker required", text_color="tomato")
            return

        self._mqtt_client = mqtt.Client()
        self._mqtt_client.on_connect    = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message    = self._on_mqtt_message
        try:
            self._mqtt_client.connect_async(broker, port, keepalive=60)
            self._mqtt_client.loop_start()
            self._conn_lbl.configure(text="● Connecting…", text_color="gold")
        except Exception as exc:
            self._conn_lbl.configure(text=f"Error: {exc}", text_color="tomato")

    def _disconnect_mqtt(self):
        if self._mqtt_client:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()
            self._mqtt_client = None
        self._connected = False
        self._conn_btn.configure(text="CONNECT")
        self._conn_lbl.configure(text="● Disconnected", text_color="tomato")

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            for entry in (self._sub_status_e, self._sub_vision_e):
                t = entry.get().strip()
                if t:
                    client.subscribe(t, qos=1)
            self._connected = True
            self._conn_btn.configure(text="DISCONNECT")
            self._conn_lbl.configure(text="● Connected", text_color="#66BB6A")
        else:
            self._conn_lbl.configure(text=f"● MQTT rc={rc}", text_color="tomato")

    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        self._connected = False
        self._conn_btn.configure(text="CONNECT")
        self._conn_lbl.configure(text="● Disconnected", text_color="tomato")

    def _on_mqtt_message(self, client, userdata, msg):
        raw = msg.payload.decode("utf-8", errors="ignore")
        decrypted = self._try_decrypt(raw)
        try:
            payload = json.loads(decrypted)
            payload["_topic"] = msg.topic
            self._msg_queue.put(payload)
        except json.JSONDecodeError:
            pass

    def _process_queue(self):
        while not self._msg_queue.empty():
            self._handle_payload(self._msg_queue.get())
        self.after(100, self._process_queue)

    # ═══════════════════════════════════════ Payload dispatch ═══════════════

    def _handle_payload(self, payload: dict):
        # Vision / workpiece data
        if "workpieces" in payload:
            self._workpieces = list(payload.get("workpieces", []))
            self._pick_order = list(payload.get("pick_order", []))
            self._detected_lbl.configure(
                text=f"Detected Workpieces: {len(self._workpieces)}"
            )
            order_str = (
                " → ".join(str(x) for x in self._pick_order)
                if self._pick_order else "—"
            )
            self._pick_order_lbl.configure(text=order_str)
            self._refresh_workpiece_rows()

        # Reported joint angles — per-arm keys (combined payload)
        for arm_id, key in [("arm1", "arm1_deg"),
                             ("arm2", "arm2_deg"),
                             ("ejector", "ejector_deg")]:
            if key in payload:
                angles = list(payload[key])
                rep_lbl = getattr(self, f"_reported_lbl_{arm_id}", None)
                if rep_lbl:
                    rep_lbl.configure(
                        text="  ".join(f"J{i+1}={a:.1f}°"
                                       for i, a in enumerate(angles))
                    )

        # Legacy single-arm key (backward compat — maps to arm1)
        if "joint_deg" in payload:
            angles = list(payload["joint_deg"])[:4]
            rep_lbl = getattr(self, "_reported_lbl_arm1", None)
            if rep_lbl:
                rep_lbl.configure(
                    text="  ".join(f"J{i+1}={a:.1f}°" for i, a in enumerate(angles))
                )

        # Route remaining fields
        self._apply_device_status(payload)
        self._apply_counts(payload)
        self._apply_speeds(payload)
        self._apply_sensor(payload)
        self._apply_pie_charts(payload)
        self._apply_db_events(payload)

    def _apply_device_status(self, p: dict):
        for key in ("arm1", "arm2", "belt1", "belt2", "ejector"):
            raw = str(p.get(f"{key}_status", "")).strip().lower()
            if not raw:
                continue
            badge = self._dev_badges.get(key)
            if badge:
                fg, tc = _STATUS_COLORS.get(raw, ("#546E7A", "white"))
                badge.configure(text=raw.upper(), fg_color=fg, text_color=tc)

    def _apply_counts(self, p: dict):
        counts = p.get("counts", {})
        for key in ("total_in", "good_out", "bad_out", "total_out"):
            val = counts.get(key)
            if val is not None:
                self._counts[key] = int(val)
                self._count_labels[key].configure(text=str(int(val)))
        intrusion = p.get("intrusion_count")
        if intrusion is not None:
            self._intrusion_count = int(intrusion)
            self._intrusion_lbl.configure(text=str(self._intrusion_count))

    def _apply_speeds(self, p: dict):
        s1 = p.get("conveyor1_speed_mps", p.get("conveyor_speed_mps"))
        s2 = p.get("conveyor2_speed_mps")
        if s1 is not None:
            self._speed1_lbl.configure(text=f"{float(s1):.3f} m/s")
        if s2 is not None:
            self._speed2_lbl.configure(text=f"{float(s2):.3f} m/s")

    def _apply_sensor(self, p: dict):
        raw = str(p.get("safety_sensor_status",
                         p.get("safety_sensor", ""))).strip().lower()
        if not raw:
            return
        DISPLAY = {
            "normal":    "NORMAL",
            "detecting": "DETECTING",
            "damaged":   "DAMAGED",
            "unknown":   "UNKNOWN",
        }
        fg, tc = _STATUS_COLORS.get(raw, ("#546E7A", "white"))
        self._sensor_badge.configure(
            text=DISPLAY.get(raw, raw.upper()), fg_color=fg, text_color=tc
        )

    def _apply_pie_charts(self, p: dict):
        a1 = p.get("arm1_runtime", p.get("arm_runtime"))
        if a1:
            self._arm1_runtime.update(a1)
            self._draw_pie(self._ax_arm1, self._canvas_arm1, self._arm1_runtime)
        ej = p.get("ejector_runtime")
        if ej:
            self._ejector_runtime.update(ej)
            self._draw_pie(self._ax_ej, self._canvas_ej, self._ejector_runtime)

    def _apply_db_events(self, p: dict):
        # Workpiece enters belt 2 (arm just picked it up)
        entry = p.get("conveyor2_entry")
        if entry:
            wp_id       = entry.get("workpiece_id")
            quality     = entry.get("quality", "unknown")
            pickup_time = entry.get("pickup_time",
                                    datetime.now().isoformat(timespec="seconds"))
            self._db_log_workpiece(wp_id, quality, pickup_time)
            self._db_event(f"belt2_entry wp_id={wp_id} quality={quality}")

        # Workpiece exits belt 2 (leaves the system)
        exit_ev = p.get("conveyor2_exit")
        if exit_ev:
            wp_id   = exit_ev.get("workpiece_id")
            exit_t  = exit_ev.get("exit_time",
                                   datetime.now().isoformat(timespec="seconds"))
            temp_c  = exit_ev.get("temperature_c")
            hum_pct = exit_ev.get("humidity_pct")
            self._db_update_exit(wp_id, exit_t, temp_c, hum_pct)
            self._db_event(
                f"belt2_exit wp_id={wp_id} temp={temp_c}°C hum={hum_pct}%"
            )

        # Legacy single-event (backward compat)
        if p.get("conveyor2_passed"):
            now = datetime.now().isoformat(timespec="seconds")
            self._db_log_workpiece(
                None, None, now, now,
                p.get("temperature_c"), p.get("humidity_pct"),
            )

    # ═══════════════════════════════════════════ Commands ═══════════════════

    def _publish(self, payload: dict):
        if not self._connected or not self._mqtt_client:
            self._conn_lbl.configure(text="Not connected", text_color="tomato")
            return
        topic = self._pub_e.get().strip()
        if not topic:
            return
        try:
            raw = json.dumps(payload, ensure_ascii=False).encode()
            data = self._get_cipher().encrypt(raw) if self._fernet_enabled.get() else raw
            self._mqtt_client.publish(topic, data, qos=1)
        except Exception as exc:
            self._conn_lbl.configure(text=f"Pub error: {exc}", text_color="tomato")

    def _send_start(self):
        self._publish({"cmd": "START", "ts": datetime.now().isoformat()})
        self._db_event("CMD: START")

    def _send_stop(self):
        self._publish({"cmd": "STOP", "ts": datetime.now().isoformat()})
        self._db_event("CMD: STOP")

    def _send_estop(self):
        self._publish({"cmd": "EMERGENCY_STOP", "ts": datetime.now().isoformat()})
        self._db_event("CMD: EMERGENCY_STOP from GUI")

    def _send_joints(self, arm_id: str = "arm1"):
        var_list = getattr(self, f"_joint_vars_{arm_id}", [])
        deg = [round(v.get(), 2) for v in var_list]
        self._publish({"cmd": "SET_JOINTS", "target": arm_id,
                        "joints_deg": deg, "ts": datetime.now().isoformat()})

    def _reset_joints(self, arm_id: str = "arm1"):
        var_list = getattr(self, f"_joint_vars_{arm_id}", [])
        lbl_list = getattr(self, f"_joint_val_labels_{arm_id}", [])
        for var, lbl in zip(var_list, lbl_list):
            var.set(0.0)
            lbl.configure(text="  0.0°")

    def _on_joint_drag(self, arm_id: str, idx: int, value: float):
        lbl_list = getattr(self, f"_joint_val_labels_{arm_id}", [])
        if idx < len(lbl_list):
            lbl_list[idx].configure(text=f"  {float(value):.1f}°")

    # ═══════════════════════════════════════ Encryption helpers ═════════════

    def _get_cipher(self) -> Fernet:
        return Fernet(self._key_e.get().strip().encode())

    def _try_decrypt(self, text: str) -> str:
        if not self._fernet_enabled.get():
            return text
        try:
            return self._get_cipher().decrypt(text.encode()).decode("utf-8")
        except (InvalidToken, Exception):
            return text

    def _gen_key(self):
        self._key_e.delete(0, "end")
        self._key_e.insert(0, Fernet.generate_key().decode())

    def _toggle_key(self):
        self._key_e.configure(
            show="" if self._key_e.cget("show") == "•" else "•"
        )

    # ═══════════════════════════════════════════ Digital Twin ════════════════

    def _open_twin(self):
        twin_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "digital_twin_5dof.py"
        )
        if os.path.exists(twin_path):
            subprocess.Popen([sys.executable, twin_path])
        else:
            self._conn_lbl.configure(
                text="digital_twin_5dof.py not found", text_color="tomato"
            )

    # ═══════════════════════════════════════════════ Lifecycle ══════════════

    def _on_close(self):
        self._disconnect_mqtt()
        self._db.close()
        self.destroy()


if __name__ == "__main__":
    app = FactoryRobotGUI()
    app.mainloop()

