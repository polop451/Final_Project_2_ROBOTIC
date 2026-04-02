import json
import queue
import sqlite3
from datetime import datetime

import customtkinter as ctk
import paho.mqtt.client as mqtt
from cryptography.fernet import Fernet, InvalidToken
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class FactoryRobotGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Factory Robotics Control - MQTT + CustomTkinter")
        self.geometry("1280x860")

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.mqtt_client = None
        self.incoming_queue = queue.Queue()
        self.connected = False

        self.db_conn = sqlite3.connect("factory_data.db", check_same_thread=False)
        self._init_db()

        self._build_ui()
        self.after(100, self._process_incoming_messages)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _init_db(self):
        cursor = self.db_conn.cursor()
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS conveyor2_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                temperature_c REAL,
                humidity_pct REAL
            )
            """
        )
        self.db_conn.commit()

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)

        self._build_connection_panel()
        self._build_control_panel()
        self._build_dashboard_panel()
        self._build_chart_panel()

    def _build_connection_panel(self):
        frame = ctk.CTkFrame(self)
        frame.grid(row=0, column=0, columnspan=2, padx=12, pady=12, sticky="ew")
        frame.grid_columnconfigure(9, weight=1)

        ctk.CTkLabel(frame, text="Broker IP").grid(row=0, column=0, padx=8, pady=8)
        self.broker_entry = ctk.CTkEntry(frame, width=180)
        self.broker_entry.insert(0, "127.0.0.1")
        self.broker_entry.grid(row=0, column=1, padx=8, pady=8)

        ctk.CTkLabel(frame, text="Port").grid(row=0, column=2, padx=8, pady=8)
        self.port_entry = ctk.CTkEntry(frame, width=90)
        self.port_entry.insert(0, "1883")
        self.port_entry.grid(row=0, column=3, padx=8, pady=8)

        ctk.CTkLabel(frame, text="Subscribe Topic").grid(row=0, column=4, padx=8, pady=8)
        self.sub_topic_entry = ctk.CTkEntry(frame, width=220)
        self.sub_topic_entry.insert(0, "factory/robot/status")
        self.sub_topic_entry.grid(row=0, column=5, padx=8, pady=8)

        ctk.CTkLabel(frame, text="Publish Topic").grid(row=0, column=6, padx=8, pady=8)
        self.pub_topic_entry = ctk.CTkEntry(frame, width=220)
        self.pub_topic_entry.insert(0, "factory/robot/cmd")
        self.pub_topic_entry.grid(row=0, column=7, padx=8, pady=8)

        self.connect_btn = ctk.CTkButton(frame, text="CONNECT", command=self._connect_mqtt, width=120)
        self.connect_btn.grid(row=0, column=8, padx=8, pady=8)

        self.status_label = ctk.CTkLabel(frame, text="Disconnected", text_color="tomato")
        self.status_label.grid(row=0, column=9, padx=8, pady=8, sticky="e")

        encryption_frame = ctk.CTkFrame(self)
        encryption_frame.grid(row=1, column=0, columnspan=2, padx=12, pady=(0, 12), sticky="ew")
        encryption_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(encryption_frame, text="Fernet Key").grid(row=0, column=0, padx=8, pady=8)
        self.fernet_key_entry = ctk.CTkEntry(encryption_frame)
        self.fernet_key_entry.insert(0, Fernet.generate_key().decode())
        self.fernet_key_entry.grid(row=0, column=1, padx=8, pady=8, sticky="ew")

        ctk.CTkButton(encryption_frame, text="GENERATE KEY", command=self._generate_key, width=130).grid(
            row=0, column=2, padx=8, pady=8
        )

    def _build_control_panel(self):
        frame = ctk.CTkFrame(self)
        frame.grid(row=2, column=0, padx=(12, 6), pady=12, sticky="nsew")
        frame.grid_columnconfigure(0, weight=1)

        self.start_btn = ctk.CTkButton(
            frame,
            text="START",
            font=ctk.CTkFont(size=34, weight="bold"),
            height=120,
            command=self._send_start,
        )
        self.start_btn.grid(row=0, column=0, padx=16, pady=(16, 12), sticky="ew")

        self.estop_btn = ctk.CTkButton(
            frame,
            text="EMERGENCY STOP",
            fg_color="#D32F2F",
            hover_color="#B71C1C",
            font=ctk.CTkFont(size=30, weight="bold"),
            height=120,
            command=self._send_emergency_stop,
        )
        self.estop_btn.grid(row=1, column=0, padx=16, pady=12, sticky="ew")

        angle_frame = ctk.CTkFrame(frame)
        angle_frame.grid(row=2, column=0, padx=16, pady=12, sticky="ew")
        angle_frame.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(angle_frame, text="Arm Joint Angle (deg)", font=ctk.CTkFont(size=18, weight="bold")).grid(
            row=0, column=0, padx=12, pady=(10, 2), sticky="w"
        )

        self.angle_var = ctk.DoubleVar(value=90.0)
        self.angle_slider = ctk.CTkSlider(
            angle_frame,
            from_=0,
            to=180,
            variable=self.angle_var,
            command=self._on_angle_change,
        )
        self.angle_slider.grid(row=1, column=0, padx=12, pady=8, sticky="ew")

        self.angle_value_label = ctk.CTkLabel(angle_frame, text="90.0°", font=ctk.CTkFont(size=16))
        self.angle_value_label.grid(row=2, column=0, padx=12, pady=4, sticky="w")

        ctk.CTkButton(angle_frame, text="SEND ANGLE", command=self._send_angle).grid(
            row=3, column=0, padx=12, pady=(8, 12), sticky="ew"
        )

    def _build_dashboard_panel(self):
        frame = ctk.CTkFrame(self)
        frame.grid(row=2, column=1, padx=(6, 12), pady=12, sticky="nsew")
        frame.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkLabel(frame, text="Production Dashboard", font=ctk.CTkFont(size=24, weight="bold")).grid(
            row=0, column=0, columnspan=2, padx=12, pady=(14, 10), sticky="w"
        )

        self.conveyor_var = ctk.StringVar(value="Conveyor Speed: 0.00 m/s")
        self.good_var = ctk.StringVar(value="Good: 0")
        self.bad_var = ctk.StringVar(value="Bad: 0")
        self.total_var = ctk.StringVar(value="Total: 0")
        self.safety_sensor_var = ctk.StringVar(value="Safety Sensor: UNKNOWN")

        ctk.CTkLabel(frame, textvariable=self.conveyor_var, font=ctk.CTkFont(size=18)).grid(
            row=1, column=0, columnspan=2, padx=12, pady=6, sticky="w"
        )
        ctk.CTkLabel(frame, textvariable=self.good_var, font=ctk.CTkFont(size=18)).grid(
            row=2, column=0, padx=12, pady=6, sticky="w"
        )
        ctk.CTkLabel(frame, textvariable=self.bad_var, font=ctk.CTkFont(size=18)).grid(
            row=2, column=1, padx=12, pady=6, sticky="w"
        )
        ctk.CTkLabel(frame, textvariable=self.total_var, font=ctk.CTkFont(size=18)).grid(
            row=3, column=0, padx=12, pady=6, sticky="w"
        )
        ctk.CTkLabel(frame, textvariable=self.safety_sensor_var, font=ctk.CTkFont(size=18)).grid(
            row=3, column=1, padx=12, pady=6, sticky="w"
        )

        sensor_health_title = ctk.CTkLabel(frame, text="Sensor Health", font=ctk.CTkFont(size=18, weight="bold"))
        sensor_health_title.grid(row=4, column=0, padx=12, pady=(14, 4), sticky="w")

        self.sensor_health_label = ctk.CTkLabel(
            frame,
            text="Normal",
            font=ctk.CTkFont(size=18, weight="bold"),
            text_color="black",
            fg_color="#66BB6A",
            corner_radius=8,
            width=130,
        )
        self.sensor_health_label.grid(row=4, column=1, padx=12, pady=(14, 4), sticky="w")

    def _build_chart_panel(self):
        frame = ctk.CTkFrame(self)
        frame.grid(row=3, column=0, columnspan=2, padx=12, pady=(0, 12), sticky="nsew")
        frame.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(frame, text="Arm Runtime Distribution", font=ctk.CTkFont(size=22, weight="bold")).grid(
            row=0, column=0, padx=12, pady=(12, 6)
        )

        self.figure = Figure(figsize=(6.5, 3.2), dpi=100)
        self.pie_ax = self.figure.add_subplot(111)
        self.pie_canvas = FigureCanvasTkAgg(self.figure, master=frame)
        self.pie_canvas.get_tk_widget().grid(row=1, column=0, padx=12, pady=(0, 12), sticky="nsew")

        self._update_pie_chart({"moving": 1, "idle": 1, "error": 1})

    def _generate_key(self):
        self.fernet_key_entry.delete(0, "end")
        self.fernet_key_entry.insert(0, Fernet.generate_key().decode())

    def _connect_mqtt(self):
        if self.connected:
            self._disconnect_mqtt()
            return

        broker = self.broker_entry.get().strip()
        topic = self.sub_topic_entry.get().strip()
        try:
            port = int(self.port_entry.get().strip())
        except ValueError:
            self.status_label.configure(text="Invalid Port", text_color="tomato")
            return

        if not broker or not topic:
            self.status_label.configure(text="Broker/Topic Required", text_color="tomato")
            return

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_message = self._on_message

        try:
            self.mqtt_client.connect_async(broker, port, keepalive=60)
            self.mqtt_client.loop_start()
            self.status_label.configure(text="Connecting...", text_color="gold")
        except Exception as exc:
            self.status_label.configure(text=f"Connect Error: {exc}", text_color="tomato")

    def _disconnect_mqtt(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None
        self.connected = False
        self.connect_btn.configure(text="CONNECT")
        self.status_label.configure(text="Disconnected", text_color="tomato")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            sub_topic = self.sub_topic_entry.get().strip()
            client.subscribe(sub_topic)
            self.connected = True
            self.connect_btn.configure(text="DISCONNECT")
            self.status_label.configure(text=f"Connected: {sub_topic}", text_color="#66BB6A")
        else:
            self.status_label.configure(text=f"MQTT Error rc={rc}", text_color="tomato")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.connected = False
        self.connect_btn.configure(text="CONNECT")
        self.status_label.configure(text="Disconnected", text_color="tomato")

    def _on_message(self, client, userdata, msg):
        raw_payload = msg.payload.decode("utf-8", errors="ignore")
        decoded_payload = self._try_decrypt_payload(raw_payload)
        try:
            payload_json = json.loads(decoded_payload)
            self.incoming_queue.put(payload_json)
        except json.JSONDecodeError:
            pass

    def _get_cipher(self):
        key_text = self.fernet_key_entry.get().strip().encode()
        return Fernet(key_text)

    def _encrypt_payload(self, payload_dict):
        plaintext = json.dumps(payload_dict, ensure_ascii=False).encode()
        cipher = self._get_cipher()
        return cipher.encrypt(plaintext).decode()

    def _try_decrypt_payload(self, payload_text):
        try:
            cipher = self._get_cipher()
            return cipher.decrypt(payload_text.encode()).decode("utf-8")
        except (ValueError, InvalidToken):
            return payload_text

    def _publish_encrypted(self, payload_dict):
        if not self.connected or not self.mqtt_client:
            self.status_label.configure(text="MQTT not connected", text_color="tomato")
            return

        topic = self.pub_topic_entry.get().strip()
        if not topic:
            self.status_label.configure(text="Publish topic required", text_color="tomato")
            return

        try:
            encrypted_payload = self._encrypt_payload(payload_dict)
            self.mqtt_client.publish(topic, encrypted_payload, qos=1)
        except Exception as exc:
            self.status_label.configure(text=f"Publish Error: {exc}", text_color="tomato")

    def _send_start(self):
        self._publish_encrypted({"command": "START", "timestamp": datetime.now().isoformat()})

    def _send_emergency_stop(self):
        self._publish_encrypted({"command": "EMERGENCY_STOP", "timestamp": datetime.now().isoformat()})

    def _send_angle(self):
        angle = round(float(self.angle_var.get()), 2)
        self._publish_encrypted({"command": "SET_ANGLE", "angle_deg": angle, "timestamp": datetime.now().isoformat()})

    def _on_angle_change(self, value):
        self.angle_value_label.configure(text=f"{float(value):.1f}°")

    def _process_incoming_messages(self):
        while not self.incoming_queue.empty():
            payload = self.incoming_queue.get()
            self._update_dashboard(payload)
        self.after(100, self._process_incoming_messages)

    def _update_dashboard(self, payload):
        conveyor_speed = float(payload.get("conveyor_speed_mps", 0.0))
        counts = payload.get("counts", {})
        good = int(counts.get("good", 0))
        bad = int(counts.get("bad", 0))
        total = int(counts.get("total", good + bad))
        safety_sensor = str(payload.get("safety_sensor", "UNKNOWN"))

        self.conveyor_var.set(f"Conveyor Speed: {conveyor_speed:.2f} m/s")
        self.good_var.set(f"Good: {good}")
        self.bad_var.set(f"Bad: {bad}")
        self.total_var.set(f"Total: {total}")
        self.safety_sensor_var.set(f"Safety Sensor: {safety_sensor}")

        sensor_health = str(payload.get("sensor_health", "Normal"))
        self._update_sensor_health(sensor_health)

        runtime_data = payload.get("arm_runtime", {})
        self._update_pie_chart(runtime_data)

        if payload.get("conveyor2_passed", False):
            temperature = payload.get("temperature_c")
            humidity = payload.get("humidity_pct")
            self._log_conveyor2_data(temperature, humidity)

    def _update_sensor_health(self, status):
        status_normalized = status.strip().lower()
        mapping = {
            "normal": ("Normal", "#66BB6A"),
            "warning": ("Warning", "#FFB300"),
            "fault": ("Fault", "#EF5350"),
        }
        text, color = mapping.get(status_normalized, (status, "#90A4AE"))
        self.sensor_health_label.configure(text=text, fg_color=color)

    def _update_pie_chart(self, runtime_data):
        moving = float(runtime_data.get("moving", 0.0))
        idle = float(runtime_data.get("idle", 0.0))
        error = float(runtime_data.get("error", 0.0))

        values = [moving, idle, error]
        labels = ["Moving", "Idle", "Error"]
        colors = ["#42A5F5", "#66BB6A", "#EF5350"]

        if sum(values) <= 0:
            values = [1, 1, 1]

        self.pie_ax.clear()
        self.pie_ax.pie(values, labels=labels, autopct="%1.1f%%", startangle=90, colors=colors)
        self.pie_ax.axis("equal")
        self.figure.tight_layout()
        self.pie_canvas.draw_idle()

    def _log_conveyor2_data(self, temperature, humidity):
        cursor = self.db_conn.cursor()
        cursor.execute(
            "INSERT INTO conveyor2_log (timestamp, temperature_c, humidity_pct) VALUES (?, ?, ?)",
            (datetime.now().isoformat(timespec="seconds"), temperature, humidity),
        )
        self.db_conn.commit()

    def _on_close(self):
        self._disconnect_mqtt()
        self.db_conn.close()
        self.destroy()


if __name__ == "__main__":
    app = FactoryRobotGUI()
    app.mainloop()
