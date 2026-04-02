import json
import queue
from math import pi

import customtkinter as ctk
import numpy as np
import paho.mqtt.client as mqtt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    from roboticstoolbox import DHRobot, RevoluteDH
    from spatialmath import SE3
except Exception:
    DHRobot = None
    RevoluteDH = None
    SE3 = None


class DigitalTwin5DOF(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Digital Twin 5-DOF Arm (MQTT)")
        self.geometry("1100x760")

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.queue = queue.Queue()
        self.mqtt_client = None
        self.robot = self._build_robot_model()

        self._build_ui()
        self.after(80, self._process_queue)

    def _build_robot_model(self):
        if DHRobot is None:
            return None
        links = [
            RevoluteDH(d=0.16, a=0.0, alpha=pi / 2),
            RevoluteDH(d=0.0, a=0.28, alpha=0.0),
            RevoluteDH(d=0.0, a=0.22, alpha=0.0),
            RevoluteDH(d=0.0, a=0.0, alpha=pi / 2),
            RevoluteDH(d=0.12, a=0.0, alpha=0.0),
        ]
        return DHRobot(links, name="FactoryArm5DOF")

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        top = ctk.CTkFrame(self)
        top.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        top.grid_columnconfigure(5, weight=1)

        ctk.CTkLabel(top, text="Broker IP").grid(row=0, column=0, padx=6, pady=8)
        self.broker = ctk.CTkEntry(top, width=180)
        self.broker.insert(0, "127.0.0.1")
        self.broker.grid(row=0, column=1, padx=6, pady=8)

        ctk.CTkLabel(top, text="Port").grid(row=0, column=2, padx=6, pady=8)
        self.port = ctk.CTkEntry(top, width=90)
        self.port.insert(0, "1883")
        self.port.grid(row=0, column=3, padx=6, pady=8)

        ctk.CTkLabel(top, text="Joint Topic").grid(row=0, column=4, padx=6, pady=8)
        self.topic = ctk.CTkEntry(top, width=280)
        self.topic.insert(0, "factory/robot/joint_angles")
        self.topic.grid(row=0, column=5, padx=6, pady=8, sticky="ew")

        self.connect_btn = ctk.CTkButton(top, text="CONNECT", command=self._connect)
        self.connect_btn.grid(row=0, column=6, padx=6, pady=8)

        self.status = ctk.CTkLabel(top, text="Disconnected", text_color="tomato")
        self.status.grid(row=0, column=7, padx=6, pady=8)

        panel = ctk.CTkFrame(self)
        panel.grid(row=1, column=0, padx=10, pady=(0, 10), sticky="nsew")
        panel.grid_columnconfigure(0, weight=1)
        panel.grid_rowconfigure(1, weight=1)

        self.xyz_label = ctk.CTkLabel(panel, text="End-Effector Position: x=0.000, y=0.000, z=0.000", font=ctk.CTkFont(size=18, weight="bold"))
        self.xyz_label.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        self.figure = Figure(figsize=(8.5, 5.6), dpi=100)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.figure, master=panel)
        self.canvas.get_tk_widget().grid(row=1, column=0, padx=10, pady=(0, 10), sticky="nsew")

        if self.robot is None:
            self.status.configure(text="Install roboticstoolbox-python", text_color="orange")
        self._draw_robot(np.zeros(5))

    def _connect(self):
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None
            self.connect_btn.configure(text="CONNECT")
            self.status.configure(text="Disconnected", text_color="tomato")
            return

        try:
            port = int(self.port.get().strip())
        except ValueError:
            self.status.configure(text="Invalid Port", text_color="tomato")
            return

        broker = self.broker.get().strip()
        topic = self.topic.get().strip()
        if not broker or not topic:
            self.status.configure(text="Broker/Topic Required", text_color="tomato")
            return

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.on_disconnect = self._on_disconnect

        self.mqtt_client.connect_async(broker, port, keepalive=60)
        self.mqtt_client.loop_start()
        self.status.configure(text="Connecting...", text_color="gold")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(self.topic.get().strip())
            self.connect_btn.configure(text="DISCONNECT")
            self.status.configure(text="Connected", text_color="#66BB6A")
        else:
            self.status.configure(text=f"MQTT Error rc={rc}", text_color="tomato")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.connect_btn.configure(text="CONNECT")
        self.status.configure(text="Disconnected", text_color="tomato")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8", errors="ignore"))
            joint_deg = payload.get("joint_deg", [0, 0, 0, 0, 0])
            if isinstance(joint_deg, list) and len(joint_deg) == 5:
                self.queue.put(joint_deg)
        except json.JSONDecodeError:
            pass

    def _process_queue(self):
        while not self.queue.empty():
            joint_deg = self.queue.get()
            q_rad = np.radians(np.array(joint_deg, dtype=float))
            self._draw_robot(q_rad)
        self.after(80, self._process_queue)

    def _draw_robot(self, q):
        self.ax.clear()
        points = self._fk_points(q)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        self.ax.plot(x, y, z, marker="o", linewidth=2.5, color="#42A5F5")
        self.ax.set_xlim(-0.7, 0.7)
        self.ax.set_ylim(-0.7, 0.7)
        self.ax.set_zlim(0.0, 0.8)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("5-DOF Digital Twin")

        ee = points[-1]
        self.xyz_label.configure(text=f"End-Effector Position: x={ee[0]:.3f}, y={ee[1]:.3f}, z={ee[2]:.3f}")
        self.canvas.draw_idle()

    def _fk_points(self, q):
        if self.robot is not None and SE3 is not None:
            t = SE3()
            points = [t.t]
            for i, link in enumerate(self.robot.links):
                t = t * link.A(q[i])
                points.append(t.t)
            return np.array(points)

        d = [0.16, 0.0, 0.0, 0.0, 0.12]
        a = [0.0, 0.28, 0.22, 0.0, 0.0]
        alpha = [pi / 2, 0.0, 0.0, pi / 2, 0.0]

        t = np.eye(4)
        points = [t[:3, 3].copy()]

        for i in range(5):
            ct = np.cos(q[i])
            st = np.sin(q[i])
            ca = np.cos(alpha[i])
            sa = np.sin(alpha[i])
            ti = np.array(
                [
                    [ct, -st * ca, st * sa, a[i] * ct],
                    [st, ct * ca, -ct * sa, a[i] * st],
                    [0, sa, ca, d[i]],
                    [0, 0, 0, 1],
                ]
            )
            t = t @ ti
            points.append(t[:3, 3].copy())

        return np.array(points)


if __name__ == "__main__":
    app = DigitalTwin5DOF()
    app.mainloop()
