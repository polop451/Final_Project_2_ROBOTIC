import json
import queue
from math import pi, cos, sin

import customtkinter as ctk
import numpy as np
import paho.mqtt.client as mqtt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Dobot MG400 link parameters (metres) — from Dobot-MG400.robot model file ─
_MG_D1 = 0.135   # column height (base-bottom → shoulder pivot)
_MG_A2 = 0.175   # upper-arm length   (file: a2 = 175 mm)
_MG_A3 = 0.175   # forearm length     (file: a3 = 175 mm)
_MG_D4 = 0.066   # wrist → TCP        (file: d4 = 66 mm)

# ── 2-DOF Ejector arm parameters (metres) ────────────────────────────────────
_EJ_H    = 0.080  # column height
_EJ_L1   = 0.200  # boom length
_EJ_TOOL = 0.055  # vertical tool offset (down)

# ── World base positions for each robot (metres) ──────────────────────────────
#   Dobot 1  – left side, at Belt-1 input
#   Dobot 2  – right side, at Belt-2 input
#   Ejector  – centre, between the two belts
_BASE_ARM1 = np.array([-0.58, -0.10, 0.0])
_BASE_ARM2 = np.array([ 0.58, -0.10, 0.0])
_BASE_EJ   = np.array([ 0.00,  0.22, 0.0])


# ─────────────────────────────────────────────────────────────────────────────
#  3-D geometry helpers (module-level)
# ─────────────────────────────────────────────────────────────────────────────

def _cylinder_faces(p0, p1, radius=0.022, n=16):
    """Quad + cap faces for a cylinder between p0 and p1."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    v = p1 - p0
    length = np.linalg.norm(v)
    if length < 1e-9:
        return []
    v_hat = v / length
    ref = np.array([0., 0., 1.]) if abs(v_hat[2]) < 0.9 else np.array([1., 0., 0.])
    u = np.cross(v_hat, ref);  u /= np.linalg.norm(u)
    w = np.cross(v_hat, u)
    theta = np.linspace(0, 2 * pi, n, endpoint=False)
    c0 = p0 + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), w))
    c1 = p1 + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), w))
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append([c0[i].tolist(), c0[j].tolist(), c1[j].tolist(), c1[i].tolist()])
    faces.append([c0[i].tolist() for i in range(n)])
    faces.append([c1[i].tolist() for i in range(n)])
    return faces


def _box_faces(center, lx, ly, lz):
    """Six quad faces for an axis-aligned box."""
    cx, cy, cz = center
    dx, dy, dz = lx / 2, ly / 2, lz / 2
    xm, xp = cx - dx, cx + dx
    ym, yp = cy - dy, cy + dy
    zm, zp = cz - dz, cz + dz
    return [
        [[xm,ym,zm],[xp,ym,zm],[xp,yp,zm],[xm,yp,zm]],
        [[xm,ym,zp],[xp,ym,zp],[xp,yp,zp],[xm,yp,zp]],
        [[xm,ym,zm],[xp,ym,zm],[xp,ym,zp],[xm,ym,zp]],
        [[xm,yp,zm],[xp,yp,zm],[xp,yp,zp],[xm,yp,zp]],
        [[xm,ym,zm],[xm,yp,zm],[xm,yp,zp],[xm,ym,zp]],
        [[xp,ym,zm],[xp,yp,zm],[xp,yp,zp],[xp,ym,zp]],
    ]


# ─────────────────────────────────────────────────────────────────────────────
#  Digital Twin — 2× Dobot MG400  +  2-DOF Ejector
# ─────────────────────────────────────────────────────────────────────────────

class DigitalTwin5DOF(ctk.CTk):

    def __init__(self):
        super().__init__()
        self.title("Digital Twin — 2× Dobot MG400 + Ejector 2-DOF  (MQTT)")
        self.geometry("1280x820")

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.queue = queue.Queue()
        self.mqtt_client = None

        # Per-arm joint angles (radians)
        self._q_arm1    = np.zeros(4)   # Dobot MG400 #1
        self._q_arm2    = np.zeros(4)   # Dobot MG400 #2
        self._q_ejector = np.zeros(2)   # 2-DOF Ejector

        self._build_ui()
        self.after(80, self._process_queue)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)

        # ── Connection bar (row 0) ──────────────────────────────────────────
        top = ctk.CTkFrame(self, corner_radius=8)
        top.grid(row=0, column=0, padx=10, pady=(10, 4), sticky="ew")
        top.grid_columnconfigure(9, weight=1)

        def _lbl(text, col):
            ctk.CTkLabel(top, text=text).grid(row=0, column=col, padx=(10, 2), pady=8)

        _lbl("Broker IP", 0)
        self.broker = ctk.CTkEntry(top, width=145)
        self.broker.insert(0, "127.0.0.1")
        self.broker.grid(row=0, column=1, padx=4)

        _lbl("Port", 2)
        self.port = ctk.CTkEntry(top, width=60)
        self.port.insert(0, "1883")
        self.port.grid(row=0, column=3, padx=4)

        _lbl("Arm 1 Topic", 4)
        self.topic1 = ctk.CTkEntry(top, width=200)
        self.topic1.insert(0, "factory/robot1/joint_angles")
        self.topic1.grid(row=0, column=5, padx=4)

        _lbl("Arm 2 Topic", 6)
        self.topic2 = ctk.CTkEntry(top, width=200)
        self.topic2.insert(0, "factory/robot2/joint_angles")
        self.topic2.grid(row=0, column=7, padx=4)

        _lbl("Ejector Topic", 8)
        self.topic3 = ctk.CTkEntry(top, width=200)
        self.topic3.insert(0, "factory/ejector/joint_angles")
        self.topic3.grid(row=0, column=9, padx=4, sticky="ew")

        self.connect_btn = ctk.CTkButton(top, text="CONNECT",
                                          command=self._connect, width=110)
        self.connect_btn.grid(row=0, column=10, padx=8)

        self.status = ctk.CTkLabel(top, text="● Disconnected",
                                    text_color="tomato",
                                    font=ctk.CTkFont(size=12, weight="bold"))
        self.status.grid(row=0, column=11, padx=8)

        # ── EE position bar (row 1) ─────────────────────────────────────────
        ee_bar = ctk.CTkFrame(self, corner_radius=8)
        ee_bar.grid(row=1, column=0, padx=10, pady=4, sticky="ew")
        ee_bar.grid_columnconfigure((0, 1, 2), weight=1)

        self._ee1_lbl = ctk.CTkLabel(
            ee_bar,
            text="Arm 1 (MG400):  x=+000.0  y=+000.0  z=+000.0 mm",
            font=ctk.CTkFont(size=13, weight="bold"), text_color="#42A5F5",
        )
        self._ee1_lbl.grid(row=0, column=0, padx=16, pady=8, sticky="w")

        self._ee2_lbl = ctk.CTkLabel(
            ee_bar,
            text="Arm 2 (MG400):  x=+000.0  y=+000.0  z=+000.0 mm",
            font=ctk.CTkFont(size=13, weight="bold"), text_color="#66BB6A",
        )
        self._ee2_lbl.grid(row=0, column=1, padx=16, pady=8)

        self._ee3_lbl = ctk.CTkLabel(
            ee_bar,
            text="Ejector (2-DOF):  x=+000.0  y=+000.0  z=+000.0 mm",
            font=ctk.CTkFont(size=13, weight="bold"), text_color="#FF9800",
        )
        self._ee3_lbl.grid(row=0, column=2, padx=16, pady=8, sticky="e")

        # ── 3D canvas (row 2) ───────────────────────────────────────────────
        panel = ctk.CTkFrame(self, corner_radius=8)
        panel.grid(row=2, column=0, padx=10, pady=(0, 10), sticky="nsew")
        panel.grid_columnconfigure(0, weight=1)
        panel.grid_rowconfigure(0, weight=1)

        self.figure = Figure(figsize=(10, 5.8), dpi=100)
        self.figure.patch.set_facecolor("#12122A")
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.figure, master=panel)
        self.canvas.get_tk_widget().grid(row=0, column=0, padx=8, pady=8,
                                          sticky="nsew")
        self._draw_scene()

    # ── MQTT ──────────────────────────────────────────────────────────────────

    def _connect(self):
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None
            self.connect_btn.configure(text="CONNECT")
            self.status.configure(text="● Disconnected", text_color="tomato")
            return

        try:
            port = int(self.port.get().strip())
        except ValueError:
            self.status.configure(text="Invalid Port", text_color="tomato")
            return

        broker = self.broker.get().strip()
        if not broker:
            self.status.configure(text="Broker required", text_color="tomato")
            return

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect    = self._on_connect
        self.mqtt_client.on_message    = self._on_message
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.connect_async(broker, port, keepalive=60)
        self.mqtt_client.loop_start()
        self.status.configure(text="● Connecting…", text_color="gold")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            for t_ent in (self.topic1, self.topic2, self.topic3):
                t = t_ent.get().strip()
                if t:
                    client.subscribe(t, qos=1)
            self.connect_btn.configure(text="DISCONNECT")
            self.status.configure(text="● Connected", text_color="#66BB6A")
        else:
            self.status.configure(text=f"● MQTT rc={rc}", text_color="tomato")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.connect_btn.configure(text="CONNECT")
        self.status.configure(text="● Disconnected", text_color="tomato")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8", errors="ignore"))
        except json.JSONDecodeError:
            return

        t1 = self.topic1.get().strip()
        t2 = self.topic2.get().strip()
        t3 = self.topic3.get().strip()
        jd = payload.get("joint_deg", [])

        if msg.topic == t1 and len(jd) >= 4:
            self.queue.put(("arm1", jd[:4]))
        elif msg.topic == t2 and len(jd) >= 4:
            self.queue.put(("arm2", jd[:4]))
        elif msg.topic == t3 and len(jd) >= 2:
            self.queue.put(("ejector", jd[:2]))

        # Also accept combined-payload format
        if "arm1_deg" in payload:
            self.queue.put(("arm1",    list(payload["arm1_deg"])[:4]))
        if "arm2_deg" in payload:
            self.queue.put(("arm2",    list(payload["arm2_deg"])[:4]))
        if "ejector_deg" in payload:
            self.queue.put(("ejector", list(payload["ejector_deg"])[:2]))

    def _process_queue(self):
        updated = False
        while not self.queue.empty():
            arm_id, joint_deg = self.queue.get()
            q_rad = np.radians(np.array(joint_deg, dtype=float))
            if arm_id == "arm1":
                self._q_arm1 = q_rad
            elif arm_id == "arm2":
                self._q_arm2 = q_rad
            elif arm_id == "ejector":
                self._q_ejector = q_rad
            updated = True
        if updated:
            self._draw_scene()
        self.after(80, self._process_queue)

    # ── Forward Kinematics ────────────────────────────────────────────────────

    def _fk_dobot_mg400(self, q):
        """MG400 geometric FK → (5,3) array: base-bottom, shoulder, elbow, wrist, ee-tip."""
        q1 = float(q[0]) if len(q) > 0 else 0.0
        q2 = float(q[1]) if len(q) > 1 else 0.0
        q3 = float(q[2]) if len(q) > 2 else 0.0

        hdx, hdy = cos(q1), sin(q1)
        shoulder  = np.array([0.0, 0.0, _MG_D1])
        elbow     = shoulder + np.array([_MG_A2 * cos(q2) * hdx,
                                          _MG_A2 * cos(q2) * hdy,
                                          _MG_A2 * sin(q2)])
        ab = q2 + q3
        wrist     = elbow   + np.array([_MG_A3 * cos(ab) * hdx,
                                         _MG_A3 * cos(ab) * hdy,
                                         _MG_A3 * sin(ab)])
        ee_tip    = wrist   + np.array([0.0, 0.0, -_MG_D4])
        return np.array([[0., 0., 0.], shoulder, elbow, wrist, ee_tip])

    def _fk_ejector_2dof(self, q):
        """2-DOF ejector FK → (4,3) array: base-bottom, shoulder, boom-tip, ee-tip."""
        q1 = float(q[0]) if len(q) > 0 else 0.0
        q2 = float(q[1]) if len(q) > 1 else 0.0

        c1, s1    = cos(q1), sin(q1)
        shoulder  = np.array([0.0, 0.0, _EJ_H])
        boom_tip  = shoulder + np.array([_EJ_L1 * cos(q2) * c1,
                                          _EJ_L1 * cos(q2) * s1,
                                          _EJ_L1 * sin(q2)])
        ee_tip    = boom_tip + np.array([0.0, 0.0, -_EJ_TOOL])
        return np.array([[0., 0., 0.], shoulder, boom_tip, ee_tip])

    # ── 3-D rendering helpers ─────────────────────────────────────────────────

    def _add_poly(self, faces, facecolor, edgecolor=None, alpha=1.0):
        if not faces:
            return
        pc = Poly3DCollection(faces, alpha=alpha)
        pc.set_facecolor(facecolor)
        pc.set_edgecolor(edgecolor or facecolor)
        self.ax.add_collection3d(pc)

    def _draw_sphere(self, center, radius, color):
        u = np.linspace(0, 2 * pi, 14)
        v = np.linspace(0, pi, 10)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
        self.ax.plot_surface(x, y, z, color=color, linewidth=0,
                              antialiased=True, shade=True)

    def _draw_dobot_robot(self, q, world_offset, label, arm_color):
        """Draw one MG400 robot with its base at world_offset."""
        pts = self._fk_dobot_mg400(q) + world_offset
        base, shoulder, elbow, wrist, ee = pts

        # 1. Base platform
        self._add_poly(_box_faces(world_offset + [0, 0, 0.030],
                                   0.130, 0.130, 0.060), "#607D8B", "#455A64")
        # 2. Column
        self._add_poly(_cylinder_faces(world_offset + [0, 0, 0.060],
                                        shoulder, radius=0.030), "#90A4AE", "#455A64")
        # 3. Shoulder joint
        self._draw_sphere(shoulder, 0.026, "#546E7A")
        # 4. Upper arm
        self._add_poly(_cylinder_faces(shoulder, elbow, radius=0.022),
                        arm_color, "#B0BEC5")
        # 5. Elbow joint
        self._draw_sphere(elbow, 0.022, "#546E7A")
        # 6. Forearm
        self._add_poly(_cylinder_faces(elbow, wrist, radius=0.017),
                        "#CFD8DC", "#90A4AE")
        # 7. Wrist joint
        self._draw_sphere(wrist, 0.020, "#546E7A")
        # 8. Tool shaft
        self._add_poly(_cylinder_faces(wrist, ee, radius=0.012),
                        "#37474F", "#263238")
        # 9. EE tip (red)
        self._draw_sphere(ee, 0.014, "#EF5350")

        # Robot label below base
        self.ax.text(world_offset[0], world_offset[1], -0.04,
                     label, ha="center", va="top", color=arm_color,
                     fontsize=9, fontweight="bold")

    def _draw_ejector_arm(self, q, world_offset, label):
        """Draw 2-DOF ejector arm with its base at world_offset."""
        pts = self._fk_ejector_2dof(q) + world_offset
        base, shoulder, boom_tip, ee = pts

        # 1. Base box (smaller than Dobot)
        self._add_poly(_box_faces(world_offset + [0, 0, 0.020],
                                   0.090, 0.090, 0.040), "#78909C", "#546E7A")
        # 2. Column
        self._add_poly(_cylinder_faces(world_offset + [0, 0, 0.040],
                                        shoulder, radius=0.020), "#90A4AE", "#546E7A")
        # 3. Shoulder joint
        self._draw_sphere(shoulder, 0.018, "#546E7A")
        # 4. Boom (orange to distinguish)
        self._add_poly(_cylinder_faces(shoulder, boom_tip, radius=0.015),
                        "#FF9800", "#E65100")
        # 5. Boom-tip joint
        self._draw_sphere(boom_tip, 0.015, "#546E7A")
        # 6. Tool shaft
        self._add_poly(_cylinder_faces(boom_tip, ee, radius=0.010),
                        "#37474F", "#263238")
        # 7. EE tip
        self._draw_sphere(ee, 0.012, "#EF5350")

        self.ax.text(world_offset[0], world_offset[1], -0.04,
                     label, ha="center", va="top", color="#FF9800",
                     fontsize=9, fontweight="bold")

    def _draw_belt_box(self, center, lx, ly, lz, color):
        """Draw a flat conveyor belt rectangle."""
        self._add_poly(_box_faces(center, lx, ly, lz), color,
                        edgecolor="#1A1A2A", alpha=0.75)

    # ── Master scene draw ─────────────────────────────────────────────────────

    def _draw_scene(self):
        try:
            elev, azim = self.ax.elev, self.ax.azim
        except Exception:
            elev, azim = 28, -58

        self.ax.clear()
        self.ax.set_facecolor("#12122A")
        for pane in (self.ax.xaxis.pane, self.ax.yaxis.pane, self.ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor("#2A2A4A")

        # Ground grid
        for gv in np.linspace(-1.05, 1.05, 12):
            self.ax.plot([-1.05, 1.05], [gv, gv], [0, 0], color="#1E1E3A", lw=0.6)
            self.ax.plot([gv, gv], [-0.65, 0.65], [0, 0], color="#1E1E3A", lw=0.6)

        # Conveyor Belt 1 — Arm1 → Ejector (dark green)
        self._draw_belt_box(
            [(_BASE_ARM1[0] + _BASE_EJ[0]) / 2,
             (_BASE_ARM1[1] + _BASE_EJ[1]) / 2,
             0.012],
            abs(_BASE_EJ[0] - _BASE_ARM1[0]) - 0.14,  # length
            0.09, 0.014, "#1B5E20"
        )
        # Conveyor Belt 2 — Arm2 → exit (dark blue)
        self._draw_belt_box(
            [_BASE_ARM2[0] + 0.30, _BASE_ARM2[1], 0.012],
            0.52, 0.09, 0.014, "#0D47A1"
        )

        # ── Draw all 3 robots ──────────────────────────────────────────────
        self._draw_dobot_robot(self._q_arm1, _BASE_ARM1,
                                "Arm 1 (MG400)", "#42A5F5")
        self._draw_dobot_robot(self._q_arm2, _BASE_ARM2,
                                "Arm 2 (MG400)", "#66BB6A")
        self._draw_ejector_arm(self._q_ejector, _BASE_EJ,
                                "Ejector (2-DOF)")

        # ── Axes & style ───────────────────────────────────────────────────
        self.ax.set_xlim(-1.10, 1.10)
        self.ax.set_ylim(-0.65, 0.65)
        self.ax.set_zlim(0.0, 0.55)
        self.ax.set_xlabel("X (m)", color="#90A4AE", labelpad=6)
        self.ax.set_ylabel("Y (m)", color="#90A4AE", labelpad=6)
        self.ax.set_zlabel("Z (m)", color="#90A4AE", labelpad=6)
        self.ax.tick_params(colors="#607D8B", labelsize=8)
        self.ax.set_title(
            "Digital Twin — 2× Dobot MG400  +  Ejector 2-DOF",
            fontsize=12, fontweight="bold", color="#ECEFF1", pad=10,
        )
        self.ax.view_init(elev=elev, azim=azim)

        # ── EE readouts ────────────────────────────────────────────────────
        ee1 = self._fk_dobot_mg400(self._q_arm1)[-1] + _BASE_ARM1
        ee2 = self._fk_dobot_mg400(self._q_arm2)[-1] + _BASE_ARM2
        ee3 = self._fk_ejector_2dof(self._q_ejector)[-1] + _BASE_EJ

        self._ee1_lbl.configure(
            text=(f"Arm 1 (MG400):  "
                  f"x={ee1[0]*1000:+7.1f}  y={ee1[1]*1000:+7.1f}  z={ee1[2]*1000:+7.1f} mm")
        )
        self._ee2_lbl.configure(
            text=(f"Arm 2 (MG400):  "
                  f"x={ee2[0]*1000:+7.1f}  y={ee2[1]*1000:+7.1f}  z={ee2[2]*1000:+7.1f} mm")
        )
        self._ee3_lbl.configure(
            text=(f"Ejector (2-DOF):  "
                  f"x={ee3[0]*1000:+7.1f}  y={ee3[1]*1000:+7.1f}  z={ee3[2]*1000:+7.1f} mm")
        )
        self.canvas.draw_idle()


if __name__ == "__main__":
    app = DigitalTwin5DOF()
    app.mainloop()

