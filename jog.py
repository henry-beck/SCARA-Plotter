"""SCARA Plotter - Jog Slice v1 desktop controller (Tkinter + pyserial)."""
import math
import tkinter as tk
from tkinter import filedialog, ttk

import serial

from kinematics import ScaraArm
from svg_import import anchor_bbox_transform, fit_bbox_transform, load_svg_polylines

# ---------------- Config ----------------
SERIAL_PORT = "COM3"  # Windows example. macOS: `ls /dev/cu.*`  Linux: `ls /dev/ttyACM*`
BAUD = 115200
JOG_STEPS = 200
CONNECT_DELAY = 2.0  # seconds to wait after opening the port (Arduino auto-reset)
READ_TIMEOUT = 2.0   # seconds; bounds every readline() so a silent Arduino can't hang us
HOME_TIMEOUT = 30.0  # seconds; homing replies can take far longer than a jog
MOVE_TIMEOUT = 15.0  # seconds; a full-workspace MOVE takes longer than a jog too

# Mirrors firmware A_HOME_ANGLE_RAD / B_HOME_ANGLE_RAD - keep these two in sync
# by hand whenever the firmware constants change.
A_HOME_ANGLE_RAD = 0.0
B_HOME_ANGLE_RAD = 0.0

# Mirrors firmware A_MIN_RAD/A_MAX_RAD/B_MIN_RAD/B_MAX_RAD - used to discard IK
# solutions the firmware would reject anyway. Keep in sync with the firmware.
A_MIN_RAD = -3.141592653589793
A_MAX_RAD = 3.141592653589793
B_MIN_RAD = -3.141592653589793
B_MAX_RAD = 3.141592653589793

# TODO: measure your arm's real link lengths (mm) - these are placeholders.
L1 = 150.0
L2 = 150.0

CANVAS_SIZE = 500          # pixels, square canvas the SVG is rendered into
WORKSPACE_VIEW_SIZE = 400  # pixels, square canvas for the physical workspace preview

# The physical rectangle (mm) the loaded image is scaled to fit (size only - see
# ANCHOR_OPTIONS/"Set Origin" for where it's positioned): (x_min, y_min, width,
# height). TODO: calibrate this by trial to a rectangle your arm can actually reach.
DRAW_AREA_MM = (50.0, -100.0, 200.0, 200.0)

# Which point of the scaled SVG bounding box lands on the calibrated origin.
# (u, v) fractions of the bbox: u 0->1 is left->right, v 0->1 is bottom->top.
ANCHOR_OPTIONS = {
    "Bottom-left": (0.0, 0.0),
    "Bottom-right": (1.0, 0.0),
    "Top-left": (0.0, 1.0),
    "Top-right": (1.0, 1.0),
    "Center": (0.5, 0.5),
}
# -----------------------------------------


class JogApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Jog Controller")
        self.ser = None
        self.current_angles = [None, None]  # [theta1, theta2] rad; set by homing/MOVE
        self.arm = ScaraArm(L1, L2)

        self.svg_polylines = []
        self.svg_bbox = None
        self.canvas_to_svg = None    # canvas pixel -> SVG user-space
        self.svg_to_workspace = None  # SVG user-space -> workspace mm (calibrated)
        self.origin_xy = (DRAW_AREA_MM[0], DRAW_AREA_MM[1])  # default: frame's bottom-left

        conn_frame = ttk.Frame(root, padding=8)
        conn_frame.grid(row=0, column=0, sticky="ew")

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=(0, 4))
        self.port_var = tk.StringVar(value=SERIAL_PORT)
        ttk.Entry(conn_frame, textvariable=self.port_var, width=20).grid(row=0, column=1)
        ttk.Button(conn_frame, text="Connect", command=self.connect).grid(row=0, column=2, padx=4)
        ttk.Button(conn_frame, text="Ping", command=self.ping).grid(row=0, column=3, padx=4)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn_frame, textvariable=self.status_var).grid(row=0, column=4, padx=8)

        self.busy_buttons = []

        jog_frame = ttk.Frame(root, padding=8)
        jog_frame.grid(row=1, column=0)

        jog_specs = [
            ("Jog A +", 0, 0, "A", "+"),
            ("Jog A −", 0, 1, "A", "-"),
            ("Jog B +", 1, 0, "B", "+"),
            ("Jog B −", 1, 1, "B", "-"),
        ]
        for text, r, c, axis, direction in jog_specs:
            btn = ttk.Button(jog_frame, text=text, width=12,
                              command=lambda a=axis, d=direction: self.jog(a, d))
            btn.grid(row=r, column=c, padx=4, pady=4)
            self.busy_buttons.append(btn)

        home_frame = ttk.Frame(root, padding=8)
        home_frame.grid(row=2, column=0)

        home_specs = [
            ("Home A", 0, "A"),
            ("Home B", 1, "B"),
        ]
        for text, c, axis in home_specs:
            btn = ttk.Button(home_frame, text=text, width=12,
                              command=lambda a=axis: self.home(a))
            btn.grid(row=0, column=c, padx=4, pady=4)
            self.busy_buttons.append(btn)

        home_all_btn = ttk.Button(home_frame, text="Home All", width=12,
                                   command=self.home_all)
        home_all_btn.grid(row=0, column=2, padx=4, pady=4)
        self.busy_buttons.append(home_all_btn)

        move_frame = ttk.Frame(root, padding=8)
        move_frame.grid(row=3, column=0)

        ttk.Label(move_frame, text="θ1 (deg):").grid(row=0, column=0, padx=(0, 4))
        self.theta1_var = tk.StringVar(value="0.0")
        ttk.Entry(move_frame, textvariable=self.theta1_var, width=10).grid(row=0, column=1, padx=4)

        ttk.Label(move_frame, text="θ2 (deg):").grid(row=0, column=2, padx=(8, 4))
        self.theta2_var = tk.StringVar(value="0.0")
        ttk.Entry(move_frame, textvariable=self.theta2_var, width=10).grid(row=0, column=3, padx=4)

        go_btn = ttk.Button(move_frame, text="Go", width=8, command=self.move_to_angles)
        go_btn.grid(row=0, column=4, padx=(8, 0))
        self.busy_buttons.append(go_btn)

        point_frame = ttk.Frame(root, padding=8)
        point_frame.grid(row=4, column=0)

        ttk.Label(point_frame, text="x (mm):").grid(row=0, column=0, padx=(0, 4))
        self.x_var = tk.StringVar(value="0.0")
        ttk.Entry(point_frame, textvariable=self.x_var, width=10).grid(row=0, column=1, padx=4)

        ttk.Label(point_frame, text="y (mm):").grid(row=0, column=2, padx=(8, 4))
        self.y_var = tk.StringVar(value="0.0")
        ttk.Entry(point_frame, textvariable=self.y_var, width=10).grid(row=0, column=3, padx=4)

        point_go_btn = ttk.Button(point_frame, text="Go to point", width=12,
                                   command=self.go_to_point)
        point_go_btn.grid(row=0, column=4, padx=(8, 0))
        self.busy_buttons.append(point_go_btn)

        svg_frame = ttk.Frame(root, padding=8)
        svg_frame.grid(row=5, column=0, sticky="w")
        ttk.Button(svg_frame, text="Load SVG", command=self.load_svg).grid(
            row=0, column=0, padx=(0, 8))

        ttk.Label(svg_frame, text="Anchor:").grid(row=0, column=1, padx=(0, 4))
        self.anchor_var = tk.StringVar(value="Bottom-left")
        anchor_combo = ttk.Combobox(svg_frame, textvariable=self.anchor_var, state="readonly",
                                     width=12, values=list(ANCHOR_OPTIONS.keys()))
        anchor_combo.grid(row=0, column=2, padx=(0, 8))
        anchor_combo.bind("<<ComboboxSelected>>", self.on_anchor_change)

        ttk.Button(svg_frame, text="Set Origin", command=self.set_origin).grid(
            row=0, column=3, padx=(0, 8))

        draw_svg_btn = ttk.Button(svg_frame, text="Draw SVG", command=self.draw_svg)
        draw_svg_btn.grid(row=0, column=4)
        self.busy_buttons.append(draw_svg_btn)

        canvas_frame = ttk.Frame(root, padding=8)
        canvas_frame.grid(row=6, column=0)
        ttk.Label(canvas_frame, text="SVG preview (click to move)").grid(row=0, column=0)
        self.canvas = tk.Canvas(canvas_frame, width=CANVAS_SIZE, height=CANVAS_SIZE,
                                 bg="white", highlightthickness=1, highlightbackground="gray")
        self.canvas.grid(row=1, column=0)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        workspace_frame = ttk.Frame(root, padding=8)
        workspace_frame.grid(row=7, column=0)
        ttk.Label(workspace_frame,
                  text="Workspace preview (mm): drawing frame + arm links").grid(row=0, column=0)
        self.workspace_canvas = tk.Canvas(workspace_frame, width=WORKSPACE_VIEW_SIZE,
                                           height=WORKSPACE_VIEW_SIZE, bg="white",
                                           highlightthickness=1, highlightbackground="gray")
        self.workspace_canvas.grid(row=1, column=0)

        log_frame = ttk.Frame(root, padding=8)
        log_frame.grid(row=8, column=0, sticky="nsew")
        root.grid_rowconfigure(8, weight=1)
        root.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, width=50, height=16, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text["yscrollcommand"] = scrollbar.set

        self.render_workspace_view()

    def log(self, line):
        self.log_text["state"] = "normal"
        self.log_text.insert("end", line + "\n")
        self.log_text["state"] = "disabled"
        self.log_text.see("end")

    def connect(self):
        port = self.port_var.get().strip()

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        try:
            self.ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT)
        except serial.SerialException as e:
            self.ser = None
            self.status_var.set("Disconnected")
            self.log(f"! Failed to open {port}: {e}")
            return

        self.status_var.set(f"Connecting to {port}...")
        self.root.update_idletasks()
        self.root.after(int(CONNECT_DELAY * 1000), self._finish_connect)

    def _finish_connect(self):
        try:
            line = self.ser.readline().decode(errors="replace").strip()
        except Exception as e:
            self.log(f"! Error reading from port: {e}")
            self.status_var.set("Disconnected")
            return

        if line == "READY":
            self.log(f"< {line}")
            self.status_var.set("Connected — READY")
        elif line:
            self.log(f"< {line}")
            self.status_var.set(f"Connected — unexpected reply: {line}")
        else:
            self.log("! No READY line received (timed out)")
            self.status_var.set("Connected — no READY")

    def send(self, cmd, timeout=None):
        if self.ser is None or not self.ser.is_open:
            self.log("! Not connected — connect first")
            return None

        self.log(f"> {cmd}")
        original_timeout = self.ser.timeout
        try:
            if timeout is not None:
                self.ser.timeout = timeout
            self.ser.write((cmd + "\n").encode())
            reply = self.ser.readline().decode(errors="replace").strip()
        except Exception as e:
            self.log(f"! Serial error: {e}")
            self.status_var.set("Disconnected")
            return None
        finally:
            self.ser.timeout = original_timeout

        if reply:
            self.log(f"< {reply}")
        else:
            self.log("! No reply (timed out)")
        return reply

    def set_buttons_enabled(self, enabled):
        for btn in self.busy_buttons:
            btn.state(["!disabled"] if enabled else ["disabled"])

    def ping(self):
        self.send("PING")

    def jog(self, axis, direction):
        self.send(f"JOG {axis} {direction} {JOG_STEPS}")

    def home(self, axis):
        if self.ser is None or not self.ser.is_open:
            self.log("! Not connected — connect first")
            return None

        self.status_var.set(f"Homing {axis}…")
        self.root.update_idletasks()

        self.set_buttons_enabled(False)
        try:
            reply = self.send(f"HOME {axis}", timeout=HOME_TIMEOUT)
        finally:
            self.set_buttons_enabled(True)

        if reply == "OK":
            self.status_var.set("Connected — READY")
            if axis == "A":
                self.current_angles[0] = A_HOME_ANGLE_RAD
            elif axis == "B":
                self.current_angles[1] = B_HOME_ANGLE_RAD
            self.render_workspace_view()
        elif reply:
            self.status_var.set(f"Home {axis} failed: {reply}")
        else:
            self.status_var.set(f"Home {axis}: no reply")

        return reply

    def home_all(self):
        if self.home("A") == "OK":
            self.home("B")

    def move_to_angles(self):
        try:
            theta1 = math.radians(float(self.theta1_var.get()))
            theta2 = math.radians(float(self.theta2_var.get()))
        except ValueError:
            self.log("! Invalid angle input — enter numbers in degrees")
            return

        if self.ser is None or not self.ser.is_open:
            self.log("! Not connected — connect first")
            return

        self._move_to(theta1, theta2)

    def go_to_point(self):
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
        except ValueError:
            self.log("! Invalid coordinate input — enter numbers in mm")
            return

        self._go_to_xy(x, y)

    def _go_to_xy(self, x, y):
        if self.ser is None or not self.ser.is_open:
            self.log("! Not connected — connect first")
            return

        solutions = self.arm.ik(x, y)
        if not solutions:
            self.log(f"! ({x}, {y}) is out of reach")
            self.status_var.set(f"Out of reach: ({x}, {y})")
            return

        valid = [(t1, t2) for t1, t2 in solutions
                 if A_MIN_RAD <= t1 <= A_MAX_RAD and B_MIN_RAD <= t2 <= B_MAX_RAD]
        if not valid:
            self.log(f"! ({x}, {y}) is reachable but violates joint limits")
            self.status_var.set(f"No valid elbow solution for ({x}, {y})")
            return

        cur1, cur2 = self.current_angles
        if cur1 is None or cur2 is None:
            theta1, theta2 = valid[0]  # no known pose yet (never homed) - just take one
        else:
            theta1, theta2 = min(valid, key=lambda sol: abs(sol[0] - cur1) + abs(sol[1] - cur2))

        return self._move_to(theta1, theta2)

    def load_svg(self):
        path = filedialog.askopenfilename(filetypes=[("SVG files", "*.svg")])
        if not path:
            return

        try:
            polylines, bbox = load_svg_polylines(path)
        except Exception as e:
            self.log(f"! Failed to load SVG: {e}")
            return

        self.svg_polylines = polylines
        self.svg_bbox = bbox
        svg_to_canvas, self.canvas_to_svg = fit_bbox_transform(
            bbox, 0, 0, CANVAS_SIZE, CANVAS_SIZE, flip_y=False)
        self.update_svg_to_workspace()

        self.render_svg(svg_to_canvas)
        self.render_workspace_view()
        self.log(f"Loaded {path} ({len(polylines)} polylines)")

    def update_svg_to_workspace(self):
        if self.svg_bbox is None:
            return
        _, _, draw_w, draw_h = DRAW_AREA_MM
        anchor_frac = ANCHOR_OPTIONS[self.anchor_var.get()]
        self.svg_to_workspace = anchor_bbox_transform(
            self.svg_bbox, draw_w, draw_h, self.origin_xy, anchor_frac)

    def on_anchor_change(self, event=None):
        self.update_svg_to_workspace()
        self.render_workspace_view()

    def set_origin(self):
        cur1, cur2 = self.current_angles
        if cur1 is None or cur2 is None:
            self.log("! Home the arm first — current pose is unknown")
            return

        self.origin_xy = self.arm.fk(cur1, cur2)
        self.update_svg_to_workspace()
        self.render_workspace_view()
        self.log(f"Origin set to ({self.origin_xy[0]:.1f}, {self.origin_xy[1]:.1f}) mm")

    def draw_svg(self):
        if not self.svg_polylines:
            self.log("! Load an SVG first")
            return
        if self.ser is None or not self.ser.is_open:
            self.log("! Not connected — connect first")
            return

        points = [self.svg_to_workspace(x, y)
                  for polyline in self.svg_polylines for x, y in polyline]
        total = len(points)
        self.log(f"Drawing SVG: {total} points - no pen lift, shapes will be "
                  f"connected by travel lines")

        for i, (x, y) in enumerate(points, start=1):
            self.status_var.set(f"Drawing point {i}/{total}")
            self.root.update_idletasks()
            reply = self._go_to_xy(x, y)
            if reply != "OK":
                self.log(f"! Drawing aborted at point {i}/{total}: "
                         f"{reply or 'out of reach / no reply'}")
                return

        self.status_var.set("Drawing complete")
        self.log("Drawing complete")

    def workspace_view_bounds(self):
        reach = self.arm.l1 + self.arm.l2
        dx, dy, dw, dh = DRAW_AREA_MM
        xmin = min(-reach, dx)
        xmax = max(reach, dx + dw)
        ymin = min(-reach, dy)
        ymax = max(reach, dy + dh)
        return (xmin, ymin, xmax, ymax)

    def render_workspace_view(self):
        self.workspace_canvas.delete("all")
        bounds = self.workspace_view_bounds()
        workspace_to_view, _ = fit_bbox_transform(
            bounds, 0, 0, WORKSPACE_VIEW_SIZE, WORKSPACE_VIEW_SIZE, flip_y=True)

        dx, dy, dw, dh = DRAW_AREA_MM
        frame_corners = [(dx, dy), (dx + dw, dy), (dx + dw, dy + dh), (dx, dy + dh), (dx, dy)]
        flat = []
        for x, y in frame_corners:
            vx, vy = workspace_to_view(x, y)
            flat.extend([vx, vy])
        self.workspace_canvas.create_line(*flat, fill="blue", width=1, dash=(4, 2))

        if self.svg_to_workspace is not None:
            for polyline in self.svg_polylines:
                flat = []
                for svg_x, svg_y in polyline:
                    wx, wy = self.svg_to_workspace(svg_x, svg_y)
                    vx, vy = workspace_to_view(wx, wy)
                    flat.extend([vx, vy])
                if len(flat) >= 4:
                    self.workspace_canvas.create_line(*flat, fill="black", width=1)

        ox, oy = self.origin_xy
        ovx, ovy = workspace_to_view(ox, oy)
        r = 4
        self.workspace_canvas.create_oval(ovx - r, ovy - r, ovx + r, ovy + r,
                                           outline="red", width=2)

        cur1, cur2 = self.current_angles
        if cur1 is not None and cur2 is not None:
            elbow = (self.arm.l1 * math.cos(cur1), self.arm.l1 * math.sin(cur1))
            tip = self.arm.fk(cur1, cur2)
            bx, by = workspace_to_view(0.0, 0.0)
            ex, ey = workspace_to_view(*elbow)
            tx, ty = workspace_to_view(*tip)
            self.workspace_canvas.create_line(bx, by, ex, ey, fill="green", width=3)
            self.workspace_canvas.create_line(ex, ey, tx, ty, fill="orange", width=3)

    def render_svg(self, svg_to_canvas):
        self.canvas.delete("all")
        for polyline in self.svg_polylines:
            flat = []
            for x, y in polyline:
                cx, cy = svg_to_canvas(x, y)
                flat.extend([cx, cy])
            if len(flat) >= 4:
                self.canvas.create_line(*flat, fill="black", width=1)

    def on_canvas_click(self, event):
        if self.canvas_to_svg is None:
            self.log("! Load an SVG first")
            return

        svg_x, svg_y = self.canvas_to_svg(event.x, event.y)
        wx, wy = self.svg_to_workspace(svg_x, svg_y)
        self.log(f"Click canvas ({event.x}, {event.y}) -> workspace ({wx:.1f}, {wy:.1f}) mm")
        self._go_to_xy(wx, wy)

    def _move_to(self, theta1, theta2):
        self.set_buttons_enabled(False)
        try:
            reply = self.send(f"MOVE {theta1} {theta2}", timeout=MOVE_TIMEOUT)
        finally:
            self.set_buttons_enabled(True)

        if reply == "OK":
            self.current_angles = [theta1, theta2]
            self.render_workspace_view()
        elif reply:
            self.status_var.set(f"Move failed: {reply}")
        else:
            self.status_var.set("Move: no reply")

        return reply


if __name__ == "__main__":
    root = tk.Tk()
    JogApp(root)
    root.mainloop()
