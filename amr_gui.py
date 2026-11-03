from __future__ import annotations

import json
import math
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402
from matplotlib.colors import to_rgba  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

from amr_client import (DEFAULT_HOST, AMRClient, ChannelSilentError, check_robot,  # noqa: E402
                        find_robots, local_subnets, normalize_host, recent_hosts,
                        remember_host, robot_to_map)

APP_TITLE = "AMR Agent"

# =============================================================================
# Look and feel (colours taken from the robot's web app)
# =============================================================================

FONT = "Segoe UI"
BG = "#f9f9fa"            # page and map background
CARD_BORDER = "#c9c9cc"
LINE = "#e6e6e8"
TEXT = "#282828"
MUTED = "#4b4b4b"
ACCENT = "#449d6a"        # active buttons
ACCENT_DARK = "#3a8a5c"
LIVE_FILL, LIVE_RING = "#42eb42", "#04cd68"

ICON_RED, ICON_YELLOW = "#e53935", "#dfaf08"
ICON_GREEN, ICON_BLUE = "#31eb68", "#5c5cfd"

MAP_POINTS = "#707070"
LIDAR = "#0066C5"
SCAN_2 = "red"
FOOTPRINT = "orange"
LOCAL_PATH = "blue"
GLOBAL_PATH = "#e8c800"   # the web app uses yellow; slightly darker to show on light grey
PATH = "#dd6023"
PATH_ARROW = "#FFD700"
STATION_EDGE, STATION_FILL, STATION_TEXT = "#b9f20c", "#013399", "#6a6a6a"
REFLECTOR = "#32a852"
AREA_COLORS = {"AdvancedArea": "#0066C5", "ForbiddenArea": "red", "LocalizationArea": "#FDD394"}
ROBOT_BODY, ROBOT_FRONT, ROBOT_PANEL = "#111111", "#f7e400", "#3a3a3a"

PREDICTED_FOOTPRINT = (1.14, 0.85)   # length, width (web app default)
REFRESH_MS = 200                     # how often the window updates
STALE_S = {"pose": 2, "lidar": 2, "scan2": 2, "footprint": 3, "local_path": 3,
           "global_path": 10, "errors": 5, "status": 4}

# Layers that can be switched on and off: (key, label)
LAYERS = [
    ("map", "Map points"), ("stations", "Stations"), ("paths", "Paths"),
    ("areas", "Areas"), ("reflectors", "Reflectors"), ("lidar", "Lidar"),
    ("scan2", "Second scan"), ("robot", "Robot"), ("footprint", "Footprint"),
    ("local_path", "Local path"), ("predicted", "Predicted footprints"),
    ("global_path", "Global path"),
]


# =============================================================================
# Geometry
# =============================================================================

def transform(points, x: float, y: float, angle: float) -> list:
    """Rotate points by `angle` and move them to (x, y)."""
    c, s = math.cos(angle), math.sin(angle)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in points]


def rectangle(length: float, width: float) -> list:
    hl, hw = length / 2, width / 2
    return [(-hl, hw), (hl, hw), (hl, -hw), (-hl, -hw)]


def robot_shapes(length: float, width: float):
    """Robot outline in robot coordinates (front = +x): dark body, yellow front, panel."""
    L, W, c = length, width, 0.1 * length
    xf = -L / 2 + 0.8 * L
    body = [(-L / 2 + c, W / 2), (xf, W / 2), (xf, -W / 2), (-L / 2 + 0.3 * c, -W / 2),
            (-L / 2, -W / 2 + 0.3 * c), (-L / 2, W / 2 - c)]
    front = [(xf, W / 2), (L / 2 - c, W / 2), (L / 2, W / 2 - c), (L / 2, -W / 2 + c),
             (L / 2 - c, -W / 2), (xf, -W / 2)]
    p0, p1 = L / 2 - 0.1 * L, L / 2 - 0.025 * L
    panel = [(p0, 0.22 * W), (p1, 0.22 * W), (p1, -0.22 * W), (p0, -0.22 * W)]
    return body, front, panel


def station_shapes(w: float, h: float, border: float = 0.03):
    """Station marker like the web app: framed rectangle, direction arrow, centre dot."""
    outer = rectangle(w + 2 * border, h + 2 * border)
    inner = rectangle(w, h)
    arrow = [(w / 4, -h / 5), (w / 4, h / 5), (w / 2, 0)]
    dot = [(w / 20 * math.cos(a), w / 20 * math.sin(a))
           for a in (i * 2 * math.pi / 12 for i in range(12))]
    return [(outer, STATION_EDGE), (inner, STATION_FILL), (arrow, STATION_EDGE), (dot, STATION_EDGE)]


def bezier_points(p0, p1, p2, p3, n=40):
    out = []
    for i in range(n + 1):
        t = i / n
        w = ((1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t ** 3)
        out.append((sum(wi * p["x"] for wi, p in zip(w, (p0, p1, p2, p3))),
                    sum(wi * p["y"] for wi, p in zip(w, (p0, p1, p2, p3)))))
    return out


def arc_points(p0, p1, p2, n=40):
    """Circular arc from p0 through p1 to p2."""
    ax, ay, bx, by, cx, cy = p0["x"], p0["y"], p1["x"], p1["y"], p2["x"], p2["y"]
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [(ax, ay), (cx, cy)]
    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    a0, a1, a2 = (math.atan2(y - uy, x - ux) for x, y in ((ax, ay), (bx, by), (cx, cy)))
    sweep = (a2 - a0) % (2 * math.pi)
    if (a1 - a0) % (2 * math.pi) > sweep:
        sweep -= 2 * math.pi
    return [(ux + r * math.cos(a0 + sweep * i / n), uy + r * math.sin(a0 + sweep * i / n))
            for i in range(n + 1)]


def bspline_points(ctrl, degree, knots, n=60):
    def basis(i, k, t):
        if k == 0:
            inside = knots[i] <= t < knots[i + 1]
            return 1.0 if inside or (t == knots[-1] and knots[i] < t <= knots[i + 1]) else 0.0
        left = 0.0 if knots[i + k] == knots[i] else \
            (t - knots[i]) / (knots[i + k] - knots[i]) * basis(i, k - 1, t)
        right = 0.0 if knots[i + k + 1] == knots[i + 1] else \
            (knots[i + k + 1] - t) / (knots[i + k + 1] - knots[i + 1]) * basis(i + 1, k - 1, t)
        return left + right
    out = []
    for s in range(n + 1):
        w = [basis(i, degree, s / n) for i in range(len(ctrl))]
        out.append((sum(wi * p["x"] for wi, p in zip(w, ctrl)), sum(wi * p["y"] for wi, p in zip(w, ctrl))))
    return out


def path_points(path: dict, params: dict) -> list:
    """Points along a map path (BezierPath, ArcPath, NURBS6 or StraightLine)."""
    start, end = path["startPos"]["pos"], path["endPos"]["pos"]
    kind = path.get("className")
    try:
        if kind == "BezierPath":
            return bezier_points(start, path["controlPos1"], path["controlPos2"], end)
        if kind == "ArcPath":
            return arc_points(start, path["controlPos1"], end)
        if kind == "NURBS6":
            ctrl = [start] + [path[f"controlPos{i}"] for i in range(1, 5)] + [end]
            return bspline_points(ctrl, params.get("degree", 3),
                                  params.get("knots", [0, 0, 0, 0, .4, .6, 1, 1, 1, 1]))
    except (KeyError, TypeError, ZeroDivisionError):
        pass
    return [(start["x"], start["y"]), (end["x"], end["y"])]


def arrow_at_middle(points: list, size: float = 0.18) -> list:
    """Small triangle at the middle of a path, pointing along it."""
    i = len(points) // 2
    (x0, y0), (x1, y1) = points[max(i - 1, 0)], points[min(i + 1, len(points) - 1)]
    a = math.atan2(y1 - y0, x1 - x0)
    mx, my = points[i]
    return transform([(size, 0), (-size * 0.7, size * 0.6), (-size * 0.7, -size * 0.6)], mx, my, a)


# =============================================================================
# Background data from the robot
# =============================================================================

class LiveData:
    """Follows the robot's live channels in background threads.

    The window reads the latest values with snapshot(); no Tk calls happen
    in these threads.
    """

    CHANNELS = {"status": "robot_status", "pose": "robot_pose", "lidar": "lidar",
                "scan2": "scan_2", "footprint": "footprint", "local_path": "local_path",
                "global_path": "global_path", "errors": "errors_warnings"}

    def __init__(self, host: str):
        self.host = host
        self._lock = threading.Lock()
        self._values: dict = {}         # key -> (data, time)
        self._last_error = ""
        self._stop = threading.Event()
        self.version = 0                 # increases on every new message

    def start(self) -> None:
        for key, channel in self.CHANNELS.items():
            threading.Thread(target=self._follow, args=(key, channel), daemon=True).start()

    def stop(self) -> None:
        """Ask all threads to finish (they exit within a few seconds)."""
        self._stop.set()

    def _follow(self, key: str, channel: str) -> None:
        client = AMRClient(self.host, timeout=5)
        while not self._stop.is_set():
            try:
                for data in client.stream(channel, timeout=5):
                    if self._stop.is_set():
                        return
                    with self._lock:
                        self._values[key] = (data, time.time())
                        self.version += 1
            except ChannelSilentError:
                continue
            except Exception as e:
                with self._lock:
                    self._last_error = str(e)
                self._stop.wait(2)

    def snapshot(self) -> dict:
        """Fresh values only, plus 'online' and 'error'."""
        now = time.time()
        with self._lock:
            snap = {k: d for k, (d, t) in self._values.items() if now - t < STALE_S.get(k, 3)}
            snap["online"] = "status" in snap
            snap["error"] = self._last_error
            snap["version"] = self.version
        return snap


def run_in_background(results: queue.Queue, tag: str, func, *args) -> None:
    """Run func(*args) in a thread and put (tag, ok, result) on the queue."""
    def work():
        try:
            results.put((tag, True, func(*args)))
        except Exception as e:
            results.put((tag, False, e))
    threading.Thread(target=work, daemon=True).start()


# =============================================================================
# Small widgets
# =============================================================================

class Tooltip:
    def __init__(self, widget, text_func):
        self.widget, self.text_func, self.tip = widget, text_func, None
        widget.bind("<Enter>", self.show, add="+")
        widget.bind("<Leave>", self.hide, add="+")

    def show(self, _event=None):
        text = self.text_func() if callable(self.text_func) else self.text_func
        if not text:
            return
        x = self.widget.winfo_rootx() + 10
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 6
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.wm_geometry(f"+{x}+{y}")
        tk.Label(self.tip, text=text, bg="#333333", fg="white", font=(FONT, 9),
                 padx=8, pady=4, justify="left").pack()

    def hide(self, _event=None):
        if self.tip:
            self.tip.destroy()
            self.tip = None


def card(parent, **kw) -> tk.Frame:
    """A light panel with a thin border, like the web app's floating boxes."""
    return tk.Frame(parent, bg=BG, highlightthickness=1, highlightbackground=CARD_BORDER, **kw)


def plain_button(parent, text, command) -> tk.Button:
    return tk.Button(parent, text=text, command=command, bg="white", fg=TEXT,
                     activebackground="#eeeeee", relief="flat", bd=0, padx=14, pady=6,
                     font=(FONT, 10), cursor="hand2", highlightthickness=1,
                     highlightbackground=CARD_BORDER)


def set_window_icon(win) -> None:
    """Small robot icon instead of Tk's default feather."""
    icon = tk.PhotoImage(width=32, height=32)
    icon.put(BG, to=(0, 0, 32, 32))
    icon.put(ROBOT_BODY, to=(3, 7, 23, 25))
    icon.put(ROBOT_FRONT, to=(23, 7, 29, 25))
    win.iconphoto(True, icon)
    win._amr_icon = icon          # keep a reference so Tk doesn't drop it


def style_ttk(win) -> None:
    style = ttk.Style(win)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass
    style.configure("Treeview", background="white", fieldbackground="white", rowheight=26,
                    font=(FONT, 10))
    style.configure("Treeview.Heading", font=(FONT, 10, "bold"))
    style.map("Treeview", background=[("selected", "#d6ecdf")], foreground=[("selected", TEXT)])
    style.configure("TCombobox", padding=4)


def cancel_timers(win) -> None:
    """Cancel every pending after() timer of this Tk interpreter (before closing a window)."""
    try:
        for timer in win.tk.splitlist(win.tk.call("after", "info")):
            win.after_cancel(timer)
    except tk.TclError:
        pass


def center_window(win, over=None) -> None:
    """Place a window (at its natural size) in the middle of the screen or of another window."""
    win.update_idletasks()
    width, height = win.winfo_reqwidth(), win.winfo_reqheight()
    if over is not None:
        x = over.winfo_rootx() + (over.winfo_width() - width) // 2
        y = over.winfo_rooty() + (over.winfo_height() - height) // 3
    else:
        x = (win.winfo_screenwidth() - width) // 2
        y = (win.winfo_screenheight() - height) // 3
    win.geometry(f"+{max(x, 0)}+{max(y, 0)}")   # position only, so the window can grow


def green_button(parent, text, command) -> tk.Button:
    return tk.Button(parent, text=text, command=command, bg=ACCENT, fg="#edf2f6",
                     activebackground=ACCENT_DARK, activeforeground="white", relief="flat",
                     bd=0, padx=16, pady=6, font=(FONT, 10, "bold"), cursor="hand2")


class StatusIcon(tk.Canvas):
    """Vector status icons drawn like the web app's top bar."""

    def __init__(self, parent, kind: str):
        super().__init__(parent, width=26, height=26, bg=BG, highlightthickness=0)
        self.kind = kind

    def draw(self, color: str, level: float = 1.0) -> None:
        self.delete("all")
        k = self.kind
        if k == "error":
            self.create_polygon(13, 2, 25, 23, 1, 23, fill=color, outline=color)
            self.create_text(13, 16, text="!", fill="white", font=(FONT, 11, "bold"))
        elif k == "battery":
            self.create_rectangle(10, 2, 16, 5, fill=color, outline=color)
            self.create_rectangle(7, 5, 19, 24, outline=color, width=2)
            top = 23 - 17 * max(0.0, min(1.0, level))
            self.create_rectangle(9, top, 17, 22, fill=color, outline="")
        elif k == "wifi":
            lit = 1 + (level >= 0.25) + (level >= 0.75)
            for i, r in enumerate((5, 10, 15)):
                c = color if i < lit else "#d0d0d0"
                self.create_arc(13 - r, 23 - r, 13 + r, 23 + r, start=45, extent=90,
                                style="arc", outline=c, width=3)
        elif k == "estop":
            self.create_oval(3, 3, 23, 23, outline=color, width=2)
            self.create_rectangle(9, 9, 17, 17, fill=color, outline=color)
        elif k == "localization":
            self.create_oval(5, 5, 21, 21, outline=color, width=2)
            for x0, y0, x1, y1 in ((13, 1, 13, 6), (13, 20, 13, 25), (1, 13, 6, 13), (20, 13, 25, 13)):
                self.create_line(x0, y0, x1, y1, fill=color, width=2)
            self.create_oval(10, 10, 16, 16, fill=color, outline=color)


class ViewButton(tk.Canvas):
    """Round icon button for the map view options (pan, fit, zoom in, zoom out)."""

    def __init__(self, parent, kind: str, command, tooltip: str):
        super().__init__(parent, width=36, height=36, bg=BG, highlightthickness=0, cursor="hand2")
        self.kind, self.active = kind, False
        self.bind("<Button-1>", lambda _e: command())
        Tooltip(self, tooltip)
        self.draw()

    def set_active(self, active: bool) -> None:
        self.active = active
        self.draw()

    def draw(self) -> None:
        self.delete("all")
        fg = MUTED
        if self.active:
            self.create_oval(1, 1, 35, 35, fill=ACCENT, outline=ACCENT)
            fg = "#edf2f6"
        k = self.kind
        if k == "pan":
            self.create_line(18, 7, 18, 29, fill=fg, width=2)
            self.create_line(7, 18, 29, 18, fill=fg, width=2)
            for pts in ((18, 5, 14, 10, 22, 10), (18, 31, 14, 26, 22, 26),
                        (5, 18, 10, 14, 10, 22), (31, 18, 26, 14, 26, 22)):
                self.create_polygon(*pts, fill=fg, outline=fg)
        elif k == "fit":
            for x0, y0, dx, dy in ((7, 7, 1, 1), (29, 7, -1, 1), (7, 29, 1, -1), (29, 29, -1, -1)):
                self.create_line(x0, y0 + 6 * dy, x0, y0, x0 + 6 * dx, y0, fill=fg, width=2)
            self.create_rectangle(12, 14, 24, 22, fill=fg, outline=fg)
        elif k in ("zoom_in", "zoom_out"):
            self.create_oval(7, 7, 23, 23, outline=fg, width=2)
            self.create_line(21, 21, 29, 29, fill=fg, width=3)
            self.create_line(11, 15, 19, 15, fill=fg, width=2)
            if k == "zoom_in":
                self.create_line(15, 11, 15, 19, fill=fg, width=2)


# =============================================================================
# Map View page
# =============================================================================

class MapViewPage(tk.Frame):
    def __init__(self, parent, app: "AMRAgentApp"):
        super().__init__(parent, bg=BG)
        self.app = app
        self.params: dict = {}
        self.map: dict | None = None
        self.map_bounds = None
        self.static_artists: list = []
        self.layer_artists: dict = {k: [] for k, _ in LAYERS}
        self.layer_vars = {k: tk.BooleanVar(value=True) for k, _ in LAYERS}
        self.follow_var = tk.BooleanVar(value=False)
        self.pan_enabled = True
        self._drag = None
        self._drawn_version = -1
        self._dirty = True
        self.map_state = "loading"       # loading | ready | failed

        self._build_canvas()
        self._build_dynamic_layers()
        self._build_overlays()

    # ---------------------------------------------------------------- set-up
    def _build_canvas(self) -> None:
        self.fig = Figure(facecolor=BG)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_facecolor(BG)
        self.ax.set_axis_off()
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().configure(bg=BG, highlightthickness=0)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.canvas.mpl_connect("button_press_event", self._on_press)
        self.canvas.mpl_connect("button_release_event", self._on_release)
        self.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.canvas.mpl_connect("resize_event", lambda _e: self._set_view(*self._get_view()))

    def _build_dynamic_layers(self) -> None:
        ax = self.ax
        self.global_path_line, = ax.plot([], [], "o", ms=3.5, mew=0, color=GLOBAL_PATH, zorder=5)
        self.predicted = PolyCollection([], facecolors=(1, 0, 0, 0.25), edgecolors="none", zorder=6)
        ax.add_collection(self.predicted)
        self.lidar_line, = ax.plot([], [], "o", ms=2.2, mew=0, color=LIDAR, zorder=7)
        self.scan2_line, = ax.plot([], [], "o", ms=2.8, mew=0, color=SCAN_2, zorder=7)
        self.local_path_line, = ax.plot([], [], "o", ms=3, mew=0, color=LOCAL_PATH, zorder=8)
        self.robot_patches = [Polygon([[0, 0]], closed=True, color=c, zorder=9, visible=False)
                              for c in (ROBOT_BODY, ROBOT_FRONT, ROBOT_PANEL)]
        for p in self.robot_patches:
            ax.add_patch(p)
        self.footprint_line, = ax.plot([], [], "-", lw=2, color=FOOTPRINT, zorder=10)
        self.layer_artists["global_path"] += [self.global_path_line]
        self.layer_artists["predicted"] += [self.predicted]
        self.layer_artists["lidar"] += [self.lidar_line]
        self.layer_artists["scan2"] += [self.scan2_line]
        self.layer_artists["local_path"] += [self.local_path_line]
        self.layer_artists["robot"] += self.robot_patches
        self.layer_artists["footprint"] += [self.footprint_line]

    def _build_overlays(self) -> None:
        # "Live" badge, top-left
        self.badge = tk.Frame(self, bg=BG)
        self.badge_dot = tk.Canvas(self.badge, width=22, height=22, bg=BG, highlightthickness=0)
        self.badge_dot.pack(side="left")
        self.badge_text = tk.Label(self.badge, bg=BG, font=(FONT, 11, "bold"))
        self.badge_text.pack(side="left", padx=4)
        self.badge.place(x=12, y=10)
        self._badge_tip = ""
        Tooltip(self.badge_dot, lambda: self._badge_tip)

        # status bar, top centre
        bar = card(self)
        self.status_items = {}
        for kind in ("error", "battery", "wifi", "estop", "localization"):
            box = tk.Frame(bar, bg=BG)
            icon = StatusIcon(box, kind)
            icon.pack(side="left")
            label = tk.Label(box, bg=BG, fg="#4c535f", font=(FONT, 10, "bold"))
            if kind not in ("error", "estop"):
                label.pack(side="left", padx=(3, 0))
            box.pack(side="left", padx=9, pady=9)
            self.status_items[kind] = (icon, label, {"tip": ""})
            Tooltip(icon, lambda k=kind: self.status_items[k][2]["tip"])
        bar.place(relx=0.5, y=16, anchor="n")

        # view options, left middle
        views = card(self)
        self.pan_button = ViewButton(views, "pan", self.toggle_pan, "Pan (drag the map)")
        self.pan_button.set_active(True)
        self.pan_button.pack(padx=6, pady=(8, 4))
        ViewButton(views, "fit", self.fit_view, "Fit to screen").pack(padx=6, pady=4)
        ViewButton(views, "zoom_in", lambda: self.zoom(1 / 1.4), "Zoom in").pack(padx=6, pady=4)
        ViewButton(views, "zoom_out", lambda: self.zoom(1.4), "Zoom out").pack(padx=6, pady=(4, 8))
        views.place(x=20, rely=0.5, anchor="w")

        # layers panel toggle, right
        self.panel_button = tk.Label(self, text="«", bg=BG, fg=MUTED, font=(FONT, 16, "bold"),
                                     width=2, cursor="hand2", highlightthickness=1,
                                     highlightbackground=CARD_BORDER)
        self.panel_button.bind("<Button-1>", lambda _e: self.toggle_panel())
        self.panel_button.place(relx=1.0, x=-4, y=90, anchor="ne")
        Tooltip(self.panel_button, "Layers and map info")
        self.panel = self._build_panel()
        self.panel_open = False

        # mouse position and messages, bottom-left
        self.coord_label = tk.Label(self, bg=BG, fg=MUTED, font=(FONT, 9))
        self.coord_label.place(x=12, rely=1.0, y=-8, anchor="sw")
        self.message_box = tk.Frame(self, bg=BG)
        self.message = tk.Label(self.message_box, bg=BG, fg=MUTED, font=(FONT, 12))
        self.message.pack()
        self.change_button = green_button(self.message_box, "Change robot address", self.app.change_robot)
        self.message_box.place(relx=0.5, rely=0.5, anchor="center")

    def _build_panel(self) -> tk.Frame:
        panel = card(self, padx=12, pady=10)
        tk.Label(panel, text="Layers", bg=BG, fg=TEXT, font=(FONT, 11, "bold")).pack(anchor="w")
        for key, label in LAYERS:
            tk.Checkbutton(panel, text=label, variable=self.layer_vars[key], bg=BG, fg=TEXT,
                           activebackground=BG, selectcolor="white", font=(FONT, 9), anchor="w",
                           highlightthickness=0, bd=0,
                           command=self.apply_layer_visibility).pack(anchor="w")
        tk.Frame(panel, bg=LINE, height=1).pack(fill="x", pady=6)
        tk.Checkbutton(panel, text="Follow robot", variable=self.follow_var, bg=BG, fg=TEXT,
                       activebackground=BG, selectcolor="white", font=(FONT, 9, "bold"),
                       highlightthickness=0, bd=0,
                       command=self.mark_dirty).pack(anchor="w")
        tk.Frame(panel, bg=LINE, height=1).pack(fill="x", pady=6)
        tk.Label(panel, text="Live map", bg=BG, fg=TEXT, font=(FONT, 11, "bold")).pack(anchor="w")
        self.map_info = tk.Label(panel, text="loading...", bg=BG, fg=MUTED, font=(FONT, 9),
                                 justify="left", anchor="w")
        self.map_info.pack(anchor="w", fill="x")
        green_button(panel, "Reload map", self.app.load_live_map).pack(anchor="w", pady=(8, 0))
        return panel

    # ------------------------------------------------------------ static map
    def set_map(self, params: dict, m: dict) -> None:
        """Draw a newly loaded map (the parts that don't move)."""
        self.params, self.map = params, m
        for artist in self.static_artists:
            artist.remove()
        self.static_artists = []
        for key in ("map", "stations", "paths", "areas", "reflectors"):
            self.layer_artists[key] = []
        ax = self.ax

        pts = m.get("normalPosList") or []
        if pts:
            line, = ax.plot([p["x"] for p in pts], [p["y"] for p in pts], "o", ms=1.3, mew=0,
                            color=MAP_POINTS, zorder=1)
            self._add_static("map", line)

        polys, faces, edges = [], [], []
        for area in m.get("advancedAreaList") or []:
            poly = [(p["x"], p["y"]) for p in area.get("posGroup") or []]
            if len(poly) >= 3:
                color = AREA_COLORS.get(area.get("className"), "#0066C5")
                polys.append(poly)
                faces.append(to_rgba(color, 0.2))
                edges.append(color)
        if polys:
            self._add_static("areas", ax.add_collection(
                PolyCollection(polys, facecolors=faces, edgecolors=edges, linewidths=1, zorder=2)))

        curves, arrows = [], []
        for path in m.get("advancedCurveList") or []:
            pp = path_points(path, params)
            curves.append(pp)
            if len(pp) >= 2:
                arrows.append(arrow_at_middle(pp))
        if curves:
            self._add_static("paths", ax.add_collection(
                LineCollection(curves, colors=PATH, linewidths=2.5, zorder=3)))
            self._add_static("paths", ax.add_collection(
                PolyCollection(arrows, facecolors=PATH_ARROW, edgecolors="none", zorder=3.5)))

        size = params.get("advanced_point", {"width": 1.2, "height": 0.85})
        w, h = size.get("width", 1.2), size.get("height", 0.85)
        shapes = station_shapes(w, h, params.get("path_width", 0.03))
        polys, colors = [], []
        for st in m.get("advancedPointList") or []:
            x, y, d = st["pos"]["x"], st["pos"]["y"], st.get("dir", 0)
            for shape, color in shapes:
                polys.append(transform(shape, x, y, d))
                colors.append(color)
            half_height = abs(math.sin(d)) * w / 2 + abs(math.cos(d)) * h / 2
            self._add_static("stations", ax.text(
                x, y - half_height - 0.08, st.get("instanceName", ""), ha="center", va="top",
                fontsize=8, fontweight="bold", color=STATION_TEXT, zorder=4))
        if polys:
            self._add_static("stations", ax.add_collection(
                PolyCollection(polys, facecolors=colors, edgecolors="none", zorder=4)))

        for r in m.get("reflectorList") or []:
            self._add_static("reflectors", ax.add_patch(Circle(
                (r["x"], r["y"]), 0.15, color=REFLECTOR, zorder=4)))

        hdr = m.get("header", {})
        if hdr.get("minPos") and hdr.get("maxPos") and hdr["minPos"] != hdr["maxPos"]:
            self.map_bounds = (hdr["minPos"]["x"], hdr["maxPos"]["x"],
                               hdr["minPos"]["y"], hdr["maxPos"]["y"])
        elif pts:
            xs, ys = [p["x"] for p in pts], [p["y"] for p in pts]
            self.map_bounds = (min(xs), max(xs), min(ys), max(ys))
        else:
            self.map_bounds = None

        info = AMRClient.summarize_map(m)
        self.map_info.config(text=f"{info['name'] or '(no name)'}\n{info['points']:,} points\n"
                                  f"{info['stations']} stations, {info['paths']} paths\n"
                                  f"{info['areas']} areas, {info['reflectors']} reflectors")
        self.map_state = "ready"
        self.apply_layer_visibility()
        self.fit_view()

    def _add_static(self, layer: str, artist) -> None:
        self.static_artists.append(artist)
        self.layer_artists[layer].append(artist)

    def map_failed(self, error: Exception) -> None:
        if self.map is None:
            self.map_state = "failed"
            self.message.config(text=f"Cannot reach the robot at {self.app.host}\n"
                                     f"Retrying every few seconds...\n\n"
                                     f"If the robot's address has changed, set the new one:")
            self.change_button.pack(pady=(10, 0))
        self.map_info.config(text="Could not load the map.")

    # ------------------------------------------------------------- live data
    def refresh(self, snap: dict, blink: bool) -> None:
        self._update_badge(snap)
        self._update_status_bar(snap.get("status"), blink)

        status = snap.get("status")
        if "pose" in snap:
            pose = snap["pose"]
        elif status:
            pose = {"x": status.get("x", 0), "y": status.get("y", 0), "angle": status.get("angle", 0)}
        else:
            pose = None

        if self._drawn_version == snap["version"] and not self._dirty:
            return
        self._drawn_version, self._dirty = snap["version"], False

        client = self.app.client
        have_params = bool(self.params)       # lidar offsets come from the parameters
        if pose:
            for patch, shape in zip(self.robot_patches, robot_shapes(*self._robot_size())):
                patch.set_xy(transform(shape, pose["x"], pose["y"], pose["angle"]))
                patch.set_visible(self.layer_vars["robot"].get())
        self._set_points(self.lidar_line, client.lidar_to_points(snap["lidar"], pose)
                         if pose and have_params and "lidar" in snap else [])
        self._set_points(self.scan2_line, client.lidar_to_points(snap["scan2"], pose, use_mount_angle=False)
                         if pose and have_params and "scan2" in snap else [])

        fp = snap.get("footprint") or []
        if pose and len(fp) >= 6:
            pts = [(fp[i], fp[i + 1]) for i in range(0, len(fp) - 1, 2)]
            pts = robot_to_map(pts + pts[:1], pose)
            self.footprint_line.set_data([p[0] for p in pts], [p[1] for p in pts])
        else:
            self.footprint_line.set_data([], [])

        local = snap.get("local_path") or []
        if pose and local:
            self._set_points(self.local_path_line, robot_to_map([(p["x"], p["y"]) for p in local], pose))
            boxes = []
            for p in local[::5]:
                if p.get("x") or p.get("y"):
                    box = transform(rectangle(*PREDICTED_FOOTPRINT), p["x"], p["y"], p.get("angle", 0))
                    boxes.append(robot_to_map(box, pose))
            self.predicted.set_verts(boxes)
        else:
            self._set_points(self.local_path_line, [])
            self.predicted.set_verts([])

        gp = snap.get("global_path") or []
        self._set_points(self.global_path_line,
                         [(p["x"], p["y"]) for p in gp if isinstance(p, dict)])

        if self.follow_var.get() and pose:
            self.center_on(pose["x"], pose["y"])
        self.canvas.draw_idle()

    def _robot_size(self):
        dims = self.params.get("robot_dimensions") or {}
        return dims.get("width", 0.85), dims.get("height", 0.75)

    @staticmethod
    def _set_points(line, points) -> None:
        line.set_data([p[0] for p in points], [p[1] for p in points])

    def _update_badge(self, snap: dict) -> None:
        if self.map_state == "loading":
            fill, ring, text, color = "#9ec5f0", LIDAR, "Loading map", LIDAR
        elif snap["online"]:
            fill, ring, text, color = LIVE_FILL, LIVE_RING, "Live", LIVE_RING
        else:
            fill, ring, text, color = "#d6d6d6", "#9e9e9e", "Offline", "#9e9e9e"
        self._badge_tip = (f"Connected to {self.app.host}" if snap["online"]
                           else f"No data from {self.app.host}\n{snap.get('error') or ''}".strip())
        self.badge_dot.delete("all")
        self.badge_dot.create_oval(3, 3, 19, 19, fill=fill, outline=ring, width=3)
        self.badge_text.config(text=text, fg=color)
        if self.map_state == "ready":
            self.message_box.place_forget()
        elif self.map_state == "loading":
            self.message.config(text="Loading the live map...")

    def _update_status_bar(self, s: dict | None, blink: bool) -> None:
        def show(kind, color, text="", tip="", level=1.0):
            icon, label, meta = self.status_items[kind]
            if color == ICON_RED and blink:
                color = "#f4a3a1"          # red icons pulse, like the web app
            icon.draw(color, level)
            label.config(text=text)
            meta["tip"] = tip

        if not s:
            for kind in self.status_items:
                show(kind, "#bdbdbd", "--", "No data from the robot")
            return
        d = self.app.client.get_summary(s)
        err = s.get("error_id", 0)
        show("error", ICON_RED if err not in (0, 1) else ICON_YELLOW, "",
             {"none": "No errors", "warning": "Warning"}.get(d["error_level"], "Error")
             + " (click Robot Status for details)")
        b = d["battery_percent"]
        show("battery", ICON_RED if b < 30 else ICON_BLUE if b < 80 else ICON_GREEN,
             f"{math.floor(b * 10) / 10:g}%", f"Battery ({d['charging_state']})", b / 100)
        wf = d["wifi_signal"]
        show("wifi", ICON_RED if wf < 25 else ICON_BLUE if wf < 75 else ICON_GREEN,
             f"{wf:g}", "Wi-Fi signal", wf / 100)
        pressed = d["emergency_stop"] == "pressed"
        show("estop", ICON_RED if pressed else ICON_BLUE, "",
             "Emergency stop PRESSED" if pressed else "Emergency stop released")
        conf = d["localization_percent"]
        show("localization", ICON_RED if conf < 70 else ICON_GREEN, f"{conf:.1f}%",
             "Localization confidence")

    # ---------------------------------------------------------- view control
    def mark_dirty(self) -> None:
        self._dirty = True

    def apply_layer_visibility(self) -> None:
        for key, artists in self.layer_artists.items():
            for a in artists:
                a.set_visible(self.layer_vars[key].get())
        self._dirty = True
        self.canvas.draw_idle()

    def _get_view(self):
        """Current view as (centre x, centre y, width in metres)."""
        (x0, x1), (y0, y1) = self.ax.get_xlim(), self.ax.get_ylim()
        return (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0

    def _set_view(self, cx: float, cy: float, width: float) -> None:
        """Show `width` metres around (cx, cy), same scale on both axes."""
        w_px, h_px = max(self.ax.bbox.width, 1), max(self.ax.bbox.height, 1)
        height = width * h_px / w_px
        self.ax.set_xlim(cx - width / 2, cx + width / 2)
        self.ax.set_ylim(cy - height / 2, cy + height / 2)
        self.canvas.draw_idle()

    # space (pixels) kept clear of the floating controls when fitting the map
    FIT_MARGIN = {"left": 90, "right": 60, "top": 80, "bottom": 30}

    def fit_view(self) -> None:
        if not self.map_bounds:
            return
        x0, x1, y0, y1 = self.map_bounds
        m = self.FIT_MARGIN
        w_px, h_px = max(self.ax.bbox.width, 1), max(self.ax.bbox.height, 1)
        free_w = max(w_px - m["left"] - m["right"], 50)
        free_h = max(h_px - m["top"] - m["bottom"], 50)
        scale = 0.95 * min(free_w / max(x1 - x0, 0.5), free_h / max(y1 - y0, 0.5))  # pixels per metre
        cx = (x0 + x1) / 2 - (m["left"] - m["right"]) / 2 / scale
        cy = (y0 + y1) / 2 + (m["top"] - m["bottom"]) / 2 / scale
        self._set_view(cx, cy, w_px / scale)

    def zoom(self, factor: float, px: float | None = None, py: float | None = None) -> None:
        """Zoom by `factor` (< 1 zooms in), keeping the point (px, py) still."""
        cx, cy, width = self._get_view()
        if self.map_bounds:
            biggest = 4 * max(self.map_bounds[1] - self.map_bounds[0], self.map_bounds[3] - self.map_bounds[2], 5)
        else:
            biggest = 200
        if not 0.5 <= width * factor <= biggest:
            return
        px = cx if px is None else px
        py = cy if py is None else py
        self._set_view(px - (px - cx) * factor, py - (py - cy) * factor, width * factor)

    def center_on(self, x: float, y: float) -> None:
        self._set_view(x, y, self._get_view()[2])

    def toggle_pan(self) -> None:
        self.pan_enabled = not self.pan_enabled
        self.pan_button.set_active(self.pan_enabled)

    def toggle_panel(self) -> None:
        if self.panel_open:
            self.panel.place_forget()
            self.panel_button.config(text="«")
        else:
            self.panel.place(relx=1.0, x=-48, y=80, anchor="ne")
            self.panel.lift()
            self.panel_button.config(text="»")
        self.panel_open = not self.panel_open

    # ------------------------------------------------------------ mouse input
    def _on_scroll(self, event) -> None:
        if event.xdata is not None:
            self.zoom(1 / 1.2 if event.button == "up" else 1.2, event.xdata, event.ydata)

    def _on_press(self, event) -> None:
        if event.button == 1 and self.pan_enabled:
            self._drag = (event.x, event.y, self._get_view())
            if self.follow_var.get():
                self.follow_var.set(False)   # dragging takes over from follow mode

    def _on_release(self, _event) -> None:
        self._drag = None

    def _on_motion(self, event) -> None:
        if event.xdata is not None:
            self.coord_label.config(text=f"x {event.xdata:.2f} m   y {event.ydata:.2f} m")
        if self._drag:
            px, py, (cx, cy, width) = self._drag
            metres_per_pixel = width / max(self.ax.bbox.width, 1)
            self._set_view(cx - (event.x - px) * metres_per_pixel,
                           cy - (event.y - py) * metres_per_pixel, width)


# =============================================================================
# Robot Status page
# =============================================================================

class StatusPage(tk.Frame):
    FIELDS = [
        ("Battery", lambda d, s: f"{d['battery_percent']}%  ({d['battery_level']})"),
        ("Charging", lambda d, s: d["charging_state"]),
        ("Battery current", lambda d, s: f"{d['battery_current_A']:.2f} A"),
        ("Battery voltage", lambda d, s: f"{d['battery_voltage_V']} V"),
        ("Temperature", lambda d, s: f"{d['temperature_C']} °C"),
        ("Wi-Fi signal", lambda d, s: f"{d['wifi_signal']:g}"),
        ("Emergency stop", lambda d, s: d["emergency_stop"]),
        ("Errors", lambda d, s: d["error_level"]),
        ("Localization", lambda d, s: f"{d['localization_percent']}%"
                                      + ("" if d["localized"] else "  (not localized)")),
        ("Position", lambda d, s: f"x {d['pose']['x']:.2f}   y {d['pose']['y']:.2f}   "
                                  f"{math.degrees(d['pose']['angle'] or 0):.0f}°"),
        ("Speed", lambda d, s: f"{d['linear_speed']} m/s   {d['angular_speed']} rad/s"),
        ("Motor RPM", lambda d, s: "left {left_motor_rpm}   right {right_motor_rpm}".format(
            **(d["motor_rpm"] or {"left_motor_rpm": "-", "right_motor_rpm": "-"}))),
    ]

    def __init__(self, parent, app: "AMRAgentApp"):
        super().__init__(parent, bg=BG, padx=24, pady=20)
        self.app = app
        self._last = 0.0
        self._error_rows: list = []
        self.columnconfigure(0, weight=1, uniform="c")
        self.columnconfigure(1, weight=1, uniform="c")
        self.rowconfigure(1, weight=1)

        overview = card(self, padx=16, pady=12)
        overview.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=(0, 14))
        tk.Label(overview, text="Overview", bg=BG, fg=TEXT, font=(FONT, 13, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))
        self.values = {}
        for i, (name, _) in enumerate(self.FIELDS, start=1):
            tk.Label(overview, text=name, bg=BG, fg=MUTED, font=(FONT, 10)).grid(
                row=i, column=0, sticky="w", pady=2)
            v = tk.Label(overview, text="--", bg=BG, fg=TEXT, font=(FONT, 10, "bold"))
            v.grid(row=i, column=1, sticky="w", padx=(20, 0), pady=2)
            self.values[name] = v

        errors = card(self, padx=16, pady=12)
        errors.grid(row=0, column=1, sticky="nsew", padx=(10, 0), pady=(0, 14))
        tk.Label(errors, text="Errors & warnings", bg=BG, fg=TEXT, font=(FONT, 13, "bold")).pack(anchor="w")
        self.error_list = tk.Listbox(errors, height=10, font=(FONT, 10), bd=0, highlightthickness=1,
                                     highlightbackground=LINE, activestyle="none")
        self.error_list.pack(fill="both", expand=True, pady=8)
        green_button(errors, "Clear errors", self.clear_errors).pack(anchor="w")

        raw = card(self, padx=16, pady=12)
        raw.grid(row=1, column=0, columnspan=2, sticky="nsew")
        tk.Label(raw, text="All status fields", bg=BG, fg=TEXT, font=(FONT, 13, "bold")).pack(anchor="w")
        self.tree = ttk.Treeview(raw, columns=("value",), show="tree headings", height=8)
        self.tree.heading("#0", text="Field")
        self.tree.heading("value", text="Value")
        self.tree.column("#0", width=260)
        scroll = ttk.Scrollbar(raw, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True, pady=(8, 0))
        scroll.pack(side="right", fill="y", pady=(8, 0))

    def refresh(self, snap: dict, _blink: bool) -> None:
        if time.time() - self._last < 1:
            return
        self._last = time.time()
        s = snap.get("status")
        if s:
            d = self.app.client.get_summary(s)
            for name, fmt in self.FIELDS:
                try:
                    self.values[name].config(text=fmt(d, s))
                except (TypeError, ValueError, KeyError):
                    self.values[name].config(text="--")
            for key in sorted(s):                     # update in place: keeps scroll position
                value = s[key]
                text = json.dumps(value) if isinstance(value, (dict, list)) else str(value)
                if self.tree.exists(key):
                    self.tree.item(key, values=(text,))
                else:
                    self.tree.insert("", "end", iid=key, text=key, values=(text,))
        else:
            for v in self.values.values():
                v.config(text="--")

        e = snap.get("errors")
        if e is None:
            rows = [("No data from the robot", MUTED)]
        else:
            rows = ([(f"ERROR    {m}", ICON_RED) for m in e.get("errors") or []]
                    + [(f"WARNING  {m}", "#b8860b") for m in e.get("warnings") or []])
            rows = rows or [("No errors or warnings", MUTED)]
        if rows != self._error_rows:                 # redraw only when something changed
            self._error_rows = rows
            self.error_list.delete(0, "end")
            for text, color in rows:
                self.error_list.insert("end", text)
                self.error_list.itemconfig("end", fg=color)

    def clear_errors(self) -> None:
        run_in_background(self.app.results, "cleared", AMRClient(self.app.host).clear_errors)


# =============================================================================
# Saved Maps page
# =============================================================================

class MapsPage(tk.Frame):
    COLUMNS = [("id", "ID", 60), ("name", "Name", 280), ("live", "Live", 60), ("points", "Points", 90),
               ("stations", "Stations", 80), ("paths", "Paths", 70), ("areas", "Areas", 70)]

    def __init__(self, parent, app: "AMRAgentApp"):
        super().__init__(parent, bg=BG, padx=24, pady=20)
        self.app = app
        self.maps: list = []
        self.loaded_once = False

        top = tk.Frame(self, bg=BG)
        top.pack(fill="x")
        tk.Label(top, text="Saved maps on the robot", bg=BG, fg=TEXT, font=(FONT, 15, "bold")).pack(side="left")
        green_button(top, "Export live map...", self.export_live).pack(side="right", padx=(8, 0))
        green_button(top, "Export selected...", self.export_selected).pack(side="right", padx=(8, 0))
        green_button(top, "Refresh", self.load).pack(side="right")

        self.info = tk.Label(self, text="", bg=BG, fg=MUTED, font=(FONT, 10), anchor="w")
        self.info.pack(fill="x", pady=(10, 6))

        box = card(self)
        box.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(box, columns=[c[0] for c in self.COLUMNS], show="headings")
        for key, title, width in self.COLUMNS:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w" if key == "name" else "center")
        scroll = ttk.Scrollbar(box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.tree.tag_configure("live", foreground=ACCENT_DARK)

    def on_show(self) -> None:
        if not self.loaded_once:
            self.load()

    def refresh(self, _snap: dict, _blink: bool) -> None:
        pass

    def load(self) -> None:
        self.loaded_once = True
        self.info.config(text="Loading maps from the robot (this downloads every map, may take a while)...")
        run_in_background(self.app.results, "maps", AMRClient(self.app.host, timeout=30).list_maps)

    def show_maps(self, maps: list) -> None:
        self.maps = maps
        self.tree.delete(*self.tree.get_children())
        for m in maps:
            s = AMRClient.summarize_map(m)
            self.tree.insert("", "end", iid=str(s["id"]), tags=("live",) if s["is_live"] else (),
                             values=(s["id"], s["name"], "yes" if s["is_live"] else "", f"{s['points']:,}",
                                     s["stations"], s["paths"], s["areas"]))
        self.info.config(text=f"{len(maps)} saved maps. The green row is the live map.")

    def show_error(self, error: Exception) -> None:
        self.info.config(text=f"Could not load maps: {error}")

    def export_selected(self) -> None:
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo(APP_TITLE, "Select a map in the list first.")
            return
        m = next((m for m in self.maps if str(m.get("id")) == sel[0]), None)
        if m:
            self._save(m["map"], f"map_{m.get('id')}.json")

    def export_live(self) -> None:
        live = self.app.map_page.map
        if live is None:
            messagebox.showinfo(APP_TITLE, "The live map has not loaded yet.")
            return
        self._save(live, "live_map.json")

    def _save(self, body: dict, default_name: str) -> None:
        path = filedialog.asksaveasfilename(title="Save map", defaultextension=".json",
                                            initialfile=default_name, filetypes=[("JSON", "*.json")])
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(body, f)
            self.info.config(text=f"Saved {path}")


# =============================================================================
# Connecting (the robot's IP address can change)
# =============================================================================

class ConnectForm(tk.Frame):
    """Asks for the robot's IP address, can search the network, and tests the address."""

    def __init__(self, parent, on_connected, on_cancel=None):
        super().__init__(parent, bg=BG, padx=28, pady=20)
        self.on_connected, self.on_cancel = on_connected, on_cancel
        self.results: queue.Queue = queue.Queue()
        self._failed_host = None          # address that didn't answer; next Connect opens it anyway
        self._found: list = []

        tk.Frame(self, bg=BG, width=420, height=0).pack()      # sets the form's width
        tk.Label(self, text=APP_TITLE, bg=BG, fg="#000000", font=(FONT, 20, "bold")).pack()
        tk.Label(self, text="Connect to a robot", bg=BG, fg=MUTED, font=(FONT, 11)).pack(pady=(0, 16))

        tk.Label(self, text="Robot IP address", bg=BG, fg=TEXT, font=(FONT, 10, "bold")).pack(anchor="w")
        recent = recent_hosts()
        self.host_var = tk.StringVar(value=recent[0] if recent else DEFAULT_HOST)
        self.entry = ttk.Combobox(self, textvariable=self.host_var, values=recent, font=(FONT, 12))
        self.entry.pack(fill="x", pady=(4, 2))
        self.entry.bind("<Return>", lambda _e: self.connect())
        self.host_var.trace_add("write", lambda *_a: self._address_edited())
        tk.Label(self, text="For example 192.168.1.50. You can also paste the robot's web address.",
                 bg=BG, fg=MUTED, font=(FONT, 8)).pack(anchor="w")

        self.message = tk.Label(self, text="", bg=BG, fg=MUTED, font=(FONT, 9), justify="left",
                                anchor="w", wraplength=400)
        self.message.pack(fill="x", pady=(12, 8))

        buttons = tk.Frame(self, bg=BG)
        buttons.pack(fill="x")
        self.search_button = plain_button(buttons, "Search network", self.search)
        self.search_button.pack(side="left")
        self.connect_button = green_button(buttons, "Connect", self.connect)
        self.connect_button.pack(side="right")
        if on_cancel:
            plain_button(buttons, "Cancel", on_cancel).pack(side="right", padx=(0, 8))

        self.found_frame = tk.Frame(self, bg=BG)
        tk.Label(self.found_frame, text="Robots found (double-click to connect)", bg=BG, fg=TEXT,
                 font=(FONT, 9, "bold")).pack(anchor="w", pady=(12, 4))
        self.found_list = tk.Listbox(self.found_frame, height=4, font=(FONT, 11), bd=0,
                                     highlightthickness=1, highlightbackground=CARD_BORDER,
                                     activestyle="none", selectbackground="#d6ecdf",
                                     selectforeground=TEXT)
        self.found_list.pack(fill="x")
        self.found_list.bind("<<ListboxSelect>>", self._pick_found)
        self.found_list.bind("<Double-Button-1>", lambda _e: self.connect())

        self.entry.focus_set()
        self.entry.select_range(0, "end")
        self._poll_timer = self.after(100, self._poll)

    def destroy(self) -> None:
        self.after_cancel(self._poll_timer)
        super().destroy()

    # ------------------------------------------------------------- actions
    def connect(self) -> None:
        host = normalize_host(self.host_var.get())
        if not host:
            self._say("Type the robot's IP address first.", ICON_RED)
            return
        if host == self._failed_host:      # second press after a failed check
            self._finish(host)
            return
        self._busy(True)
        self._say(f"Connecting to {host}...")
        run_in_background(self.results, "check", lambda h: (h, check_robot(h, timeout=3)), host)

    def search(self) -> None:
        prefixes = local_subnets()
        typed = normalize_host(self.host_var.get())
        if re.fullmatch(r"\d+\.\d+\.\d+\.\d+", typed):
            prefixes.append(typed.rsplit(".", 1)[0])   # also look near the typed / last address
        prefixes = list(dict.fromkeys(prefixes))
        if not prefixes:
            self._say("This computer doesn't seem to be on a network.", ICON_RED)
            return
        self._busy(True)
        self._found = []
        self.found_list.delete(0, "end")
        self._say("Searching " + ", ".join(f"{p}.1-254" for p in prefixes) + " ...")

        def progress(done, total, host):
            if host:
                self.results.put(("found", True, host))
            if done % 16 == 0 or done == total:
                self.results.put(("progress", True, (done, total)))

        run_in_background(self.results, "search", lambda: find_robots(prefixes, on_progress=progress))

    # ------------------------------------------------------------- helpers
    def _poll(self) -> None:
        try:
            while True:
                tag, ok, result = self.results.get_nowait()
                if tag == "check":
                    self._busy(False)
                    host, params = result if ok else (None, None)
                    if params is not None:
                        self._finish(host)
                        return
                    self._failed_host = normalize_host(self.host_var.get())
                    self.connect_button.config(text="Connect anyway")
                    self._say(f"No robot answered at {self._failed_host}. Check the address and that "
                              f"this computer is on the robot's network, or press Search network. "
                              f"Press Connect anyway to open it regardless.", ICON_RED)
                elif tag == "progress":
                    done, total = result
                    self._say(f"Searching... {done}/{total} addresses checked, {len(self._found)} found")
                elif tag == "found":
                    self._found.append(result)
                    self.found_frame.pack(fill="x")
                    self.found_list.insert("end", result)
                elif tag == "search":
                    self._busy(False)
                    found = result if ok else []
                    if found:
                        self.host_var.set(found[0])
                        self._say(f"Found {len(found)} robot{'s' if len(found) > 1 else ''}. "
                                  f"Press Connect.", ACCENT_DARK)
                    else:
                        self._say("No robot found. Make sure the robot is on and this computer is on "
                                  "the same network, or type the address.", ICON_RED)
        except queue.Empty:
            pass
        except tk.TclError:
            return
        self._poll_timer = self.after(100, self._poll)

    def _finish(self, host: str) -> None:
        remember_host(host)
        self.on_connected(host)

    def _address_edited(self) -> None:
        if self._failed_host and normalize_host(self.host_var.get()) != self._failed_host:
            self._failed_host = None
            self.connect_button.config(text="Connect")

    def _pick_found(self, _event=None) -> None:
        sel = self.found_list.curselection()
        if sel:
            self.host_var.set(self.found_list.get(sel[0]))

    def _busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        self.connect_button.config(state=state)
        self.search_button.config(state=state)

    def _say(self, text: str, color: str = MUTED) -> None:
        self.message.config(text=text, fg=color)


class ConnectWindow(tk.Tk):
    """The window shown at start-up. After mainloop(), .chosen_host is the address (or None)."""

    def __init__(self):
        super().__init__()
        self.chosen_host = None
        self.title(f"{APP_TITLE} - Connect")
        self.configure(bg=BG)
        self.resizable(False, False)
        set_window_icon(self)
        style_ttk(self)
        self.form = ConnectForm(self, self._done)
        self.form.pack(fill="both", expand=True)
        center_window(self)

    def _done(self, host: str) -> None:
        self.chosen_host = host
        cancel_timers(self)
        self.destroy()


def ask_for_host():
    """Show the Connect window and return the chosen address (None if closed)."""
    win = ConnectWindow()
    win.mainloop()
    return win.chosen_host


# =============================================================================
# Main window
# =============================================================================

class AMRAgentApp(tk.Tk):
    PAGES = [("map", "Map View"), ("status", "Robot Status"), ("maps", "Saved Maps")]

    def __init__(self, host: str):
        super().__init__()
        self.host = host
        self.client = AMRClient(host)
        self.results: queue.Queue = queue.Queue()
        self.data = LiveData(host)
        self._params: dict = {}
        self._blink, self._ticks = False, 0
        self._closed = False
        self.next_host = None             # set when the user switches to another robot

        self.title(f"{APP_TITLE} - {host}")
        self.geometry("1280x800")
        self.minsize(1000, 640)
        self.configure(bg=BG)
        set_window_icon(self)
        style_ttk(self)

        self._build_navbar()
        tk.Frame(self, bg=LINE, height=3).pack(fill="x")
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True)
        self._build_sidebar(body)
        self.sidebar_line = tk.Frame(body, bg=LINE, width=3)
        self.sidebar_line.pack(side="left", fill="y")
        content = tk.Frame(body, bg=BG)
        content.pack(side="left", fill="both", expand=True)
        content.rowconfigure(0, weight=1)
        content.columnconfigure(0, weight=1)

        self.map_page = MapViewPage(content, self)
        self.pages = {"map": self.map_page, "status": StatusPage(content, self), "maps": MapsPage(content, self)}
        for page in self.pages.values():
            page.grid(row=0, column=0, sticky="nsew")
        self.show_page("map")

        self.data.start()
        self.load_live_map()
        self.after(REFRESH_MS, self._tick)

    # ---------------------------------------------------------------- layout
    def _build_navbar(self) -> None:
        nav = tk.Frame(self, bg=BG, height=60, padx=30)
        nav.pack(fill="x")
        nav.pack_propagate(False)
        nav.columnconfigure(0, weight=1, uniform="side")
        nav.columnconfigure(2, weight=1, uniform="side")
        nav.rowconfigure(0, weight=1)
        menu = tk.Label(nav, text="\u2630", bg=BG, fg="#000000", font=(FONT, 20), cursor="hand2")
        menu.grid(row=0, column=0, sticky="w")
        menu.bind("<Button-1>", lambda _e: self.toggle_sidebar())
        tk.Label(nav, text=APP_TITLE, bg=BG, fg="#000000", font=(FONT, 22, "bold")).grid(row=0, column=1)
        robot = tk.Frame(nav, bg=BG)
        robot.grid(row=0, column=2, sticky="e")
        tk.Label(robot, text=f"Robot  {self.host}", bg=BG, fg=MUTED, font=(FONT, 10)).pack(side="left")
        change = tk.Label(robot, text="Change", bg=BG, fg=ACCENT_DARK, cursor="hand2",
                          font=(FONT, 10, "bold underline"))
        change.pack(side="left", padx=(10, 0))
        change.bind("<Button-1>", lambda _e: self.change_robot())
        Tooltip(change, "Connect to a different robot address")

    def _build_sidebar(self, parent) -> None:
        self.sidebar = tk.Frame(parent, bg=BG, width=190, padx=14, pady=20)
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)
        self.sidebar_items = {}
        for key, label in self.PAGES:
            item = tk.Frame(self.sidebar, bg=BG, height=40, cursor="hand2",
                            highlightthickness=1, highlightbackground=BG)
            item.pack(fill="x", pady=5)
            item.pack_propagate(False)
            icon = tk.Canvas(item, width=22, height=22, bg=BG, highlightthickness=0)
            icon.create_oval(3, 3, 19, 19, outline=MUTED, width=1.5)
            icon.create_arc(3, 3, 19, 19, start=0, extent=90, fill=MUTED, outline=MUTED)
            icon.pack(side="left", padx=(14, 6))
            text = tk.Label(item, text=label, bg=BG, fg=TEXT, font=(FONT, 11))
            text.pack(side="left")
            for w in (item, icon, text):
                w.bind("<Button-1>", lambda _e, k=key: self.show_page(k))
            self.sidebar_items[key] = (item, icon, text)
        self.sidebar_visible = True

    def change_robot(self) -> None:
        """Ask for another address; the window reopens connected to it."""
        dialog = tk.Toplevel(self)
        dialog.title(f"{APP_TITLE} - Change robot")
        dialog.configure(bg=BG)
        dialog.resizable(False, False)
        dialog.transient(self)

        def connected(host: str) -> None:
            dialog.destroy()
            if host != self.host:
                self.next_host = host
                self.destroy()

        ConnectForm(dialog, connected, on_cancel=dialog.destroy).pack(fill="both", expand=True)
        center_window(dialog, over=self)
        dialog.grab_set()

    def destroy(self) -> None:
        self._closed = True
        self.data.stop()
        cancel_timers(self)
        super().destroy()

    def toggle_sidebar(self) -> None:
        if self.sidebar_visible:
            self.sidebar.pack_forget()
        else:
            self.sidebar.pack(side="left", fill="y", before=self.sidebar_line)
        self.sidebar_visible = not self.sidebar_visible

    def show_page(self, key: str) -> None:
        self.current = key
        self.pages[key].tkraise()
        for k, (item, icon, text) in self.sidebar_items.items():
            active = k == key
            bg = "white" if active else BG
            item.config(bg=bg, highlightbackground=CARD_BORDER if active else BG)
            icon.config(bg=bg)
            text.config(bg=bg, font=(FONT, 11, "bold" if active else "normal"))
        if hasattr(self.pages[key], "on_show"):
            self.pages[key].on_show()
        if key == "map":
            self.map_page.mark_dirty()

    # ------------------------------------------------------------------ data
    def load_live_map(self) -> None:
        def fetch():
            c = AMRClient(self.host, timeout=30)
            return c.get_params(), c.get_live_map()["data"]
        run_in_background(self.results, "map", fetch)

    def _handle_results(self) -> None:
        while True:
            try:
                tag, ok, result = self.results.get_nowait()
            except queue.Empty:
                return
            if tag == "map":
                if ok:
                    self._params, live = result
                    self.client.set_params(self._params)   # reuse for lidar offsets
                    self.map_page.set_map(self._params, live)
                else:
                    self.map_page.map_failed(result)
                    self.after(5000, self.load_live_map)
            elif tag == "maps":
                page = self.pages["maps"]
                page.show_maps(result) if ok else page.show_error(result)
            elif tag == "cleared" and not ok:
                messagebox.showerror(APP_TITLE, f"Could not clear errors:\n{result}")

    def _tick(self) -> None:
        if self._closed:
            return
        try:
            self._handle_results()
            self._ticks += 1
            if self._ticks % 3 == 0:
                self._blink = not self._blink
            snap = self.data.snapshot()
            self.pages[self.current].refresh(snap, self._blink)
            if self.current != "map":
                self.map_page._update_badge(snap)
        finally:
            if not self._closed:
                self.after(REFRESH_MS, self._tick)


def run(host=None) -> None:
    """Open AMR Agent. Without `host`, the Connect window asks for the address first."""
    try:  # sharp text on high-DPI Windows screens
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    host = normalize_host(host) if host else ask_for_host()
    while host:
        remember_host(host)
        app = AMRAgentApp(host)
        app.mainloop()
        host = app.next_host          # set by "Change"; None when the window was closed


if __name__ == "__main__":
    import sys
    run(sys.argv[1] if len(sys.argv) > 1 else None)