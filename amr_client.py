from __future__ import annotations

import json
import math
import os
import re
import socket
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from typing import Any, Callable, Iterator, Optional

import requests
import websocket  # from the "websocket-client" package


# =============================================================================
# Configuration
# =============================================================================

DEFAULT_HOST = "172.22.74.119"
API_KEY = "043ea683c1d0b7b5a9f584ef2e6c36079c62a7e7f0a76606778bab99"

# Ports used by the robot
PORT_REST = 3000      # HTTP: maps, params, tasks
PORT_CONTROL = 4000   # HTTP: mapping, relocation   WS: /tele_op, /nav
PORT_STATUS = 7000    # WS /status: live data channels
PORT_MISSION = 8765   # WS: mission / task-chain status
PORT_SOUND = 8900     # WS: sound list

# Live data channels on ws://<host>:7000/status (one channel per connection).
# Positions are in map coordinates unless marked "relative to the robot".
STATUS_API = {
    "lidar": "7001",            # laser scan: angle_min, angle_increment, ranges
    "robot_pose": "7002",       # {x, y, angle}            (silent if not localized)
    "robot_status": "7004",     # battery, pose, speeds... (always, about 1 Hz)
    "global_path": "7005",      # planned path after a navigation goal
    "local_path": "7006",       # local planner poses [{x, y, angle}], relative to the robot
    "footprint": "7007",        # robot outline [x1, y1, x2, y2, ...], relative to the robot
    "reflectors": "7009",       # detected reflectors as [x1, y1, x2, y2, ...]
    "errors_warnings": "7010",  # {errors: [...], warnings: [...]}
    "mapping": "7011",          # map being built: {map: {header, normalPosList}, path}
    "scan_2": "7012",           # second laser scan (same format as "lidar")
}

# Command ids
API_TELEOP = "4001"          # ws :4000/tele_op
API_FREE_NAV = "4002"        # ws :4000/nav
API_PATH_NAV = "4003"        # ws :4000/nav
API_CANCEL_NAV = "4004"      # ws :4000/nav
API_RELOCATE = "9002"        # ws :4000/tele_op
API_CLEAR_ERRORS = "8004"    # ws :7000/status
API_MISSION_STATUS = "8768"  # ws :8765
API_SOUND_LIST = "8902"      # ws :8900

# Code tables
TASK_STATUS = {0: "in progress", 1: "completed", 2: "error", 3: "waiting", 4: "cancelled"}
POWER_SUPPLY_STATUS = {0: "unknown", 1: "charging", 2: "discharging", 3: "not charging", 4: "full"}

LOCALIZED_THRESHOLD = 70   # confidence % at which the web app shows "localized" (green)


class ChannelSilentError(TimeoutError):
    """The connection works, but the robot sent nothing on a channel in time."""


def rotate_point(p: dict, angle: float) -> dict:
    """Rotate {x, y} around the origin by `angle` radians."""
    c, s = math.cos(angle), math.sin(angle)
    return {"x": p["x"] * c - p["y"] * s, "y": p["x"] * s + p["y"] * c}


def robot_to_map(points: list, pose: dict) -> list:
    """Move robot-relative (x, y) points into map coordinates using pose {x, y, angle}."""
    c, s = math.cos(pose["angle"]), math.sin(pose["angle"])
    return [(pose["x"] + x * c - y * s, pose["y"] + x * s + y * c) for x, y in points]


# =============================================================================
# Finding the robot (its IP address can change)
# =============================================================================

SETTINGS_FILE = os.path.join(os.path.expanduser("~"), ".amr_agent.json")
MAX_RECENT = 8


def normalize_host(text: str) -> str:
    """Clean up a typed address: '  http://10.0.0.5:2000/map-view ' -> '10.0.0.5'."""
    t = re.sub(r"^[a-zA-Z][a-zA-Z0-9+.-]*://", "", (text or "").strip())
    t = t.split("/")[0]
    if t.count(":") == 1:          # drop a :port (keeps IPv6 untouched)
        t = t.split(":")[0]
    return t


def check_robot(host: str, timeout: float = 2.0) -> Optional[dict]:
    """The robot's parameters if a robot answers at `host`, otherwise None."""
    try:
        r = requests.get(f"http://{host}:{PORT_REST}/params/", timeout=timeout)
        data = r.json()
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def local_subnets() -> list:
    """First three parts of this computer's IPv4 addresses, e.g. ['192.168.1']."""
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))     # sends nothing; just picks the main network card
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        ips += socket.gethostbyname_ex(socket.gethostname())[2]
    except OSError:
        pass
    prefixes = []
    for ip in ips:
        if ip.count(".") == 3 and not ip.startswith(("127.", "169.254.")):
            prefix = ip.rsplit(".", 1)[0]
            if prefix not in prefixes:
                prefixes.append(prefix)
    return prefixes


def find_robots(prefixes: Optional[list] = None,
                on_progress: Optional[Callable[[int, int, Optional[str]], None]] = None,
                workers: int = 64, timeout: float = 0.5) -> list:
    """Search addresses <prefix>.1 to <prefix>.254 for robots. Returns their addresses.

    Uses this computer's networks by default. Takes a few seconds per network.
    on_progress(done, total, found_host_or_None) is called from worker threads.
    """
    prefixes = prefixes or local_subnets()
    hosts = [f"{p}.{i}" for p in prefixes for i in range(1, 255)]

    def probe(host: str) -> Optional[str]:
        try:
            with socket.create_connection((host, PORT_REST), timeout=timeout):
                pass
        except OSError:
            return None
        return host if check_robot(host, timeout=2.0) is not None else None

    found, done = [], 0
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for future in as_completed([pool.submit(probe, h) for h in hosts]):
            done += 1
            host = future.result()
            if host:
                found.append(host)
            if on_progress:
                on_progress(done, len(hosts), host)
    return sorted(found, key=lambda h: [int(x) if x.isdigit() else x for x in h.split(".")])


def _load_settings() -> dict:
    try:
        with open(SETTINGS_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def recent_hosts() -> list:
    """Robot addresses used before, most recent first."""
    return [h for h in _load_settings().get("recent_hosts", []) if isinstance(h, str)]


def remembered_host() -> Optional[str]:
    """The last robot address that was used, or None."""
    recent = recent_hosts()
    return recent[0] if recent else None


def remember_host(host: str) -> None:
    """Save `host` as the most recent robot address (in ~/.amr_agent.json)."""
    settings = _load_settings()
    recent = [host] + [h for h in settings.get("recent_hosts", []) if h != host]
    settings["recent_hosts"] = recent[:MAX_RECENT]
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=2)
    except OSError:
        pass  # not being able to remember is not fatal


# =============================================================================
# Client
# =============================================================================

class AMRClient:
    def __init__(self, host: str = DEFAULT_HOST, timeout: float = 10.0):
        self.host = host
        self.timeout = timeout
        self.session = requests.Session()
        self._params: Optional[dict] = None

    @property
    def params(self) -> dict:
        """Robot parameters from /params, fetched once and cached."""
        if self._params is None:
            self._params = self.get_params()
        return self._params

    def set_params(self, params: dict) -> None:
        """Use parameters you already fetched instead of downloading them again."""
        self._params = params

    # -------------------------------------------------------------------------
    # Transport (internal)
    # -------------------------------------------------------------------------

    def _http(self, method: str, path: str, port: int = PORT_REST, **kwargs) -> Any:
        """HTTP request; returns parsed JSON (or text, or None for an empty body)."""
        kwargs.setdefault("timeout", self.timeout)
        r = self.session.request(method, f"http://{self.host}:{port}{path}", **kwargs)
        r.raise_for_status()
        if not r.content:
            return None
        try:
            return r.json()
        except ValueError:
            return r.text

    def _ws_open(self, port: int, path: str = "", timeout: Optional[float] = None):
        return websocket.create_connection(f"ws://{self.host}:{port}{path}",
                                           timeout=timeout or self.timeout)

    @staticmethod
    def _payload(request: dict) -> str:
        return json.dumps({"key": API_KEY, "request": request})

    def _ws_send(self, port: int, path: str, request: dict) -> None:
        """Send one command without waiting for a reply."""
        ws = self._ws_open(port, path)
        try:
            ws.send(self._payload(request))
        finally:
            ws.close()

    def _ws_request(self, port: int, path: str, request: dict, wait: float = 5.0) -> Optional[dict]:
        """Send one command and return the first reply (None if none within `wait`)."""
        ws = self._ws_open(port, path, wait)
        try:
            ws.send(self._payload(request))
            try:
                return json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                return None
        finally:
            ws.close()

    # -------------------------------------------------------------------------
    # Live data channels (low level)
    # -------------------------------------------------------------------------

    def stream(self, channel: str, max_messages: Optional[int] = None,
               timeout: Optional[float] = None) -> Iterator[Any]:
        """Yield the `data` of each message on a channel, e.g. stream("robot_status").

        `channel` is a name from STATUS_API or a raw api id. Raises
        ChannelSilentError if nothing arrives within `timeout` seconds.
        """
        api_id = STATUS_API.get(channel, channel)
        wait = timeout or self.timeout
        ws = self._ws_open(PORT_STATUS, "/status", wait)
        try:
            ws.send(self._payload({"api_id": api_id}))
            count = 0
            while max_messages is None or count < max_messages:
                try:
                    raw = ws.recv()
                except websocket.WebSocketTimeoutException:
                    raise ChannelSilentError(
                        f"No data on '{channel}' (api {api_id}) for {wait:g}s. The robot is "
                        f"reachable but is not sending this channel right now. Pose, lidar and "
                        f"paths stay silent when the robot is not localized or the lidar has no "
                        f"valid data. Check get_summary() and get_errors()."
                    ) from None
                msg = json.loads(raw)  # Python's json accepts the NaN / Infinity the robot sends
                if msg.get("status") is True:
                    yield msg.get("data")
                    count += 1
        finally:
            ws.close()

    def read_once(self, channel: str, timeout: Optional[float] = None) -> Any:
        """The next message on a channel."""
        return next(self.stream(channel, max_messages=1, timeout=timeout))

    # -------------------------------------------------------------------------
    # 1. Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict:
        """Raw robot status (always available, about once a second).

        Keys include percentage (battery %), current, voltage, temperature,
        power_supply_status, x, y, angle, confidence (localization %),
        emergency, error_id, linear_speed, angular_speed, rpm, signal_strength.
        """
        return self.read_once("robot_status")

    def get_summary(self, status: Optional[dict] = None) -> dict:
        """Status decoded the way the web app's top bar shows it."""
        s = status or self.get_status()
        battery = float(s.get("percentage", 0))
        error_id = s.get("error_id", 0)
        confidence = float(s.get("confidence", 0))
        try:
            wifi = float(s.get("signal_strength", 0))
        except (TypeError, ValueError):
            wifi = 0.0
        return {
            "battery_percent": round(battery, 1),
            "battery_level": "low" if battery < 30 else "ok" if battery < 80 else "good",
            "charging_state": POWER_SUPPLY_STATUS.get(s.get("power_supply_status"), "unknown"),
            "battery_current_A": s.get("current"),
            "battery_voltage_V": s.get("voltage"),
            "temperature_C": s.get("temperature"),
            "wifi_signal": wifi,
            "error_level": "none" if error_id == 0 else "warning" if error_id == 1 else "error",
            "emergency_stop": "pressed" if s.get("emergency") == 1 else "released",
            "localization_percent": round(confidence, 1),
            "localized": confidence >= LOCALIZED_THRESHOLD,
            "pose": {"x": s.get("x"), "y": s.get("y"), "angle": s.get("angle")},
            "linear_speed": s.get("linear_speed"),
            "angular_speed": s.get("angular_speed"),
            "motor_rpm": s.get("rpm"),
        }

    @staticmethod
    def format_summary(d: dict) -> str:
        """One-line text for a get_summary() result."""
        p = d["pose"]
        return (f"battery {d['battery_percent']}% ({d['charging_state']})  "
                f"wifi {d['wifi_signal']:g}  errors: {d['error_level']}  "
                f"e-stop: {d['emergency_stop']}  localization {d['localization_percent']}%  "
                f"pose ({p['x']:.2f}, {p['y']:.2f}, {p['angle']:.2f})  speed {d['linear_speed']}")

    def watch_status(self) -> None:
        """Print a status line about once a second until Ctrl+C."""
        try:
            for s in self.stream("robot_status"):
                print(self.format_summary(self.get_summary(s)))
        except KeyboardInterrupt:
            pass

    def get_errors(self) -> dict:
        """Current {'errors': [...], 'warnings': [...]}."""
        return self.read_once("errors_warnings")

    def clear_errors(self) -> None:
        """Clear errors and warnings (the web app's 'clear' button)."""
        self._ws_send(PORT_STATUS, "/status", {"api_id": API_CLEAR_ERRORS})

    def get_pose(self, wait: float = 3.0) -> dict:
        """Robot pose {x, y, angle, confidence, localized, source}, angle in radians.

        Uses the pose channel; if it is silent (not localized), falls back to
        the status channel, where x / y / angle are usually 0.
        """
        try:
            p = self.read_once("robot_pose", timeout=wait)
            return {"x": p["x"], "y": p["y"], "angle": p["angle"],
                    "confidence": None, "localized": True, "source": "robot_pose"}
        except ChannelSilentError:
            s = self.get_status()
            return {"x": s["x"], "y": s["y"], "angle": s["angle"],
                    "confidence": s["confidence"],
                    "localized": s["confidence"] >= LOCALIZED_THRESHOLD,
                    "source": "robot_status (pose channel silent)"}

    # -------------------------------------------------------------------------
    # 2. Maps
    # -------------------------------------------------------------------------

    def get_params(self) -> dict:
        """Robot/app parameters: robot_dimensions, lidar_offsets, zoom limits..."""
        return self._http("GET", "/params/")

    def get_live_map(self) -> dict:
        """The active map: {status, data: {header, normalPosList, advancedPointList
        (stations), advancedCurveList (paths), advancedAreaList, reflectorList}}."""
        return self._http("GET", "/map/live", timeout=60)

    def list_maps(self) -> list:
        """Every saved map as [{id, is_live, map: {...}}]. Downloads all map data."""
        r = self._http("GET", "/map/json", timeout=120)
        return r.get("data", []) if isinstance(r, dict) else r

    @staticmethod
    def summarize_map(m: dict) -> dict:
        """Name and counts for a map from list_maps() or get_live_map()['data']."""
        body = m.get("map", m)
        h = body.get("header", {})
        return {
            "id": m.get("id"),
            "name": (h.get("mapName") or "").strip(),
            "is_live": m.get("is_live"),
            "points": len(body.get("normalPosList") or []),
            "stations": len(body.get("advancedPointList") or []),
            "paths": len(body.get("advancedCurveList") or []),
            "areas": len(body.get("advancedAreaList") or []),
            "reflectors": len(body.get("reflectorList") or []),
            "min": h.get("minPos"),
            "max": h.get("maxPos"),
        }

    def map_summaries(self) -> list:
        """One summary per saved map (see summarize_map)."""
        return [self.summarize_map(m) for m in self.list_maps()]

    def get_map(self, map_id: Any) -> dict:
        """One saved map by id, as {id, is_live, map: {...}}."""
        for m in self.list_maps():
            if str(m.get("id")) == str(map_id):
                return m
        raise KeyError(f"No map with id {map_id}")

    def export_map(self, path: str, map_id: Any = None) -> str:
        """Write the live map (or saved map `map_id`) to a JSON file.

        Same format as the web app's 'Save map to local', so it loads back in the app."""
        body = self.get_live_map()["data"] if map_id is None else self.get_map(map_id)["map"]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(body, f)
        return path

    def save_map(self, map_data: dict, is_live: bool = False) -> dict:
        """Upload a map {header, normalPosList, advanced*List, reflectorList}."""
        return self._http("POST", "/map/json", json={"is_live": is_live, "map": map_data}, timeout=60)

    def set_live_map(self, map_id: Any) -> dict:
        """Make a saved map the active one."""
        return self._http("PUT", f"/map/json/{map_id}", json={"is_live": True})

    def delete_map(self, map_id: Any) -> dict:
        return self._http("DELETE", f"/map/json/{map_id}")

    def delete_maps(self, map_ids: list) -> dict:
        return self._http("DELETE", "/map/json", json={"ids": map_ids})

    # -------------------------------------------------------------------------
    # 3. Lidar
    # -------------------------------------------------------------------------

    def get_lidar_scan(self, timeout: float = 3.0) -> dict:
        """One raw scan: angle_min, angle_increment, ranges, ...
        Raises ChannelSilentError if the lidar is not sending."""
        return self.read_once("lidar", timeout=timeout)

    def lidar_to_points(self, scan: dict, pose: Optional[dict] = None,
                        use_mount_angle: bool = True) -> list:
        """Convert a scan to [(x, y), ...] the same way the web app draws it.

        Beam i points at angle_min + i * angle_increment from the lidar, which
        is mounted at params['lidar_offsets'] on the robot. With `pose`
        ({x, y, angle}) the points are in map coordinates, otherwise relative
        to the robot centre. Beams with no return (NaN, inf, 0) are dropped.
        use_mount_angle=False skips the mounting angle (the web app does this
        for "scan_2").
        """
        off = self.params.get("lidar_offsets", {})
        ox, oy = off.get("linear_offset_x", 0), off.get("linear_offset_y", 0)
        mount = off.get("angular_offset", 0) if use_mount_angle else 0
        cm, sm = math.cos(mount), math.sin(mount)
        if pose:
            cp, sp = math.cos(pose["angle"]), math.sin(pose["angle"])
        range_max = scan.get("range_max")
        a0, da = scan.get("angle_min", 0.0), scan.get("angle_increment", 0.0)

        points = []
        for i, r in enumerate(scan.get("ranges") or []):
            if r is None or not math.isfinite(r) or r <= 0 or (range_max and r > range_max):
                continue
            a = a0 + i * da
            lx, ly = ox + r * math.cos(a), oy + r * math.sin(a)
            x, y = lx * cm - ly * sm, lx * sm + ly * cm                  # lidar mounting angle
            if pose:
                x, y = pose["x"] + x * cp - y * sp, pose["y"] + x * sp + y * cp  # robot -> map
            points.append((x, y))
        return points

    def get_lidar_points(self, in_map_frame: bool = True, timeout: float = 3.0) -> list:
        """Latest scan as [(x, y), ...], in map coordinates by default."""
        scan = self.get_lidar_scan(timeout=timeout)
        return self.lidar_to_points(scan, self.get_pose() if in_map_frame else None)

    # -------------------------------------------------------------------------
    # 4. Localization
    # -------------------------------------------------------------------------

    def auto_relocate(self) -> Any:
        """Let the robot find its own position on the live map."""
        return self._http("POST", "/reloc/auto", port=PORT_CONTROL, timeout=60)

    def relocate(self, x: float, y: float, theta: float) -> Optional[dict]:
        """Tell the robot where it is (the web app's Relocation button). theta in radians."""
        return self._ws_request(PORT_CONTROL, "/tele_op",
                                {"api_id": API_RELOCATE, "reloc": {"x": x, "y": y, "theta": theta}})

    def relocate_rest(self, x: float, y: float, theta: float) -> Any:
        """Same as relocate(), over HTTP. The Mapping page uses this after saving."""
        return self._http("POST", "/reloc", port=PORT_CONTROL, timeout=60,
                          json={"x": x, "y": y, "z": 0, "theta": theta})

    # -------------------------------------------------------------------------
    # 5. Navigation  (MOVES THE ROBOT)
    # -------------------------------------------------------------------------

    def navigate_to_point(self, x: float, y: float, theta: float) -> Optional[dict]:
        """Drive to a map point. MOVES THE ROBOT. Reply: {status, message, error_code}."""
        # The web app sends the goal in fields named linear_x / linear_y / angular_z.
        return self._ws_request(PORT_CONTROL, "/nav", {
            "api_id": API_FREE_NAV,
            "free_nav": {"linear_x": x, "linear_y": y, "angular_z": theta}})

    def navigate_to_station(self, station: str, previous_node: str = "-1") -> Optional[dict]:
        """Drive along the paths to a station such as "LM1". MOVES THE ROBOT."""
        return self._ws_request(PORT_CONTROL, "/nav", {
            "api_id": API_PATH_NAV,
            "path_nav": {"target_node": station, "previous_node": previous_node}})

    def cancel_navigation(self) -> Optional[dict]:
        """Stop the current navigation goal."""
        return self._ws_request(PORT_CONTROL, "/nav", {"api_id": API_CANCEL_NAV, "action": "cancel"})

    # -------------------------------------------------------------------------
    # 6. Mapping (building a new map)
    # -------------------------------------------------------------------------

    def start_mapping(self) -> Any:
        """Start building a new map. Then drive around to cover the area."""
        return self._http("POST", "/mapping/start", port=PORT_CONTROL, timeout=60)

    def stop_mapping(self) -> Any:
        return self._http("POST", "/mapping/stop", port=PORT_CONTROL, timeout=60)

    def get_mapping_data(self, timeout: Optional[float] = None) -> dict:
        """The map being built: {map: {header, normalPosList}, path: [...]}."""
        return self.read_once("mapping", timeout=timeout)

    def auto_align_angle(self) -> float:
        """Angle (radians) that squares the new map to the axes; 0 on failure."""
        try:
            return float(self._http("POST", "/align/map/auto", port=PORT_CONTROL, timeout=60)["angle"])
        except Exception:
            return 0.0

    def save_mapping(self, is_live: bool = False, align: bool = True,
                     name: Optional[str] = None, relocate_after: bool = True) -> dict:
        """Save the map being built, exactly like the Mapping page's Save button.

        Reads the new map, straightens it (if align and <= 50,000 points),
        saves it (is_live=True is the app's "Live" option, False is "Normal"),
        stops mapping, then relocates the robot into the saved map.
        """
        m = self.get_mapping_data()["map"]
        points = m.get("normalPosList") or []
        if not points:
            raise ValueError("The new map has no points yet. Start mapping and drive around first.")

        try:
            flat = self.read_once("reflectors", timeout=2) or []
            reflectors = [{"x": flat[i], "y": flat[i + 1]} for i in range(0, len(flat) - 1, 2)]
        except ChannelSilentError:
            reflectors = []
        pose = self.get_pose()

        angle = self.auto_align_angle() if (align and len(points) <= 50000) else 0.0
        header = dict(m["header"])
        if angle:
            points = [rotate_point(p, angle) for p in points]
            reflectors = [rotate_point(p, angle) for p in reflectors]
            header["minPos"] = rotate_point(header["minPos"], angle)
            header["maxPos"] = rotate_point(header["maxPos"], angle)

        now = datetime.now()
        header["mapType"] = "2D-Map"
        header["mapName"] = name or f"Map {now.hour}:{now.minute} {now.strftime('%B')} {now.day}  "

        result = self._http("POST", "/map/json", timeout=120, json={
            "is_live": is_live,
            "map": {"header": header, "normalPosList": points, "reflectorList": reflectors},
        })

        if isinstance(result, dict) and result.get("status") is True:
            self.stop_mapping()
            if relocate_after:
                p = rotate_point(pose, angle)
                # The web app keeps the un-rotated heading here; copied as-is.
                for delay in (1, 4):
                    time.sleep(delay)
                    self.relocate_rest(p["x"], p["y"], pose["angle"])
        return result

    # -------------------------------------------------------------------------
    # 7. Tele-op  (MOVES THE ROBOT)
    # -------------------------------------------------------------------------

    def drive(self, linear_x: float, angular_z: float, duration: float, rate_hz: float = 10.0) -> None:
        """Drive for `duration` seconds (m/s, rad/s), then stop. MOVES THE ROBOT.

        Re-sends the command 10 times a second like the web joystick and always
        sends a stop at the end, even after an error. The app caps speed at 1.0.
        """
        ws = self._ws_open(PORT_CONTROL, "/tele_op")

        def send(lx: float, az: float) -> None:
            ws.send(self._payload({"api_id": API_TELEOP, "tele_op": True,
                                   "controls": {"linear_x": lx, "linear_y": 0, "angular_z": az}}))

        try:
            end = time.time() + duration
            while time.time() < end:
                send(linear_x, angular_z)
                time.sleep(1.0 / rate_hz)
        finally:
            try:
                send(0, 0)
                send(0, 0)
            finally:
                ws.close()

    # -------------------------------------------------------------------------
    # 8. Tasks, missions, sounds
    # -------------------------------------------------------------------------

    def get_tasks(self, name: str = "default") -> dict:
        """Saved task chains."""
        return self._http("GET", "/tasks/", params={"name": name})

    def save_tasks(self, tasks: list, name: str = "default") -> dict:
        return self._http("POST", "/tasks/", json={"tasks": tasks, "name": name})

    def mission_status(self, max_messages: int = 1) -> Iterator[Any]:
        """Task-chain progress: items {Task_name, Task_status} (see TASK_STATUS)."""
        ws = self._ws_open(PORT_MISSION, "/")
        try:
            ws.send(self._payload({"api_id": API_MISSION_STATUS, "action": "status"}))
            for _ in range(max_messages):
                yield json.loads(ws.recv())
        finally:
            ws.close()

    def get_sounds(self) -> Any:
        """Sound list (this socket expects the api id without a 'request' wrapper)."""
        ws = self._ws_open(PORT_SOUND)
        try:
            ws.send(json.dumps({"key": API_KEY, "api_id": API_SOUND_LIST}))
            while True:
                msg = json.loads(ws.recv())
                if msg.get("api_id") == API_SOUND_LIST and msg.get("status") is True:
                    return msg.get("data")
        finally:
            ws.close()