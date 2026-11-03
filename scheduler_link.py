"""
scheduler_link.py - connects AMR Agent to the Task Scheduler.

The Task Scheduler (github.com/raghulrajg/Task-Scheduler-Agent) plans each job
and sends the AMR its "navigate" steps over MQTT. This module is the AMR's side
of that conversation: it follows each instruction on the robot and reports
back, using the scheduler's robot-agent contract (scheduler README, 8.5 and 9):

  robots/<id>/amr/task    scheduler -> AMR   {"type": "task", "subtask_id", "attempt", "action": "navigate",
                                              "zone": <location>, "expires_at", ...}
                                             {"type": "cancel", "subtask_id", "attempt", "reason"}
  robots/<id>/amr/status  AMR -> scheduler   {"subtask_id", "attempt", "state": "accepted" | "done" | "failed" |
                                              "cancelled", "result"?, "error"?}   (final states are retained)
  robots/<id>/amr/state   AMR -> everyone    heartbeat every 5 s, retained; the last will says offline
  robots/<id>/+/state     arm -> AMR         the arm's heartbeat, for the interlock

In TEST mode (--test) every topic starts with "sim/", like the scheduler's.

A navigate step:
  1. Read the location's x, y, theta (and tolerances) from the scheduler's
     stations.json, fresh for every move.
  2. Refuse if the robot is e-stopped, not localized, in error or already
     moving, or if the arm is busy or not in its travel pose.
  3. Send the goal and follow the robot's live status. Report "done" only when
     the robot is VERIFIED at the place: within tolerance, stopped and still
     localized for settle_s. Fail (and stop the robot) on an e-stop, lost
     localization, a lasting robot error, no start, a stall, lost contact, a
     timeout, or a cancel from the scheduler.

Only one program may serve a robot's AMR. This one does not start while
another agent (e.g. the scheduler's own "launch.py robot robot_01") is sending
heartbeats on robots/<id>/amr/state, and it stops if one appears later.

Run:  python amr_agent.py scheduler --stations <Task-Scheduler-Agent folder>/stations.json
"""

from __future__ import annotations

import copy
import json
import logging
import math
import os
import signal
import ssl
import threading
import time
import uuid
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple

import paho.mqtt.client as mqtt

from amr_client import API_KEY, PORT_CONTROL, PORT_REST, PORT_STATUS, AMRClient, remembered_host

logger = logging.getLogger("scheduler_link")

HERE = os.path.dirname(os.path.abspath(__file__))
AGENT_NAME = "amr_agent"
TERMINAL = ("done", "failed", "cancelled")
RESULT_CACHE = 500
ERROR_LEVEL_ERROR = 2        # robot error_id: 0 none, 1 warning, 2+ error
FOREIGN_FRESH_S = 15.0       # another agent's heartbeat younger than this counts as alive (scheduler: heartbeat_timeout_s)
FOREIGN_WATCH_S = 16.0       # how long to watch for that agent's next heartbeat before taking over
RETAINED_WAIT_S = 1.5        # after connecting: time for the broker to deliver the retained heartbeat

# Same settings and defaults as the scheduler's agents/robot_01_amr.json, so
# that file can be passed with --config unchanged.
DEFAULTS: Dict[str, Any] = {
    "robot_id": "robot_01",
    "subsystem": "amr",
    "heartbeat_s": 5,
    "interlock_with": ["arm"],         # the arm on the base must be online and idle before the base drives
    "require_travel_pose": ["arm"],    # ... and must report its tucked-in travel pose
    "clock_skew_tolerance_s": 5,
    "driver": {
        "stations_file": "stations.json",
        "nav_mode": "point",           # "point" = drive to x, y, theta | "station" = path nav to the linked station
        "xy_tolerance_m": 0.10,        # stations.json defaults / per-location values override these
        "yaw_tolerance_deg": 5,
        "settle_s": 1.5,               # in tolerance, stopped and localized this long = arrived
        "stationary_linear": 0.02,     # m/s below which the robot counts as stopped
        "stationary_angular": 0.02,    # rad/s
        "start_timeout_s": 15,
        "stall_timeout_s": 30,
        "mission_timeout_s": 280,
        "max_comm_loss_s": 15,
        "min_localization": 70,
        "error_grace_s": 3,
        "arrival_drift_m": 0.05,       # after arriving, still "at" the location while within this drift
        "arrival_drift_deg": 3,
        "request_timeout_s": 5,
        "ports": {"rest": PORT_REST, "control": PORT_CONTROL, "status": PORT_STATUS},
    },
}


def done(result: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return {"state": "done", **({"result": result} if result is not None else {})}


def failed(error: str) -> Dict[str, Any]:
    return {"state": "failed", "error": error}


def cancelled(reason: str = "cancelled") -> Dict[str, Any]:
    return {"state": "cancelled", "error": reason}


# =============================================================================
# Broker settings (same variable names as the scheduler's config.py / .env)
# =============================================================================

def _flag(raw: Optional[str], default: bool) -> bool:
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


class BrokerSettings:
    """MQTT broker, run mode and topic namespace, read like the scheduler reads
    them, so the scheduler's .env works here unchanged."""

    def __init__(self, env: Optional[Dict[str, str]] = None, test_mode: bool = False):
        env = os.environ if env is None else env
        mode = env.get("RUN_MODE", "real").strip().lower()
        if mode not in ("real", "test"):
            raise ValueError(f"RUN_MODE must be 'real' or 'test', not {mode!r}")
        self.test_mode = test_mode or mode == "test"
        prefix = env.get("MQTT_TOPIC_PREFIX", "sim/" if self.test_mode else "")
        if self.test_mode and not prefix:
            raise ValueError("TEST mode needs a non-empty MQTT_TOPIC_PREFIX (it keeps simulated jobs off real topics)")
        if prefix and not prefix.endswith("/"):
            prefix += "/"
        self.prefix = prefix
        self.client_suffix = env.get("MQTT_CLIENT_ID_SUFFIX", "-test" if self.test_mode else "")
        self.host = env.get("MQTT_BROKER_HOST", "localhost")
        self.port = int(env.get("MQTT_BROKER_PORT", "1883"))
        self.username = env.get("MQTT_USERNAME") or None
        self.password = env.get("MQTT_PASSWORD") or None
        self.keepalive = int(env.get("MQTT_KEEPALIVE_S", "30"))
        self.tls = _flag(env.get("MQTT_TLS"), False)
        self.tls_ca_cert = env.get("MQTT_TLS_CA_CERT") or None
        self.tls_client_cert = env.get("MQTT_TLS_CLIENT_CERT") or None
        self.tls_client_key = env.get("MQTT_TLS_CLIENT_KEY") or None

    def topic(self, name: str) -> str:
        return self.prefix + name

    def describe(self) -> str:
        if self.test_mode:
            return f"TEST mode: topics under '{self.prefix}', client ids end with '{self.client_suffix}'"
        return "REAL mode: topics " + (f"under '{self.prefix}'" if self.prefix else "at the root")

    def make_client(self, client_id: str, clean_session: bool = False, will: Optional[Tuple[str, dict]] = None):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id + self.client_suffix,
                             clean_session=clean_session)
        client.suppress_exceptions = True       # a bad message must not kill paho's network thread
        if self.username:
            client.username_pw_set(self.username, self.password)
        if self.tls:
            client.tls_set(ca_certs=self.tls_ca_cert, certfile=self.tls_client_cert, keyfile=self.tls_client_key,
                           cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        if will:
            client.will_set(will[0], json.dumps(will[1]), qos=1, retain=True)
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        return client


def load_env_file(path: str) -> bool:
    """Load KEY=VALUE settings from a .env file. Real environment variables win."""
    if not os.path.isfile(path):
        return False
    try:
        from dotenv import load_dotenv
    except ImportError:
        logger.warning("python-dotenv is not installed, so %s was not read: pip install -r requirements.txt", path)
        return False
    load_dotenv(path, override=False)
    return True


# =============================================================================
# Locations (the scheduler's stations.json, maintained with its tools/amr_stations.py)
# =============================================================================

class LocationError(ValueError):
    """The location is unknown or has no usable coordinates."""


def load_stations(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("the top level must be a JSON object")
    data.setdefault("defaults", {})
    data.setdefault("stations", {})
    data.setdefault("locations", {})
    return data


def _num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def has_pose(entry: Any) -> bool:
    return isinstance(entry, dict) and all(_num(entry.get(k)) for k in ("x", "y", "theta"))


def assigned_locations(data: Dict[str, Any]) -> List[str]:
    """Location names that have a complete x, y, theta."""
    return sorted(n for n, e in data.get("locations", {}).items() if has_pose(e))


def location_pose(data: Dict[str, Any], name: Any, xy_tol: float, yaw_tol_deg: float
                  ) -> Tuple[float, float, float, float, float]:
    """(x, y, theta, xy tolerance m, yaw tolerance rad). Tolerances: the
    location's own, else the file's defaults, else the given ones."""
    entry = data.get("locations", {}).get(name)
    if entry is None:
        raise LocationError(f"location {name!r} is not in the stations file")
    if not has_pose(entry):
        raise LocationError(f"location {name!r} has no x/y/theta yet -- assign or teach it with the scheduler's "
                            f"tools/amr_stations.py (assign {name} <station>, or teach {name})")
    d = data.get("defaults") or {}
    xy = float(entry.get("xy_tolerance_m", d.get("xy_tolerance_m", xy_tol)))
    yaw = math.radians(float(entry.get("yaw_tolerance_deg", d.get("yaw_tolerance_deg", yaw_tol_deg))))
    return float(entry["x"]), float(entry["y"]), float(entry["theta"]), xy, yaw


def wrap_angle(a: float) -> float:
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def pose_error(x: float, y: float, theta: float, gx: float, gy: float, gtheta: float) -> Tuple[float, float]:
    """(distance in m, absolute heading error in rad)."""
    return math.hypot(x - gx, y - gy), abs(wrap_angle(theta - gtheta))


def locations_at(data: Dict[str, Any], x: float, y: float, theta: float, xy_tol: float, yaw_tol_deg: float) -> List[str]:
    """Every location the robot is within tolerance of, nearest first."""
    hits = []
    for name in assigned_locations(data):
        gx, gy, gt, xt, yt = location_pose(data, name, xy_tol, yaw_tol_deg)
        d, yaw = pose_error(x, y, theta, gx, gy, gt)
        if d <= xt and yaw <= yt:
            hits.append((d, name))
    return [name for _, name in sorted(hits)]


# =============================================================================
# The robot: AMRClient on the configured ports / key
# =============================================================================

class RobotAPI(AMRClient):
    """AMRClient that uses the ports and API key from the settings. The real
    robot always uses 3000/4000/7000; a simulator may use others."""

    def __init__(self, host: str, ports: Optional[Dict[str, int]] = None, api_key: Optional[str] = None,
                 timeout: float = 5.0):
        super().__init__(host, timeout=timeout)
        p = ports or {}
        self._port_map = {PORT_REST: int(p.get("rest", PORT_REST)),
                          PORT_CONTROL: int(p.get("control", PORT_CONTROL)),
                          PORT_STATUS: int(p.get("status", PORT_STATUS))}
        self._key = api_key or API_KEY

    def _ws_open(self, port: int, path: str = "", timeout: Optional[float] = None):
        return super()._ws_open(self._port_map.get(port, port), path, timeout)

    def _http(self, method: str, path: str, port: int = PORT_REST, **kwargs) -> Any:
        return super()._http(method, path, self._port_map.get(port, port), **kwargs)

    def _payload(self, request: dict) -> str:      # instead of AMRClient's fixed key
        return json.dumps({"key": self._key, "request": request})


# =============================================================================
# Navigation: drive to a location and verify the arrival
# =============================================================================

class Navigator:
    """Turns one "navigate" step into robot commands and blocks until the robot
    is verified at the place, has failed, or the step was cancelled."""

    actions = ("navigate",)

    def __init__(self, robot: Any, stations_file: str, cfg: Dict[str, Any]):
        self.robot = robot
        self.stations_file = stations_file
        self.mode = cfg.get("nav_mode", "point")
        if self.mode not in ("point", "station"):
            raise ValueError("nav_mode must be 'point' or 'station'")
        self.xy_tol = float(cfg.get("xy_tolerance_m", 0.10))
        self.yaw_tol_deg = float(cfg.get("yaw_tolerance_deg", 5))
        self.settle = float(cfg.get("settle_s", 1.5))
        self.still_lin = float(cfg.get("stationary_linear", 0.02))
        self.still_ang = float(cfg.get("stationary_angular", 0.02))
        self.start_timeout = float(cfg.get("start_timeout_s", 15))
        self.stall_timeout = float(cfg.get("stall_timeout_s", 30))
        self.mission_timeout = float(cfg.get("mission_timeout_s", 280))
        self.max_comm_loss = float(cfg.get("max_comm_loss_s", 15))
        self.min_loc = float(cfg.get("min_localization", 70))
        self.error_grace = float(cfg.get("error_grace_s", 3))
        self.drift_m = float(cfg.get("arrival_drift_m", 0.05))
        self.drift_rad = math.radians(float(cfg.get("arrival_drift_deg", 3)))
        self.status_timeout = float(cfg.get("status_timeout_s", 3))
        self._lock = threading.Lock()
        self._active = False
        self._arrived: Optional[Dict[str, Any]] = None    # last verified arrival {"zone", "x", "y", "theta"}

    # ---- helpers ----------------------------------------------------------------------
    def _stations(self) -> Dict[str, Any]:
        return load_stations(self.stations_file)

    def _status(self, timeout: float) -> Dict[str, Any]:
        st = self.robot.read_once("robot_status", timeout=timeout)
        if not isinstance(st, dict):
            raise ValueError(f"unexpected status message {st!r}")
        return st

    def _errors_text(self) -> str:
        try:
            e = self.robot.read_once("errors_warnings", timeout=2) or {}
            parts = [str(x) for x in (e.get("errors") or [])] + [f"warning: {x}" for x in (e.get("warnings") or [])]
            return f" [{'; '.join(parts)[:300]}]" if parts else ""
        except Exception:
            return ""

    def _moving(self, st: Dict[str, Any]) -> bool:
        return abs(float(st.get("linear_speed") or 0)) > self.still_lin or \
            abs(float(st.get("angular_speed") or 0)) > self.still_ang

    def _abort(self, why: str) -> Dict[str, Any]:
        try:
            self.robot.cancel_navigation()
        except Exception as e:
            logger.error("Cancelling the robot's goal failed: %s", e)
        return failed(why + self._errors_text())

    @staticmethod
    def _close(feed) -> None:
        if feed is not None:
            try:
                feed.close()
            except Exception:
                pass

    # ---- one step ---------------------------------------------------------------------
    def execute(self, task: Dict[str, Any], cancel: threading.Event) -> Dict[str, Any]:
        zone = task.get("zone")
        with self._lock:
            self._arrived = None      # from here on, only a new verified arrival counts
        try:
            data = self._stations()
            gx, gy, gt, xy_tol, yaw_tol = location_pose(data, zone, self.xy_tol, self.yaw_tol_deg)
        except FileNotFoundError:
            return failed(f"stations file {self.stations_file} not found")
        except LocationError as e:
            return failed(str(e))
        except ValueError as e:
            return failed(f"stations file {self.stations_file} is not valid: {e}")
        station = data["locations"][zone].get("station")
        if self.mode == "station" and not station:
            return failed(f"nav_mode 'station' but location {zone!r} is not linked to a station")

        try:
            st = self._status(timeout=5)
        except Exception as e:
            return failed(f"cannot read the AMR's status: {e}")
        if st.get("emergency") == 1:
            return failed("AMR emergency stop is pressed")
        conf = float(st.get("confidence") or 0)
        if conf < self.min_loc:
            return failed(f"AMR not localized ({conf:.0f}% < {self.min_loc:.0f}%) -- relocate it in its web app first")
        if int(st.get("error_id") or 0) >= ERROR_LEVEL_ERROR:
            return failed("AMR reports an error" + self._errors_text())
        if self._moving(st):
            return failed("AMR is already moving (driven from its web app?)")

        target = f"{zone} (x={gx:.3f}, y={gy:.3f}, theta={math.degrees(gt):.1f} deg" + \
                 (f", station {station})" if station else ")")
        try:
            reply = (self.robot.navigate_to_station(station) if self.mode == "station"
                     else self.robot.navigate_to_point(gx, gy, gt))
        except Exception as e:
            return failed(f"sending the goal to the AMR failed: {e}")
        if isinstance(reply, dict) and reply.get("status") is False:
            return failed(f"AMR rejected the goal {target}: {reply.get('message')} "
                          f"(error_code {reply.get('error_code')})")
        if reply is None:
            logger.warning("No reply to the goal; checking that the robot moves instead")
        logger.info("Driving to %s", target)
        with self._lock:
            self._active = True
        try:
            outcome = self._follow(gx, gy, gt, xy_tol, yaw_tol, target, cancel)
        finally:
            with self._lock:
                self._active = False
        if outcome.get("state") == "done":
            r = outcome["result"]
            with self._lock:
                self._arrived = {"zone": zone, "x": r["x"], "y": r["y"], "theta": r["theta"]}
        return outcome

    def _follow(self, gx, gy, gt, xy_tol, yaw_tol, target, cancel) -> Dict[str, Any]:
        started = last_ok = last_motion = time.time()
        seen_motion = False
        settled_since = error_since = None
        feed = None
        try:
            while True:
                if cancel.is_set():
                    self._abort("cancelled")
                    return cancelled()
                now = time.time()
                if now - started > self.mission_timeout:
                    return self._abort(f"did not reach {target} within {self.mission_timeout:.0f}s")
                try:
                    if feed is None:
                        feed = self.robot.stream("robot_status", timeout=self.status_timeout)
                    st = next(feed)
                    last_ok = now = time.time()
                except Exception as e:     # status channel lost: reconnect, give up after max_comm_loss_s
                    self._close(feed)
                    feed = None
                    if time.time() - last_ok > self.max_comm_loss:
                        return self._abort(f"lost contact with the AMR for >{self.max_comm_loss:.0f}s: {e}")
                    cancel.wait(0.5)
                    continue
                if not isinstance(st, dict):
                    continue

                if st.get("emergency") == 1:
                    return self._abort(f"emergency stop pressed while driving to {target}")
                conf = float(st.get("confidence") or 0)
                if conf < self.min_loc:
                    return self._abort(f"localization lost ({conf:.0f}%) while driving to {target}")
                if int(st.get("error_id") or 0) >= ERROR_LEVEL_ERROR:
                    error_since = error_since or now
                    if now - error_since >= self.error_grace:
                        return self._abort(f"AMR error while driving to {target}")
                else:
                    error_since = None

                x, y, th = float(st.get("x") or 0), float(st.get("y") or 0), float(st.get("angle") or 0)
                dist, yaw = pose_error(x, y, th, gx, gy, gt)
                moving = self._moving(st)
                if moving:
                    last_motion, seen_motion = now, True
                at_goal = dist <= xy_tol and yaw <= yaw_tol

                if at_goal and not moving:
                    settled_since = settled_since or now
                    if now - settled_since >= self.settle:
                        logger.info("Verified at %s: error %.3f m / %.1f deg", target, dist, math.degrees(yaw))
                        return done({"x": round(x, 4), "y": round(y, 4), "theta": round(th, 4),
                                     "error_m": round(dist, 4), "error_deg": round(math.degrees(yaw), 2),
                                     "verified": True})
                else:
                    settled_since = None

                if not moving and not at_goal:
                    if not seen_motion and now - started > self.start_timeout:
                        return self._abort(f"AMR did not start moving towards {target} within "
                                           f"{self.start_timeout:.0f}s (goal not accepted?)")
                    if seen_motion and now - last_motion > self.stall_timeout:
                        return self._abort(f"AMR stopped {dist:.2f} m / {math.degrees(yaw):.1f} deg away from "
                                           f"{target} for {self.stall_timeout:.0f}s (blocked, or tolerance "
                                           f"tighter than the robot's own goal tolerance?)")
        finally:
            self._close(feed)

    def cancel(self) -> None:
        """Stop the robot now (called from the MQTT thread when a cancel arrives)."""
        with self._lock:
            active = self._active
        if active:
            try:
                self.robot.cancel_navigation()
            except Exception as e:
                logger.error("Cancelling the robot's goal failed: %s", e)

    # ---- heartbeat ----------------------------------------------------------------------
    def get_state(self) -> Dict[str, Any]:
        """What the heartbeat reports: ready, e-stop, battery, pose, where the
        robot is (at_location) and which locations have coordinates."""
        try:
            st = self._status(timeout=3)
        except Exception as e:
            return {"ready": False, "detail": f"cannot reach the AMR at {self.robot.host}: {e}",
                    "estop": False, "battery": None}
        conf = float(st.get("confidence") or 0)
        estop = st.get("emergency") == 1
        err = int(st.get("error_id") or 0) >= ERROR_LEVEL_ERROR
        x, y, th = float(st.get("x") or 0), float(st.get("y") or 0), float(st.get("angle") or 0)
        try:
            data = self._stations()
            locations = assigned_locations(data)
            here = locations_at(data, x, y, th, self.xy_tol, self.yaw_tol_deg) if conf >= self.min_loc else []
        except (OSError, ValueError):
            locations, here = [], []
        with self._lock:
            active, arrived = self._active, self._arrived
        if arrived and conf >= self.min_loc and arrived["zone"] in locations:
            d, yaw = pose_error(x, y, th, arrived["x"], arrived["y"], arrived["theta"])
            if d <= self.drift_m and yaw <= self.drift_rad:
                # still where the last move was verified: that is the location we are "at",
                # even if another location name shares the spot
                here = [arrived["zone"]] + [z for z in here if z != arrived["zone"]]
        at = here[0] if here else None
        ready = conf >= self.min_loc and not estop and not err and (active or not self._moving(st))
        problems = [p for p, bad in (("NOT localized", conf < self.min_loc), ("e-stop", estop), ("error", err)) if bad]
        detail = (f"loc {conf:.0f}% pose ({x:.2f}, {y:.2f}, {math.degrees(th):.0f} deg) at {at or '-'}"
                  + (f" [{', '.join(problems)}]" if problems else ""))
        battery = st.get("percentage")
        return {"ready": bool(ready), "detail": detail, "estop": bool(estop),
                "battery": float(battery) if _num(battery) else None,
                "at_location": at, "at_locations": here, "locations": locations,
                "pose": {"x": round(x, 3), "y": round(y, 3), "theta": round(th, 3)}}


# =============================================================================
# The MQTT side: the scheduler's robot-agent contract
# =============================================================================

HEARTBEAT_EXTRAS = ("at_location", "at_locations", "locations", "pose")


class AnotherAgentRunning(RuntimeError):
    """Another program already serves this robot's AMR topics."""


class SchedulerLink:
    """Takes the scheduler's steps for one robot's AMR, runs them with a
    Navigator and reports results and heartbeats."""

    def __init__(self, navigator: Navigator, broker: BrokerSettings, robot_id: str = "robot_01",
                 subsystem: str = "amr", heartbeat_s: float = 5.0, interlock_with: Tuple[str, ...] = ("arm",),
                 require_travel_pose: Tuple[str, ...] = ("arm",), clock_skew_tolerance_s: float = 5.0,
                 client: Any = None):
        self.navigator, self.broker = navigator, broker
        self.robot_id, self.subsystem = robot_id, subsystem
        self.heartbeat_s = float(heartbeat_s)
        self.interlock_with = tuple(interlock_with)
        self.require_travel_pose = tuple(require_travel_pose)
        unknown = set(self.require_travel_pose) - set(self.interlock_with)
        if unknown:
            raise ValueError(f"require_travel_pose {sorted(unknown)} must also be listed in interlock_with")
        self.skew = float(clock_skew_tolerance_s)
        self.instance = uuid.uuid4().hex[:12]
        self.task_topic = broker.topic(f"robots/{robot_id}/{subsystem}/task")
        self.status_topic = broker.topic(f"robots/{robot_id}/{subsystem}/status")
        self.state_topic = broker.topic(f"robots/{robot_id}/{subsystem}/state")
        self.state_filter = broker.topic(f"robots/{robot_id}/+/state")

        self._lock = threading.RLock()
        self._current: Optional[Dict[str, Any]] = None        # {"key", "thread", "cancel", "subtask"}
        self._results: "OrderedDict[Tuple[str, Any], dict]" = OrderedDict()
        self._siblings: Dict[str, Dict[str, Any]] = {}
        self._driver_state: Dict[str, Any] = {}
        self._stop = threading.Event()
        self._serving = threading.Event()       # set once no other agent serves this AMR
        self._foreign: Optional[Dict[str, Any]] = None   # another agent's heartbeat seen by the startup check
        self._conflict: Optional[str] = None
        self._connected = threading.Event()

        self._client = client if client is not None else broker.make_client(
            f"{AGENT_NAME}_{robot_id}_{subsystem}", clean_session=False,
            will=(self.state_topic, {"online": False, "ready": False, "detail": "agent connection lost (last will)",
                                     "agent": AGENT_NAME, "instance": self.instance}))
        self._client.on_connect = self._guarded(self._on_connect)
        self._client.on_disconnect = self._guarded(self._on_disconnect)
        self._client.on_message = self._guarded(self._on_message)

    @staticmethod
    def _guarded(fn):
        def wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except Exception:
                logger.exception("Error in MQTT callback %s", fn.__name__)
        return wrapper

    # ---- MQTT callbacks -------------------------------------------------------------------
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if getattr(reason_code, "is_failure", False):
            logger.error("Broker refused the connection: %s (check MQTT_USERNAME / MQTT_PASSWORD)", reason_code)
            return
        logger.info("Connected to the broker; listening on %s", self.task_topic)
        client.subscribe([(self.task_topic, 1), (self.state_filter, 1)])
        self._connected.set()
        if self._serving.is_set():
            self._publish_heartbeat(refresh=False)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self._connected.clear()
        if not self._stop.is_set():
            logger.warning("Lost the broker (%s); reconnecting", reason_code)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            logger.warning("Ignoring a non-JSON message on %s: %r", msg.topic, msg.payload[:200])
            return
        if msg.topic == self.task_topic:
            if not isinstance(payload, dict):
                logger.error("Ignoring a malformed task message: %r", msg.payload[:200])
                return
            self._dispatch(payload)
            return
        if not isinstance(payload, dict) or not msg.topic.startswith(self.broker.prefix):
            return
        parts = msg.topic[len(self.broker.prefix):].split("/")
        if len(parts) != 4 or parts[0] != "robots" or parts[1] != self.robot_id or parts[3] != "state":
            return
        if parts[2] == self.subsystem:
            self._check_foreign(payload, bool(getattr(msg, "retain", False)))
        else:
            self._siblings[parts[2]] = payload

    # ---- only one agent per AMR ------------------------------------------------------------
    def _alive_foreign(self, payload: Any) -> bool:
        """A recent "online" heartbeat on our state topic from someone else."""
        if not isinstance(payload, dict) or payload.get("instance") == self.instance or not payload.get("online"):
            return False        # not a heartbeat, our own, or an offline / last-will message
        ts = payload.get("ts")
        return not (_num(ts) and time.time() - ts > FOREIGN_FRESH_S)    # older ones are left over

    def _check_foreign(self, payload: Dict[str, Any], retained: bool) -> None:
        """While serving: another agent started publishing on our state topic."""
        if retained or not self._alive_foreign(payload):
            return      # a retained copy comes with every (re)subscribe; a running agent also sends live ones
        if payload.get("agent") == AGENT_NAME and str(payload.get("instance", "")) < self.instance:
            return      # two AMR Agents: the one with the lower instance id stands down
        with self._lock:
            if self._conflict:
                return
            self._conflict = (f"{self._who(payload)} started serving {self.state_topic} while this one was running. "
                              f"Only one program may drive the AMR, so this one stops. {self._advice(payload)}")
            current = self._current
        logger.error(self._conflict)
        if current:
            current["cancel"].set()
            self.navigator.cancel()
        self._stop.set()

    def _note_probe(self, msg) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return
        if not self._alive_foreign(payload):
            return
        retained = bool(getattr(msg, "retain", False))
        with self._lock:
            if not (self._foreign and not self._foreign["retained"]):     # a live one beats a retained one
                self._foreign = {"payload": payload, "retained": retained}

    def _wait_until_alone(self, probe: Any = None) -> None:
        """Before connecting: make sure no other agent is already driving this AMR.
        Uses a separate read-only connection, so a second copy of this program
        never knocks the running one off the broker (same client id)."""
        connected = threading.Event()
        probe = probe if probe is not None else self.broker.make_client(
            f"{AGENT_NAME}_{self.robot_id}_{self.subsystem}_check_{self.instance}", clean_session=True)

        def on_connect(client, userdata, flags, reason_code, properties=None):
            if getattr(reason_code, "is_failure", False):
                logger.error("Broker refused the connection: %s (check MQTT_USERNAME / MQTT_PASSWORD)", reason_code)
                return
            client.subscribe(self.state_topic, 1)
            connected.set()

        probe.on_connect = self._guarded(on_connect)
        probe.on_message = self._guarded(lambda client, userdata, msg: self._note_probe(msg))
        probe.connect_async(self.broker.host, self.broker.port, keepalive=self.broker.keepalive)
        probe.loop_start()
        try:
            while not connected.wait(10):
                if self._stop.is_set():
                    return
                logger.warning("Waiting for the broker at %s:%s ...", self.broker.host, self.broker.port)
            if self._stop.wait(RETAINED_WAIT_S):
                return
            with self._lock:
                foreign = self._foreign
            if foreign is None:
                return
            if not foreign["retained"]:
                raise AnotherAgentRunning(self._refusal(foreign["payload"]))
            logger.warning("%s holds a recent heartbeat from another agent. Watching %.0fs for it before "
                           "taking over...", self.state_topic, FOREIGN_WATCH_S)
            end = time.time() + FOREIGN_WATCH_S
            while time.time() < end:
                if self._stop.wait(0.2):
                    return
                with self._lock:
                    foreign = self._foreign
                if not foreign["retained"]:
                    raise AnotherAgentRunning(self._refusal(foreign["payload"]))
            logger.info("No heartbeat from it since: it is not running any more. Taking over.")
        finally:
            try:
                probe.disconnect()
            finally:
                probe.loop_stop()

    def _who(self, payload: Dict[str, Any]) -> str:
        if payload.get("agent") == AGENT_NAME:
            return "Another AMR Agent"
        return f"Another agent (most likely the scheduler's own, from launch.py robot {self.robot_id})"

    def _advice(self, payload: Dict[str, Any]) -> str:
        if payload.get("agent") == AGENT_NAME:
            return "Stop the other AMR Agent first."
        return (f"To use AMR Agent for the AMR, start only the arm from the scheduler on the robot PC "
                f"(python3 launch.py agents agents/{self.robot_id}_arm.json) instead of "
                f"'launch.py robot {self.robot_id}'.")

    def _refusal(self, payload: Dict[str, Any]) -> str:
        return (f"{self._who(payload)} is already serving {self.state_topic} (its heartbeat: "
                f"{payload.get('detail', '')!r}). Only one program may drive the AMR. {self._advice(payload)}")

    # ---- publishing ---------------------------------------------------------------------------
    def _publish_status(self, payload: Dict[str, Any], retain: bool = False) -> None:
        self._client.publish(self.status_topic, json.dumps(payload), qos=1, retain=retain)
        error = payload.get("error")
        logger.info("-> %s %s#%s %s%s", self.status_topic, payload.get("subtask_id"), payload.get("attempt"),
                    payload["state"], f": {error}" if error and error != payload["state"] else "")

    def _publish_heartbeat(self, refresh: bool = True) -> None:
        """refresh=False re-sends the last robot state with the current busy flag
        (used from the MQTT thread, where a slow robot call would stall it)."""
        if refresh or not self._driver_state:
            try:
                self._driver_state = self.navigator.get_state()
            except Exception as e:
                self._driver_state = {"ready": False, "detail": f"state error: {e}", "estop": False, "battery": None}
        st = self._driver_state
        with self._lock:
            current = self._current["key"][0] if self._current else None
        payload = {"online": True, "ready": bool(st.get("ready")), "estop": bool(st.get("estop")),
                   "battery": st.get("battery"), "detail": st.get("detail", ""),
                   "busy": current is not None, "current": current, "ts": time.time(),
                   "agent": AGENT_NAME, "instance": self.instance}
        for key in HEARTBEAT_EXTRAS:
            if key in st:
                payload[key] = st[key]
        if current is not None and "at_location" in payload:
            payload["at_location"], payload["at_locations"] = None, []    # driving: not "at" anywhere
        self._client.publish(self.state_topic, json.dumps(payload), qos=1, retain=True)

    # ---- steps --------------------------------------------------------------------------------
    def _dispatch(self, payload: Dict[str, Any]) -> None:
        if self._stop.is_set():
            return              # stopping: leave it to the scheduler's timeout / the other agent
        if payload.get("type", "task") == "cancel":
            self._handle_cancel(payload)
        else:
            self._handle_task(payload)

    def _interlock_problem(self) -> Optional[str]:
        for sib in self.interlock_with:
            st = self._siblings.get(sib)
            if st is None:
                return f"interlock: no state received from sibling '{sib}'"
            if not st.get("online", False):
                return f"interlock: sibling '{sib}' is offline"
            if st.get("busy"):
                return f"interlock: sibling '{sib}' is busy with {st.get('current')}"
            if st.get("travel_pose") is False:
                return f"interlock: sibling '{sib}' is not in its travel pose"
            if sib in self.require_travel_pose and st.get("travel_pose") is not True:
                return (f"interlock: sibling '{sib}' does not report a travel pose -- set travel_joints "
                        f"in its agent config")
        return None

    def _handle_task(self, task: Dict[str, Any]) -> None:
        sid, action = task.get("subtask_id"), task.get("action")
        attempt = task.get("attempt", 1)
        if not isinstance(sid, str) or not isinstance(action, str):
            logger.error("Task without subtask_id/action: %r", task)
            return
        key = (sid, attempt)
        base = {"subtask_id": sid, "attempt": attempt}
        with self._lock:
            if key in self._results:             # delivered again after we finished: re-send the result
                logger.info("Duplicate task %s#%s -- re-sending its result", sid, attempt)
                self._publish_status(self._results[key], retain=True)
                return
            if self._current and self._current["key"] == key:     # delivered again while running
                self._publish_status({**base, "state": "accepted"})
                return
            reject = None
            if self._current:
                reject = f"busy with {self._current['key'][0]}"
            elif action not in self.navigator.actions:
                reject = f"action {action!r} not supported by {self.subsystem} ({self.navigator.actions})"
            else:
                expires = task.get("expires_at")
                if _num(expires) and time.time() > expires + self.skew:
                    reject = (f"stale task: arrived {time.time() - expires:.0f}s after its accept deadline "
                              f"(robot was offline, or clocks are not NTP-synced)")
                else:
                    reject = self._interlock_problem()
            if reject:
                result = {**base, **failed(reject)}
                self._remember(key, result)
                self._publish_status(result, retain=True)
                return
            cancel = threading.Event()
            worker = threading.Thread(target=self._run, args=(task, key, cancel), name=f"task-{sid}", daemon=True)
            self._current = {"key": key, "thread": worker, "cancel": cancel, "subtask": task}
            self._publish_heartbeat(refresh=False)       # the arm sees busy=True before anything moves
            self._publish_status({**base, "state": "accepted"})
            worker.start()

    def _run(self, task: Dict[str, Any], key, cancel: threading.Event) -> None:
        sid, attempt = key
        logger.info("Executing %s#%s: %s to %s", sid, attempt, task["action"], task.get("zone"))
        try:
            outcome = self.navigator.execute(task, cancel)
            if not isinstance(outcome, dict) or outcome.get("state") not in TERMINAL:
                outcome = failed(f"invalid outcome {outcome!r}")
        except Exception as e:
            logger.exception("Navigation crashed on %s", sid)
            outcome = failed(f"AMR Agent error: {e}")
        result = {"subtask_id": sid, "attempt": attempt, **outcome}
        with self._lock:
            self._remember(key, result)
            self._current = None
        # fresh heartbeat (busy=False, where we are) BEFORE the result: the scheduler
        # sends the arm its next step only after the result, and the arm checks it
        self._publish_heartbeat(refresh=True)
        self._publish_status(result, retain=True)

    def _handle_cancel(self, msg: Dict[str, Any]) -> None:
        key = (msg.get("subtask_id"), msg.get("attempt", 1))
        with self._lock:
            if self._current and self._current["key"] == key:
                logger.warning("Cancelling %s#%s: %s", key[0], key[1], msg.get("reason"))
                self._current["cancel"].set()
                stop_robot = True
            else:
                stop_robot = False
                if key not in self._results:   # cancelled before it got here: refuse a late copy
                    self._remember(key, {"subtask_id": key[0], "attempt": key[1], "state": "cancelled",
                                         "error": f"cancelled before start: {msg.get('reason')}"})
        if stop_robot:
            self.navigator.cancel()

    def _remember(self, key, result) -> None:
        self._results[key] = result
        while len(self._results) > RESULT_CACHE:
            self._results.popitem(last=False)

    # ---- lifecycle ------------------------------------------------------------------------------
    def start_serving(self) -> None:
        """Read the robot once and announce ourselves (again on every reconnect)."""
        self._driver_state = {}
        self._serving.set()
        self._publish_heartbeat(refresh=True)

    def run_forever(self) -> int:
        """Check no other agent serves this AMR, connect, then serve until Ctrl+C.
        Returns the process exit code."""
        try:
            signal.signal(signal.SIGTERM, lambda *_: self._stop.set())
        except ValueError:
            pass                 # not in the main thread
        started = False
        try:
            self._wait_until_alone()
            if self._stop.is_set():
                return self.shutdown(started)
            self._driver_state = self.navigator.get_state()    # the first heartbeat goes out on connect
            self._serving.set()
            self._client.connect_async(self.broker.host, self.broker.port, keepalive=self.broker.keepalive)
            self._client.loop_start()
            started = True
            logger.info("Serving the scheduler's tasks for %s/%s. Ctrl+C stops.", self.robot_id, self.subsystem)
            while not self._stop.wait(self.heartbeat_s):
                if self._connected.is_set():
                    self._publish_heartbeat(refresh=True)
        except AnotherAgentRunning as e:
            logger.error("%s", e)
            self._conflict = str(e)
        except KeyboardInterrupt:
            pass
        return self.shutdown(started)

    def shutdown(self, connected: bool = True) -> int:
        self._stop.set()
        with self._lock:
            current = self._current
        if current:
            logger.warning("Stopping while executing %s -- stopping the robot", current["key"][0])
            current["cancel"].set()
            self.navigator.cancel()
            current["thread"].join(timeout=10)
        if not connected:
            return 2 if self._conflict else 0
        if self._serving.is_set() and not self._conflict:
            self._client.publish(self.state_topic, json.dumps(
                {"online": False, "ready": False, "detail": "agent stopped", "ts": time.time(),
                 "agent": AGENT_NAME, "instance": self.instance}), qos=1, retain=True)
            time.sleep(0.3)
        try:
            self._client.disconnect()       # a clean disconnect: the broker drops our last will
        finally:
            self._client.loop_stop()
        return 2 if self._conflict else 0


# =============================================================================
# Settings and command line (python amr_agent.py scheduler ...)
# =============================================================================

def _resolve(path: str, config_path: Optional[str]) -> str:
    """A relative path from a --config file: try the current folder, the
    config's folder, then its parent (the scheduler's layout: agents/x.json
    next to stations.json)."""
    if os.path.isabs(path) or not config_path:
        return path
    folder = os.path.dirname(os.path.abspath(config_path))
    for candidate in (path, os.path.join(folder, path), os.path.join(os.path.dirname(folder), path)):
        if os.path.exists(candidate):
            return candidate
    return path


def load_settings(config_path: Optional[str] = None, robot_id: Optional[str] = None, host: Optional[str] = None,
                  stations: Optional[str] = None) -> Dict[str, Any]:
    """DEFAULTS, then a scheduler-style agent config (--config), then the
    command line. Returns the merged settings with "host" and
    "stations_file" resolved."""
    cfg = copy.deepcopy(DEFAULTS)
    if config_path:
        with open(config_path, "r", encoding="utf-8") as f:
            user = json.load(f)
        drv = user.get("driver") or {}
        if drv.get("type") not in (None, "onix"):
            raise ValueError(f"{config_path} is for a {drv.get('type')!r} driver; AMR Agent drives the AMR "
                             f"through its own API (the scheduler's 'onix' driver type)")
        for k, v in user.items():
            if k != "driver" and not k.startswith("_"):
                cfg[k] = v
        cfg["driver"].update({k: v for k, v in drv.items() if not k.startswith("_")})
        if "stations_file" in drv:
            cfg["driver"]["stations_file"] = _resolve(drv["stations_file"], config_path)
    if robot_id:
        cfg["robot_id"] = robot_id
    if stations:
        cfg["driver"]["stations_file"] = stations
    elif not config_path and not os.path.exists(cfg["driver"]["stations_file"]):
        here = os.path.join(HERE, "stations.json")
        if os.path.exists(here):
            cfg["driver"]["stations_file"] = here
    cfg["host"] = host or cfg["driver"].get("host") or remembered_host()
    return cfg


def _probe_broker(broker: BrokerSettings, state_topic: str, timeout: float = 5.0) -> Dict[str, Any]:
    """Log in, read the AMR's current heartbeat, log out. Publishes nothing."""
    out: Dict[str, Any] = {"broker": f"{broker.host}:{broker.port}", "user": broker.username or "-"}
    seen: Dict[str, Any] = {}
    ok = threading.Event()
    client = broker.make_client(f"{AGENT_NAME}_check_{uuid.uuid4().hex[:6]}", clean_session=True)

    def on_connect(c, u, flags, rc, props=None):
        seen["rc"] = rc
        if not getattr(rc, "is_failure", False):
            c.subscribe(state_topic, 1)
        ok.set()

    def on_message(c, u, msg):
        try:
            seen["state"] = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            seen["state"] = msg.payload[:200]

    client.on_connect, client.on_message = on_connect, on_message
    try:
        client.connect_async(broker.host, broker.port, keepalive=broker.keepalive)
        client.loop_start()
        if not ok.wait(timeout):
            out["connected"] = False
            out["problem"] = "no answer from the broker (wrong host/port, firewall, or MQTT_TLS on a plain port?)"
            return out
        rc = seen["rc"]
        if getattr(rc, "is_failure", False):
            out["connected"] = False
            out["problem"] = f"the broker refused the login: {rc} (check MQTT_USERNAME / MQTT_PASSWORD)"
            return out
        out["connected"] = True
        time.sleep(1.5)
        st = seen.get("state")
        if isinstance(st, dict):
            age = time.time() - st["ts"] if _num(st.get("ts")) else None
            out["current_heartbeat"] = {"online": st.get("online"), "agent": st.get("agent") or "the scheduler's agent",
                                        "age_s": round(age, 1) if age is not None else None,
                                        "detail": st.get("detail")}
        else:
            out["current_heartbeat"] = None
        return out
    finally:
        client.disconnect()
        client.loop_stop()


def run_from_cli(args) -> int:
    """Entry point for `python amr_agent.py scheduler`."""
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    env_file = args.env_file or os.path.join(HERE, ".env")
    if args.env_file and not os.path.isfile(args.env_file):
        logger.error("--env-file %s not found", args.env_file)
        return 2
    loaded = load_env_file(env_file)
    try:
        broker = BrokerSettings(test_mode=args.test)
        cfg = load_settings(args.config, args.robot_id, args.host, args.stations)
    except (OSError, ValueError) as e:
        logger.error("%s", e)
        return 2
    if not cfg["host"]:
        logger.error("No robot address yet. Use --host <address>, or connect once with the desktop app.")
        return 2
    stations_file = cfg["driver"]["stations_file"]
    try:
        data = load_stations(stations_file)
    except FileNotFoundError:
        logger.error("Stations file %s not found. Point --stations at the Task Scheduler's stations.json "
                     "(it holds the x, y, theta of every location).", stations_file)
        return 2
    except ValueError as e:
        logger.error("Stations file %s is not valid: %s", stations_file, e)
        return 2

    drv = cfg["driver"]
    robot = RobotAPI(cfg["host"], drv.get("ports"), drv.get("api_key"), float(drv.get("request_timeout_s", 5)))
    navigator = Navigator(robot, stations_file, drv)
    robot_id, subsystem = cfg["robot_id"], cfg["subsystem"]
    state_topic = broker.topic(f"robots/{robot_id}/{subsystem}/state")
    with_pose = assigned_locations(data)
    without = sorted(set(data.get("locations", {})) - set(with_pose))

    logger.info("%s", broker.describe())
    logger.info("Broker %s:%s (user %s, settings from %s)", broker.host, broker.port, broker.username or "-",
                env_file if loaded else "environment variables")
    logger.info("Robot %s = %s/%s; stations %s: %d location(s) with x/y/theta%s", cfg["host"], robot_id, subsystem,
                os.path.abspath(stations_file), len(with_pose), f", no pose yet: {', '.join(without)}" if without else "")

    if args.check:
        report = {"robot": cfg["host"], "robot_id": robot_id, "subsystem": subsystem, "mode": broker.describe(),
                  "topics": {"task": broker.topic(f"robots/{robot_id}/{subsystem}/task"),
                             "status": broker.topic(f"robots/{robot_id}/{subsystem}/status"),
                             "state": state_topic},
                  "stations_file": stations_file, "locations_with_pose": with_pose, "locations_without_pose": without,
                  "heartbeat_would_say": navigator.get_state(),
                  "broker": _probe_broker(broker, state_topic)}
        print(json.dumps(report, indent=2, default=str))
        return 0

    link = SchedulerLink(navigator, broker, robot_id, subsystem, heartbeat_s=float(cfg.get("heartbeat_s", 5)),
                         interlock_with=tuple(cfg.get("interlock_with", [])),
                         require_travel_pose=tuple(cfg.get("require_travel_pose", [])),
                         clock_skew_tolerance_s=float(cfg.get("clock_skew_tolerance_s", 5)))
    logger.warning("This MOVES THE ROBOT when the scheduler sends a step. Keep the area clear.")
    return link.run_forever()
