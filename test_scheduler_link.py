"""
Tests for scheduler_link.py -- no robot, no broker needed.

    python -m unittest test_scheduler_link -v

A fake robot (drives to goals, with fault switches) stands in for AMRClient
and a fake MQTT client records what would be published. The scheduler's
messages are built exactly as the Task Scheduler sends them.
"""

import json
import math
import os
import shutil
import tempfile
import threading
import time
import unittest
from unittest import mock

import scheduler_link as sl

HALF_PI = math.pi / 2
POSES = {"manufacturing_1": (1.0, 0.0, 0.0), "quality_station": (1.0, 1.0, HALF_PI),
         "package_area": (0.0, 1.0, math.pi)}


def wait_for(pred, timeout=5.0, step=0.01):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return False


class FakeRobot:
    """Stands in for AMRClient: drives to free-nav goals at `speed`, then turns."""

    def __init__(self, speed=4.0, turn=8.0, hz=50):
        self.host = "fake-robot"
        self.x = self.y = self.theta = 0.0
        self.speed, self.turn, self.period = speed, turn, 1.0 / hz
        self.lin = self.ang = 0.0
        self.goal = None
        self.confidence, self.emergency, self.error_id, self.battery = 95.0, 0, 0, 80.0
        self.errors = []
        self.reject = self.ignore = self.unreachable = False
        self.stop_short_m = 0.0
        self.stations = {}
        self.calls = []
        self._lock = threading.Lock()
        self._last = time.time()

    def _step(self):
        now = time.time()
        with self._lock:
            dt, self._last = min(now - self._last, 0.1), now
            if self.goal is None or self.emergency:
                self.lin = self.ang = 0.0
                return
            gx, gy, gt = self.goal
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist - self.stop_short_m > 1e-3:
                step = min(dist - self.stop_short_m, self.speed * dt)
                self.x, self.y = self.x + step * dx / dist, self.y + step * dy / dist
                self.lin, self.ang = self.speed, 0.0
                return
            err = math.atan2(math.sin(gt - self.theta), math.cos(gt - self.theta))
            if not self.stop_short_m and abs(err) > 1e-3:
                self.theta += math.copysign(min(abs(err), self.turn * dt), err)
                self.lin, self.ang = 0.0, math.copysign(self.turn, err)
                return
            self.lin = self.ang = 0.0
            if not self.stop_short_m:
                self.goal = None

    def status(self):
        return {"x": self.x, "y": self.y, "angle": self.theta, "confidence": self.confidence,
                "emergency": self.emergency, "error_id": self.error_id, "linear_speed": self.lin,
                "angular_speed": self.ang, "percentage": self.battery}

    def read_once(self, channel, timeout=None):
        if self.unreachable:
            raise ConnectionRefusedError("connection refused")
        if channel == "errors_warnings":
            return {"errors": list(self.errors), "warnings": []}
        self._step()
        return self.status()

    def stream(self, channel, max_messages=None, timeout=None):
        while True:
            if self.unreachable:
                raise ConnectionRefusedError("connection refused")
            time.sleep(self.period)
            self._step()
            yield self.status()

    def navigate_to_point(self, x, y, theta):
        self.calls.append(("point", (x, y, theta)))
        if self.reject:
            return {"status": False, "message": "goal outside the map", "error_code": 3}
        if not self.ignore:
            with self._lock:
                self.goal = (x, y, theta)
        return {"status": True, "message": "ok"}

    def navigate_to_station(self, station, previous_node="-1"):
        self.calls.append(("station", station))
        with self._lock:
            self.goal = self.stations[station]
        return {"status": True}

    def cancel_navigation(self):
        self.calls.append(("cancel", None))
        with self._lock:
            self.goal = None
            self.lin = self.ang = 0.0
        return {"status": True}


class FakeClient:
    """Records publishes instead of talking to a broker."""

    def __init__(self):
        self.published, self.subscribed = [], []
        self.connects = 0
        self._lock = threading.Lock()

    def publish(self, topic, payload, qos=0, retain=False):
        with self._lock:
            self.published.append((topic, json.loads(payload), qos, retain))

    def subscribe(self, subs, qos=0):
        self.subscribed.append(subs)

    def connect_async(self, *a, **k):
        self.connects += 1

    def loop_start(self):
        pass

    def loop_stop(self):
        pass

    def disconnect(self):
        pass

    def on(self, topic):
        with self._lock:
            return [(p, retain) for t, p, q, retain in self.published if t == topic]


class Msg:
    def __init__(self, topic, payload, retain=False):
        self.topic = topic
        self.payload = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
        self.retain = retain


ARM_OK = {"online": True, "ready": True, "busy": False, "current": None, "travel_pose": True}
NAV_CFG = {"settle_s": 0.1, "start_timeout_s": 0.5, "stall_timeout_s": 0.5, "mission_timeout_s": 10,
           "max_comm_loss_s": 0.6, "error_grace_s": 0.2, "status_timeout_s": 0.3}


class LinkTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.stations = os.path.join(self.dir, "stations.json")
        locs = {k: {"x": x, "y": y, "theta": t} for k, (x, y, t) in POSES.items()}
        locs["rest_position"] = {"x": 1.0, "y": 0.0, "theta": 0.0}            # same spot as manufacturing_1
        locs["waste_area"] = {"station": None, "x": None, "y": None, "theta": None}
        locs["dock"] = {"station": "LM1", "x": -1.0, "y": 0.0, "theta": 0.0}
        with open(self.stations, "w") as f:
            json.dump({"defaults": {"xy_tolerance_m": 0.1, "yaw_tolerance_deg": 5},
                       "stations": {"LM1": {"x": -1.0, "y": 0.0, "theta": 0.0}}, "locations": locs}, f)
        self.robot = FakeRobot()
        self.robot.stations["LM1"] = (-1.0, 0.0, 0.0)
        self.link = None

    def tearDown(self):
        if self.link:
            self.link._stop.set()
            cur = self.link._current
            if cur:
                cur["cancel"].set()
                cur["thread"].join(5)
        shutil.rmtree(self.dir, ignore_errors=True)

    def make_link(self, serve=True, arm=ARM_OK, env=None, nav=None, **kw):
        self.nav = sl.Navigator(self.robot, self.stations, {**NAV_CFG, **(nav or {})})
        self.broker = sl.BrokerSettings(env=env or {})
        self.client = FakeClient()
        self.link = sl.SchedulerLink(self.nav, self.broker, client=self.client, heartbeat_s=0.2, **kw)
        self.link._connected.set()
        if arm is not None:
            self.arm_state(arm)
        if serve:
            self.link.start_serving()
        return self.link

    # ---- helpers -------------------------------------------------------------------
    def deliver(self, topic, payload, retain=False):
        self.link._on_message(None, None, Msg(topic, payload, retain))

    def arm_state(self, payload):
        self.deliver(self.broker.topic("robots/robot_01/arm/state"), payload)

    def send(self, sid="T-1", zone="manufacturing_1", attempt=1, action="navigate", **extra):
        now = time.time()
        msg = {"subtask_id": sid, "task_id": "t1", "action": action, "target": "ABCD", "zone": zone,
               "requires": [action], "group": "carrier", "releases_robot": False, "handover": None,
               "attempt": attempt, "type": "task", "dispatched_at": now, "expires_at": now + 30, **extra}
        self.deliver(self.link.task_topic, msg)

    def statuses(self, sid=None):
        return [(p, r) for p, r in self.client.on(self.link.status_topic) if sid is None or p["subtask_id"] == sid]

    def final(self, sid="T-1", timeout=5.0):
        assert wait_for(lambda: any(p["state"] in sl.TERMINAL for p, _ in self.statuses(sid)), timeout), \
            self.statuses(sid)
        return next(p for p, _ in self.statuses(sid) if p["state"] in sl.TERMINAL)

    def heartbeats(self):
        return [p for p, _ in self.client.on(self.link.state_topic)]


class NavigateTest(LinkTest):
    def test_drives_to_the_location_and_reports_verified_arrival(self):
        self.make_link()
        self.send()
        result = self.final()
        self.assertEqual(result["state"], "done", result)
        self.assertEqual(result["attempt"], 1)
        r = result["result"]
        self.assertTrue(r["verified"])
        self.assertLess(r["error_m"], 0.1)
        self.assertEqual(self.robot.calls, [("point", POSES["manufacturing_1"])])
        states = [p["state"] for p, _ in self.statuses()]
        self.assertEqual(states, ["accepted", "done"])
        retained = {p["state"]: r for p, r in self.statuses()}
        self.assertFalse(retained["accepted"])
        self.assertTrue(retained["done"])

    def test_heartbeat_has_what_the_scheduler_and_the_arm_read(self):
        self.make_link()
        hb = self.heartbeats()[-1]
        for key in ("online", "ready", "estop", "battery", "busy", "current", "detail", "ts",
                    "locations", "at_location", "at_locations", "pose"):
            self.assertIn(key, hb)
        self.assertTrue(hb["online"] and hb["ready"])
        self.assertFalse(hb["busy"] or hb["estop"])
        self.assertEqual(hb["battery"], 80.0)
        self.assertEqual(hb["locations"], sorted(["dock", "manufacturing_1", "package_area", "quality_station",
                                                  "rest_position"]))
        self.assertEqual(hb["agent"], "amr_agent")
        retained = [r for p, r in self.client.on(self.link.state_topic)]
        self.assertTrue(all(retained))

    def test_heartbeat_reports_busy_and_no_location_while_driving_then_the_arrival(self):
        self.robot.speed = 0.5
        self.make_link()
        self.send(zone="package_area")
        self.assertTrue(wait_for(lambda: any(p["busy"] for p in self.heartbeats())))
        busy = next(p for p in self.heartbeats() if p["busy"])
        self.assertEqual(busy["current"], "T-1")
        self.assertIsNone(busy["at_location"])
        self.assertEqual(busy["at_locations"], [])
        self.final()
        # the heartbeat that goes out with the result says where the base is now
        last = self.heartbeats()[-1]
        self.assertFalse(last["busy"])
        self.assertEqual(last["at_location"], "package_area")

    def test_heartbeat_goes_out_before_the_result(self):
        self.make_link()
        self.send()
        self.final()
        order = [(t, p.get("state"), p.get("busy")) for t, p, q, r in self.client.published]
        done_at = next(i for i, (t, s, _) in enumerate(order) if s == "done")
        self.assertEqual(order[done_at - 1][0], self.link.state_topic)
        self.assertFalse(order[done_at - 1][2])

    def test_location_sharing_a_spot_is_the_one_it_was_sent_to(self):
        self.make_link()
        self.send(zone="rest_position")
        self.assertEqual(self.final()["state"], "done")
        hb = self.heartbeats()[-1]
        self.assertEqual(hb["at_location"], "rest_position")
        self.assertEqual(sorted(hb["at_locations"]), ["manufacturing_1", "rest_position"])

    def test_station_mode_uses_path_navigation(self):
        self.make_link(nav={"nav_mode": "station"})
        self.send(zone="dock")
        self.assertEqual(self.final()["state"], "done")
        self.assertEqual(self.robot.calls, [("station", "LM1")])

    def test_station_mode_refuses_a_location_without_a_station(self):
        self.make_link(nav={"nav_mode": "station"})
        self.send(zone="manufacturing_1")
        self.assertIn("not linked to a station", self.final()["error"])


class RefusalTest(LinkTest):
    def check_failed(self, contains, **send):
        self.send(**send)
        result = self.final(send.get("sid", "T-1"))
        self.assertEqual(result["state"], "failed", result)
        self.assertIn(contains, result["error"])
        return result

    def test_unknown_location(self):
        self.make_link()
        self.check_failed("not in the stations file", zone="nowhere")

    def test_location_without_coordinates(self):
        self.make_link()
        self.check_failed("has no x/y/theta yet", zone="waste_area")

    def test_unsupported_action(self):
        self.make_link()
        self.check_failed("not supported", action="pick")

    def test_stale_task(self):
        self.make_link()
        self.check_failed("stale task", expires_at=time.time() - 60)

    def test_busy(self):
        self.robot.speed = 0.3
        self.make_link()
        self.send(sid="T-1", zone="package_area")
        wait_for(lambda: self.statuses("T-1"))
        self.check_failed("busy with T-1", sid="T-2")

    def test_emergency_stop(self):
        self.robot.emergency = 1
        self.make_link()
        self.check_failed("emergency stop")
        self.assertEqual(self.robot.calls, [])

    def test_not_localized(self):
        self.robot.confidence = 40
        self.make_link()
        self.check_failed("not localized (40% < 70%)")

    def test_robot_error(self):
        self.robot.error_id = 2
        self.robot.errors = ["laser fault"]
        self.make_link()
        self.check_failed("laser fault")

    def test_already_moving(self):
        self.robot.lin = 0.5
        self.robot.read_once = lambda channel, timeout=None: {**self.robot.status(), "linear_speed": 0.5}
        self.make_link()
        self.check_failed("already moving")

    def test_robot_unreachable(self):
        self.make_link()
        self.robot.unreachable = True
        self.check_failed("cannot read the AMR's status")


class InterlockTest(LinkTest):
    def refused_with(self, arm, contains):
        self.make_link(arm=arm)
        self.send()
        result = self.final()
        self.assertEqual(result["state"], "failed")
        self.assertIn(contains, result["error"])
        self.assertEqual(self.robot.calls, [])

    def test_no_arm_heartbeat(self):
        self.refused_with(None, "no state received from sibling 'arm'")

    def test_arm_offline(self):
        self.refused_with({**ARM_OK, "online": False}, "sibling 'arm' is offline")

    def test_arm_busy(self):
        self.refused_with({**ARM_OK, "busy": True, "current": "T-2"}, "sibling 'arm' is busy with T-2")

    def test_arm_not_tucked_in(self):
        self.refused_with({**ARM_OK, "travel_pose": False}, "not in its travel pose")

    def test_arm_without_a_travel_pose(self):
        arm = dict(ARM_OK)
        del arm["travel_pose"]
        self.refused_with(arm, "does not report a travel pose")

    def test_no_interlock_configured(self):
        self.make_link(arm=None, interlock_with=(), require_travel_pose=())
        self.send()
        self.assertEqual(self.final()["state"], "done")


class FailureWhileDrivingTest(LinkTest):
    def drive_and_fail(self, contains, setup=None, during=None, zone="package_area"):
        if setup:
            setup()
        self.make_link()
        self.send(zone=zone)
        if during:
            self.assertTrue(wait_for(lambda: self.robot.lin > 0))
            during()
        result = self.final(timeout=8)
        self.assertEqual(result["state"], "failed", result)
        self.assertIn(contains, result["error"])
        self.assertIn(("cancel", None), self.robot.calls)      # the robot was told to stop
        return result

    def test_goal_rejected(self):
        self.robot.reject = True
        self.make_link()
        self.send()
        result = self.final()
        self.assertIn("rejected the goal", result["error"])
        self.assertIn("goal outside the map", result["error"])

    def test_goal_ignored(self):
        self.drive_and_fail("did not start moving", setup=lambda: setattr(self.robot, "ignore", True))

    def test_blocked_short_of_the_goal(self):
        self.drive_and_fail("stopped 0.30 m", setup=lambda: setattr(self.robot, "stop_short_m", 0.3))

    def test_emergency_stop_while_driving(self):
        self.robot.speed = 0.3
        self.drive_and_fail("emergency stop pressed while driving", during=lambda: setattr(self.robot, "emergency", 1))

    def test_localization_lost_while_driving(self):
        self.robot.speed = 0.3
        self.drive_and_fail("localization lost (20%)", during=lambda: setattr(self.robot, "confidence", 20))

    def test_lasting_robot_error_while_driving(self):
        self.robot.speed = 0.3
        self.drive_and_fail("AMR error while driving", during=lambda: setattr(self.robot, "error_id", 2))

    def test_lost_contact_while_driving(self):
        self.robot.speed = 0.3
        self.drive_and_fail("lost contact with the AMR", during=lambda: setattr(self.robot, "unreachable", True))

    def test_mission_timeout(self):
        self.robot.speed = 0.05
        self.make_link(nav={"mission_timeout_s": 0.5})
        self.send(zone="package_area")
        result = self.final()
        self.assertIn("did not reach package_area", result["error"])


class CancelAndDuplicateTest(LinkTest):
    def test_cancel_while_driving_stops_the_robot(self):
        self.robot.speed = 0.3
        self.make_link()
        self.send(zone="package_area")
        self.assertTrue(wait_for(lambda: self.robot.lin > 0))
        self.deliver(self.link.task_topic, {"type": "cancel", "subtask_id": "T-1", "attempt": 1, "reason": "operator"})
        result = self.final()
        self.assertEqual(result["state"], "cancelled")
        self.assertIn(("cancel", None), self.robot.calls)
        self.assertTrue(wait_for(lambda: self.robot.lin == 0))

    def test_cancel_before_the_task_arrives_refuses_the_late_copy(self):
        self.make_link()
        self.deliver(self.link.task_topic, {"type": "cancel", "subtask_id": "T-1", "attempt": 1, "reason": "timeout"})
        self.send()
        result = self.final()
        self.assertEqual(result["state"], "cancelled")
        self.assertEqual(self.robot.calls, [])

    def test_duplicate_after_done_resends_the_result_without_moving(self):
        self.make_link()
        self.send()
        first = self.final()
        self.send()        # QoS 1 may deliver twice
        self.assertTrue(wait_for(lambda: len([p for p, _ in self.statuses() if p["state"] == "done"]) == 2))
        self.assertEqual(len(self.robot.calls), 1)
        last, retained = self.statuses()[-1]
        self.assertEqual(last, first)
        self.assertTrue(retained)

    def test_duplicate_while_running_is_acknowledged_again(self):
        self.robot.speed = 0.3
        self.make_link()
        self.send(zone="package_area")
        self.send(zone="package_area")
        self.assertEqual([p["state"] for p, _ in self.statuses()], ["accepted", "accepted"])
        self.assertEqual(self.final()["state"], "done")
        self.assertEqual(self.robot.calls, [("point", POSES["package_area"])])     # one goal, not two

    def test_a_retry_is_a_new_attempt(self):
        self.robot.reject = True
        self.make_link()
        self.send(attempt=1)
        self.assertEqual(self.final()["state"], "failed")
        self.robot.reject = False
        self.send(attempt=2)
        self.assertTrue(wait_for(lambda: any(p["attempt"] == 2 and p["state"] == "done" for p, _ in self.statuses())))

    def test_garbage_does_not_break_the_link(self):
        self.make_link()
        self.deliver(self.link.task_topic, b"not json")
        self.deliver(self.link.task_topic, [1, 2, 3])
        self.deliver(self.link.task_topic, {"type": "task"})
        self.deliver(self.broker.topic("robots/robot_01/arm/state"), b"\xff")
        self.send()
        self.assertEqual(self.final()["state"], "done")


class RC:
    is_failure = False


class ProbeClient(FakeClient):
    """The startup check's broker connection: replays retained messages on subscribe."""

    def __init__(self, retained=None):
        super().__init__()
        self.retained = retained or {}

    def loop_start(self):
        self.on_connect(self, None, None, RC())

    def subscribe(self, topic, qos=0):
        super().subscribe(topic, qos)
        if topic in self.retained:
            self.on_message(self, None, Msg(topic, self.retained[topic], retain=True))

    def live(self, topic, payload, after=0.0):
        threading.Timer(after, lambda: self.on_message(self, None, Msg(topic, payload))).start()


class OneAgentOnlyTest(LinkTest):
    def setUp(self):
        super().setUp()
        patches = [mock.patch.object(sl, "RETAINED_WAIT_S", 0.15), mock.patch.object(sl, "FOREIGN_WATCH_S", 0.6)]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def other(self, **extra):
        return {"online": True, "ready": True, "busy": False, "ts": time.time(), "detail": "loc 95%", **extra}

    def check(self, retained=None, live=None, after=0.3):
        self.make_link(serve=False)
        probe = ProbeClient({self.link.state_topic: retained} if retained else None)
        if live:
            probe.live(self.link.state_topic, live, after)
        self.link._wait_until_alone(probe)
        return probe

    def test_starts_when_nobody_else_serves_the_amr(self):
        probe = self.check()
        self.assertEqual(probe.subscribed, ["robots/robot_01/amr/state"])
        self.assertEqual(probe.published, [])            # the check only listens

    def test_refuses_when_another_agent_is_heartbeating(self):
        with self.assertRaises(sl.AnotherAgentRunning) as e:
            self.check(retained=self.other(), live=self.other())
        self.assertIn("launch.py agents agents/robot_01_arm.json", str(e.exception))
        self.assertIn("the scheduler's own", str(e.exception))

    def test_refuses_at_once_on_a_live_heartbeat(self):
        started = time.time()
        with self.assertRaises(sl.AnotherAgentRunning):
            self.check(live=self.other(agent="amr_agent", instance="x"), after=0.05)
        self.assertLess(time.time() - started, 0.5)

    def test_takes_over_from_a_leftover_heartbeat(self):
        self.check(retained=self.other())                 # nobody publishes after it

    def test_ignores_offline_and_old_heartbeats(self):
        self.check(retained={"online": False, "detail": "agent connection lost (last will)"},
                   live=self.other(ts=time.time() - 60), after=0.05)

    def test_refused_start_never_connects_the_main_client(self):
        self.make_link(serve=False)

        def refuse(probe=None):
            raise sl.AnotherAgentRunning("taken")
        self.link._wait_until_alone = refuse
        self.assertEqual(self.link.run_forever(), 2)
        self.assertEqual(self.client.connects, 0)
        self.assertEqual(self.client.published, [])

    def test_retained_copy_on_reconnect_is_not_a_conflict(self):
        self.make_link()
        self.deliver(self.link.state_topic, self.other(), retain=True)
        self.assertFalse(self.link._stop.is_set())

    def test_stands_down_when_another_agent_starts_later(self):
        self.robot.speed = 0.3
        self.make_link()
        self.send(zone="package_area")
        self.assertTrue(wait_for(lambda: self.robot.lin > 0))
        self.deliver(self.link.state_topic, self.other())
        self.assertTrue(self.link._stop.is_set())
        self.assertIn("Only one program", self.link._conflict)
        self.assertIn(("cancel", None), self.robot.calls)
        self.assertEqual(self.final()["state"], "cancelled")
        self.send(sid="T-2")                         # no longer answers tasks
        time.sleep(0.2)
        self.assertEqual(self.statuses("T-2"), [])

    def test_own_messages_are_not_another_agent(self):
        self.make_link()
        for payload, _ in self.client.on(self.link.state_topic):
            self.deliver(self.link.state_topic, payload)
        self.assertFalse(self.link._stop.is_set())

    def test_of_two_amr_agents_the_lower_instance_stands_down(self):
        self.make_link()
        self.deliver(self.link.state_topic, self.other(agent="amr_agent", instance="0" * 12))
        self.assertFalse(self.link._stop.is_set())
        self.deliver(self.link.state_topic, self.other(agent="amr_agent", instance="f" * 12))
        self.assertTrue(self.link._stop.is_set())

    def test_shutdown_after_standing_down_does_not_mark_the_amr_offline(self):
        self.make_link()
        self.deliver(self.link.state_topic, self.other())
        before = len(self.heartbeats())
        self.assertEqual(self.link.shutdown(), 2)
        self.assertEqual(len(self.heartbeats()), before)

    def test_normal_shutdown_marks_the_amr_offline(self):
        self.make_link()
        self.assertEqual(self.link.shutdown(), 0)
        self.assertFalse(self.heartbeats()[-1]["online"])


class SettingsTest(unittest.TestCase):
    def test_real_mode_topics_at_the_root(self):
        b = sl.BrokerSettings(env={"MQTT_BROKER_HOST": "10.0.0.2", "MQTT_BROKER_PORT": "1884"})
        self.assertEqual((b.host, b.port, b.prefix, b.client_suffix), ("10.0.0.2", 1884, "", ""))
        self.assertEqual(b.topic("robots/robot_01/amr/task"), "robots/robot_01/amr/task")

    def test_test_mode_like_the_scheduler(self):
        for b in (sl.BrokerSettings(env={}, test_mode=True), sl.BrokerSettings(env={"RUN_MODE": "test"})):
            self.assertEqual(b.topic("robots/robot_01/amr/task"), "sim/robots/robot_01/amr/task")
            self.assertEqual(b.client_suffix, "-test")
        self.assertEqual(sl.BrokerSettings(env={"MQTT_TOPIC_PREFIX": "pytest/x"}).prefix, "pytest/x/")
        with self.assertRaises(ValueError):
            sl.BrokerSettings(env={"RUN_MODE": "demo"})
        with self.assertRaises(ValueError):
            sl.BrokerSettings(env={"MQTT_TOPIC_PREFIX": ""}, test_mode=True)

    def test_credentials_and_tls(self):
        b = sl.BrokerSettings(env={"MQTT_USERNAME": "u", "MQTT_PASSWORD": "p", "MQTT_TLS": "1"})
        self.assertEqual((b.username, b.password, b.tls), ("u", "p", True))
        self.assertIsNone(sl.BrokerSettings(env={"MQTT_USERNAME": ""}).username)

    def test_scheduler_agent_config_is_read_as_is(self):
        root = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, root, True)
        os.makedirs(os.path.join(root, "agents"))
        with open(os.path.join(root, "stations.json"), "w") as f:
            json.dump({"locations": {}}, f)
        cfg_path = os.path.join(root, "agents", "robot_01_amr.json")
        with open(cfg_path, "w") as f:      # as shipped in the scheduler repo
            json.dump({"_comment": "x", "robot_id": "robot_01", "subsystem": "amr", "heartbeat_s": 5,
                       "interlock_with": ["arm"], "require_travel_pose": ["arm"],
                       "driver": {"type": "onix", "host": "172.22.74.119", "stations_file": "stations.json",
                                  "settle_s": 2.0, "ports": {"rest": 3000, "control": 4000, "status": 7000}}}, f)
        cwd = os.getcwd()
        os.chdir(tempfile.gettempdir())
        self.addCleanup(os.chdir, cwd)
        cfg = sl.load_settings(cfg_path)
        self.assertEqual(cfg["host"], "172.22.74.119")
        self.assertEqual(os.path.realpath(cfg["driver"]["stations_file"]),
                         os.path.realpath(os.path.join(root, "stations.json")))
        self.assertEqual(cfg["driver"]["settle_s"], 2.0)
        self.assertEqual(cfg["driver"]["min_localization"], 70)        # default kept
        self.assertNotIn("_comment", cfg)
        cfg = sl.load_settings(cfg_path, robot_id="robot_02", host="10.1.1.1", stations="mine.json")
        self.assertEqual((cfg["robot_id"], cfg["host"], cfg["driver"]["stations_file"]),
                         ("robot_02", "10.1.1.1", "mine.json"))

    def test_other_vendor_config_is_refused(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump({"driver": {"type": "mir", "host": "1.2.3.4"}}, f)
        self.addCleanup(os.remove, f.name)
        with self.assertRaises(ValueError):
            sl.load_settings(f.name)

    def test_tolerances_location_then_file_then_settings(self):
        data = {"defaults": {"xy_tolerance_m": 0.2}, "locations": {
            "a": {"x": 0, "y": 0, "theta": 0, "yaw_tolerance_deg": 10}, "b": {"x": 0, "y": 0, "theta": 0}}}
        self.assertEqual(sl.location_pose(data, "a", 0.1, 5)[3:], (0.2, math.radians(10)))
        self.assertEqual(sl.location_pose({"locations": data["locations"]}, "b", 0.1, 5)[3:], (0.1, math.radians(5)))

    def test_robot_api_uses_the_configured_ports_and_key(self):
        api = sl.RobotAPI("127.0.0.1", {"rest": 13000, "control": 14000, "status": 17000}, api_key="k")
        with mock.patch("amr_client.websocket.create_connection") as conn:
            api._ws_open(4000, "/nav")
        self.assertEqual(conn.call_args[0][0], "ws://127.0.0.1:14000/nav")
        self.assertEqual(json.loads(api._payload({"api_id": "4004"}))["key"], "k")
        self.assertEqual(json.loads(sl.RobotAPI("h")._payload({}))["key"], sl.API_KEY)


if __name__ == "__main__":
    unittest.main()
