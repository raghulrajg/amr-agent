# AMR Agent

A desktop app and command line for the AMR (autonomous mobile robot).

| File | What it is |
|---|---|
| `amr_agent.py` | Start here. Opens the desktop app, or runs a command-line command. |
| `amr_gui.py` | The desktop app (window). |
| `amr_client.py` | The library that talks to the robot. Use it in your own scripts too. |
| `requirements.txt` | The Python packages needed. |
| `scheduler_link.py` | Connects AMR Agent to the Task Scheduler (see [Task Scheduler](#task-scheduler)). |
| `test_scheduler_link.py` | Tests for the scheduler link. No robot or broker needed. |
| `.env.example` | Broker settings for the scheduler link. Copy it to `.env`. |

Keep all files in the same folder.

## Install

```
pip install -r requirements.txt
```

The desktop app uses Tkinter, which comes with Python on Windows. If you get
"Tkinter is missing", re-run the Python installer and tick **tcl/tk and IDLE**.

## Desktop app

```
python amr_agent.py
```

The robot's IP address can change, so the app opens with a **Connect** window:

- **Address box:** the last address that worked is already filled in, and the arrow lists earlier ones. Type a new one, or paste the robot's web address, and press **Connect**.
- **Search network:** finds the robot on this computer's network and lists what it finds. Double-click one to connect.
- **No answer:** if nothing answers at the address, the app tells you. Press **Connect anyway** to open it regardless.
- **Change address later:** click **Change** next to the robot address in the top bar. When the robot is unreachable, the map also shows a **Change robot address** button.

To skip the Connect window, use `python amr_agent.py gui --host 192.168.1.50`.

The app has three pages:

- **Map View:** the live map with lidar, robot, footprint, stations, paths, areas, and local and global paths, styled like the robot's web app.
  - The top bar shows errors, battery, Wi-Fi, e-stop and localization. Hover over an icon for details.
  - The "Live" badge turns grey ("Offline") when the robot stops sending data.
  - Use the mouse wheel to zoom and drag to pan. The left buttons are Pan, Fit to screen, Zoom in and Zoom out.
  - The « button on the right opens the layer switches, **Follow robot**, map info and **Reload map**.
- **Robot Status:** all status values decoded, errors and warnings with **Clear errors**, and every raw field.
- **Saved Maps:** the maps stored on the robot. You can export the selected map or the live map to a JSON file.

The app only views the robot. It never sends movement commands.

## Command line

Commands use `--host <address>` if you give it. Otherwise they use the last
address that worked (from the app, `--host` or `find`).

```
python amr_agent.py find                 search the network for robots (saves the one it finds)
python amr_agent.py overview             text overview of the robot
python amr_agent.py status               battery and status
python amr_agent.py watch                live status line (Ctrl+C to stop)
python amr_agent.py maps                 maps saved on the robot
python amr_agent.py export live.json     save the live map (add --id N for a saved map)
python amr_agent.py lidar                one lidar scan as x, y points
python amr_agent.py scheduler            follow the Task Scheduler's orders (MOVES THE ROBOT, see below)
python amr_agent.py --help               all commands
```

## Task Scheduler

AMR Agent can take its orders from the [Task Scheduler](https://github.com/raghulrajg/Task-Scheduler-Agent)
(its `master` branch). The scheduler sends each "navigate to a location" step over MQTT. AMR Agent drives
the robot there and reports back, following the scheduler's rules for robot agents:

- **Where to go:** the x, y, theta of each location come from the scheduler's `stations.json`, read again for every move.
- **Done means verified:** a move is reported done only when the robot is there (within tolerance), stopped and localized. Only then does the scheduler send the arm its next step.
- **Refusals:** a step is refused if the e-stop is pressed, the robot is not localized or is in error, or the arm on the base is busy or not in its travel pose.
- **Stopping:** the robot is stopped and the step fails on an e-stop, lost localization, a stall, lost contact or a timeout. A cancel from the scheduler also stops it.
- **Heartbeat:** every 5 s it tells the scheduler and the arm if the AMR is ready, its battery, where it is and which locations have coordinates.

**Set up on the robot PC** (it needs to reach the robot and the MQTT broker):

1. `pip install -r requirements.txt`
2. Copy `.env.example` to `.env` and fill in the broker: the same address, user and password as the scheduler. Or use the scheduler's own `.env` with `--env-file`.
3. Start it with the scheduler's `stations.json`:

```
python amr_agent.py scheduler --stations ../Task-Scheduler-Agent/stations.json
```

Other ways to run it:

```
python amr_agent.py scheduler --stations FILE --check     what the scheduler would see, and a broker login test (nothing moves)
python amr_agent.py scheduler --stations FILE --test      the scheduler's TEST mode (topics under sim/)
python amr_agent.py scheduler --config ../Task-Scheduler-Agent/agents/robot_01_amr.json
                                                          the same settings as the scheduler's own AMR agent
```

The robot address is `--host`, else the `host` in `--config`, else the last one that worked. The robot id
is `robot_01` unless you give `--robot-id`.

**Only one program may drive the AMR.** The scheduler can also drive the AMR itself: `launch.py robot robot_01`
starts its own AMR agent and the arm. When AMR Agent drives the AMR, start only the arm from the scheduler:

```
python3 launch.py agents agents/robot_01_arm.json
```

AMR Agent will not start while another program is serving the AMR, and it stops if one starts later.

**MQTT topics** (in TEST mode they start with `sim/`):

| Topic | From → to | Message |
|---|---|---|
| `robots/robot_01/amr/task` | scheduler → AMR Agent | `{"type": "task", "subtask_id", "attempt", "action": "navigate", "zone", "expires_at", ...}`, or `{"type": "cancel", "subtask_id", "attempt"}` |
| `robots/robot_01/amr/status` | AMR Agent → scheduler | `accepted`, then `done` (with the final pose), `failed` (with the reason) or `cancelled` |
| `robots/robot_01/amr/state` | AMR Agent → everyone | the heartbeat, kept by the broker; it says offline when AMR Agent stops or loses the connection |
| `robots/robot_01/arm/state` | arm → AMR Agent | the arm's heartbeat, for the interlock |

To test the scheduler link without a robot or a broker: `python -m unittest test_scheduler_link -v`

## In your own script

```python
from amr_client import AMRClient

from amr_client import find_robots, remembered_host

host = remembered_host() or find_robots()[0]   # last address used, or search
c = AMRClient(host)
print(c.get_summary())              # battery, e-stop, errors, localization
live = c.get_live_map()["data"]     # active map
for m in c.map_summaries():         # saved maps
    print(m["id"], m["name"], m["is_live"])
points = c.get_lidar_points()       # lidar as [(x, y), ...]
```

The top of `amr_client.py` lists every method, grouped as Status, Maps, Lidar,
Localization, Navigation, Mapping, Tele-op and Tasks.

## Good to know

- **Some library methods move the robot:** `drive`, `navigate_to_point` and `navigate_to_station`. So does `python amr_agent.py scheduler`. Keep the area clear.
- **Lidar and pose need localization:** they only stream while the robot is localized and the lidar works. Until then the map shows no lidar points and the robot sits at (0, 0).
- **Not localized:** relocate the robot first, for example with `c.auto_relocate()`.
- **Where addresses are saved:** recent robot addresses are kept in `.amr_agent.json` in your user folder.
- **What Search network covers:** it checks this computer's own network (x.x.x.1 to 254) plus the network of the last address used. If the robot is on a different range, type its address.