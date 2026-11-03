# AMR Agent

A desktop app and command line for the AMR (autonomous mobile robot).

| File | What it is |
|---|---|
| `amr_agent.py` | Start here. Opens the desktop app, or runs a command-line command. |
| `amr_gui.py` | The desktop app (window). |
| `amr_client.py` | The library that talks to the robot. Use it in your own scripts too. |
| `requirements.txt` | The Python packages needed. |

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
python amr_agent.py --help               all commands
```

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

- **Some library methods move the robot:** `drive`, `navigate_to_point` and `navigate_to_station`. Keep the area clear.
- **Lidar and pose need localization:** they only stream while the robot is localized and the lidar works. Until then the map shows no lidar points and the robot sits at (0, 0).
- **Not localized:** relocate the robot first, for example with `c.auto_relocate()`.
- **Where addresses are saved:** recent robot addresses are kept in `.amr_agent.json` in your user folder.
- **What Search network covers:** it checks this computer's own network (x.x.x.1 to 254) plus the network of the last address used. If the robot is on a different range, type its address.