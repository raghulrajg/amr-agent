from __future__ import annotations

import argparse
import json
import sys

from amr_client import (AMRClient, ChannelSilentError, find_robots, local_subnets,
                        normalize_host, remember_host, remembered_host)

APP_TITLE = "AMR Agent"


# =============================================================================
# Command-line commands
# =============================================================================

def cmd_overview(c: AMRClient) -> None:
    print(f"{APP_TITLE} - robot {c.host}\n")
    print(AMRClient.format_summary(c.get_summary()))
    print("Errors/warnings:", c.get_errors())

    if not c.get_pose()["localized"]:
        print("Robot is NOT localized (position is 0). Relocate it first.")

    live = c.get_live_map()["data"]
    info = c.summarize_map(live)
    print(f"\nLive map: {info['name']} - {info['points']} points, "
          f"{info['stations']} stations, {info['paths']} paths")
    for st in live.get("advancedPointList") or []:
        print(f"  {st['instanceName']}: ({st['pos']['x']:.3f}, {st['pos']['y']:.3f}) dir={st['dir']}")

    print()
    cmd_maps(c)

    try:
        print(f"\nLidar: {len(c.get_lidar_points())} points")
    except ChannelSilentError:
        print("\nLidar: no data (the robot is not sending scans right now)")


def cmd_status(c: AMRClient) -> None:
    print(json.dumps(c.get_summary(), indent=2))


def cmd_watch(c: AMRClient) -> None:
    print(f"{APP_TITLE} - live status of {c.host} (Ctrl+C to stop)")
    c.watch_status()


def cmd_maps(c: AMRClient) -> None:
    maps = c.map_summaries()
    print(f"Saved maps ({len(maps)}):")
    for m in maps:
        live = "  (LIVE)" if m["is_live"] else ""
        print(f"  id {m['id']}: {m['name']}{live} - {m['points']} points, "
              f"{m['stations']} stations, {m['paths']} paths")


def cmd_export(c: AMRClient, file: str, map_id=None) -> None:
    print("Saved", c.export_map(file, map_id))


def cmd_lidar(c: AMRClient) -> None:
    try:
        pts = c.get_lidar_points()
    except ChannelSilentError as e:
        print(e)
        return
    print(f"{len(pts)} points (map coordinates):")
    for x, y in pts:
        print(f"{x:.3f}\t{y:.3f}")


def cmd_find() -> None:
    prefixes = local_subnets()
    last = remembered_host()
    if last and last.count(".") == 3:
        prefixes.append(last.rsplit(".", 1)[0])      # also look near the last known address
    prefixes = list(dict.fromkeys(prefixes))
    if not prefixes:
        print("This computer doesn't seem to be on a network.")
        return
    print("Searching " + ", ".join(f"{p}.1-254" for p in prefixes) + " for robots...")

    def progress(done, total, host):
        if host:
            print(f"  found {host}")
        elif done == total or done % 64 == 0:
            print(f"  {done}/{total} checked", end="\r", flush=True)

    found = find_robots(prefixes, on_progress=progress)
    print(" " * 40, end="\r")
    if not found:
        print("No robot found. Is the robot on, and is this computer on the same network?")
        return
    print(f"Robots found: {', '.join(found)}")
    if len(found) == 1:
        remember_host(found[0])
        print(f"Saved {found[0]} as the robot address for the next commands.")
    else:
        print("Use --host <address> to pick one.")


def cmd_gui(host) -> None:
    try:
        from amr_gui import run
    except ImportError as e:
        if "tkinter" in str(e).lower():
            raise SystemExit("Tkinter is missing. On Windows, re-run the Python installer and "
                             "tick 'tcl/tk and IDLE'.") from None
        raise
    run(host)


# =============================================================================
# Argument parsing
# =============================================================================

def main() -> None:
    # --host may be given before or after the command
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--host", default=argparse.SUPPRESS,
                        help="robot IP address (default: the last one that worked)")

    ap = argparse.ArgumentParser(prog="amr_agent.py",
                                 description=f"{APP_TITLE}: desktop app and command line for the AMR. "
                                             "With no command, the desktop app opens.")
    ap.add_argument("--host", default=None, help="robot IP address (default: the last one that worked)")
    sub = ap.add_subparsers(dest="command", metavar="command")
    sub.add_parser("gui", parents=[common], help="open the desktop app (default)")
    sub.add_parser("find", parents=[common], help="search the network for robots")
    sub.add_parser("overview", parents=[common], help="text overview of the robot")
    sub.add_parser("status", parents=[common], help="battery and status, decoded")
    sub.add_parser("watch", parents=[common], help="live status line, Ctrl+C to stop")
    sub.add_parser("maps", parents=[common], help="list maps saved on the robot")
    ex = sub.add_parser("export", parents=[common], help="save the live map (or --id N) to JSON")
    ex.add_argument("file")
    ex.add_argument("--id", help="saved map id instead of the live map")
    sub.add_parser("lidar", parents=[common], help="one lidar scan as x, y points")
    args = ap.parse_args()

    if args.command in (None, "gui"):
        cmd_gui(args.host)               # no --host: the app asks for the address
        return
    if args.command == "find":
        cmd_find()
        return

    host = normalize_host(args.host) if args.host else remembered_host()
    if not host:
        sys.exit("No robot address yet. Use --host <address>, run 'python amr_agent.py find', "
                 "or connect once with the desktop app.")
    if not args.host:
        print(f"(robot {host}, last used; change it with --host)", file=sys.stderr)

    c = AMRClient(host)
    try:
        run_command(c, args)
    except BrokenPipeError:
        pass                             # output was cut short (e.g. piped to "more"); not a robot problem
    except OSError as e:
        sys.exit(f"Cannot reach the robot at {host}: {e}\n"
                 f"If its address changed, use --host <address> or 'python amr_agent.py find'.")
    if args.host:
        remember_host(host)              # it worked, so use it next time


def run_command(c: AMRClient, args) -> None:
    if args.command == "overview":
        cmd_overview(c)
    elif args.command == "status":
        cmd_status(c)
    elif args.command == "watch":
        cmd_watch(c)
    elif args.command == "maps":
        cmd_maps(c)
    elif args.command == "export":
        cmd_export(c, args.file, args.id)
    elif args.command == "lidar":
        cmd_lidar(c)


if __name__ == "__main__":
    main()