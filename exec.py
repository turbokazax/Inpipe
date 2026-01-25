#!/usr/bin/env python3
"""
exec.py — run an experiment routine + optional viewer in one command.

Supports:
  - routine as dotted module:   Routines.test934
  - routine as file path:       Routines/test934.py   (tab-completable)

Pass-through args:
  - Everything after `--` is forwarded to the routine module.

Auto-sync:
  - If routine passthru contains `--r0 <int>`, viewer gets `--r0 <same>`
    unless you explicitly set --viewer-r0.

Examples:
  py exec.py Routines/test934.py
  py exec.py Routines/test934.py -- --r0 455626 --k 16875 --period 20
  py exec.py Routines/test934.py --viewer-axis-mode locked -- --r0 455626
  py exec.py Routines/test934.py --viewer-r0 123 -- --r0 455626   # viewer override wins
"""

import argparse
import os
import random
import signal
import socket
import subprocess
import sys
import time
import textwrap
from pathlib import Path


# -----------------------------
# UDP port helpers
# -----------------------------
def pick_free_udp_port(host: str = "127.0.0.1", lo: int = 49152, hi: int = 65535) -> int:
    for _ in range(2000):
        p = random.randint(lo, hi)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((host, p))
            return p
        except OSError:
            pass
        finally:
            s.close()
    raise RuntimeError("No free UDP port found")


def is_udp_port_free(host: str, port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


# -----------------------------
# Routine path → module name
# -----------------------------
def module_from_path(path_str: str, project_root: Path) -> str:
    p = Path(path_str)
    if not p.is_absolute():
        p = (project_root / p).resolve()
    else:
        p = p.resolve()

    if not p.exists():
        raise FileNotFoundError(f"Routine path does not exist: {p}")
    if p.suffix != ".py":
        raise ValueError(f"Expected a .py file, got: {p}")

    rel = p.relative_to(project_root)
    parts = list(rel.with_suffix("").parts)  # drop .py
    return ".".join(parts)


def resolve_routine_arg(routine_arg: str, project_root: Path) -> str:
    if routine_arg.endswith(".py") or "/" in routine_arg or "\\" in routine_arg:
        return module_from_path(routine_arg, project_root)
    return routine_arg


# -----------------------------
# Parse-through helpers
# -----------------------------
def extract_int_flag(argv, flag_name: str):
    """
    Extract `--flag <int>` from an argv list without modifying it.
    Returns int or None.
    """
    try:
        i = argv.index(flag_name)
    except ValueError:
        return None
    if i + 1 >= len(argv):
        return None
    try:
        return int(argv[i + 1])
    except ValueError:
        return None


# -----------------------------
# Process shutdown (SIGINT grace)
# -----------------------------
def _send_sigint(proc: subprocess.Popen):
    if proc and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            try:
                proc.send_signal(signal.SIGINT)
            except Exception:
                pass


def _terminate(proc: subprocess.Popen):
    if proc and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass


def _kill(proc: subprocess.Popen):
    if proc and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def terminate_processes(viewer_proc, routine_proc, routine_grace_s=6.0, viewer_grace_s=1.0):
    _send_sigint(viewer_proc)
    _send_sigint(routine_proc)

    if routine_proc and routine_proc.poll() is None:
        t0 = time.time()
        while time.time() - t0 < routine_grace_s:
            if routine_proc.poll() is not None:
                break
            time.sleep(0.05)

    if viewer_proc and viewer_proc.poll() is None:
        t0 = time.time()
        while time.time() - t0 < viewer_grace_s:
            if viewer_proc.poll() is not None:
                break
            time.sleep(0.05)

    _terminate(viewer_proc)
    _terminate(routine_proc)

    time.sleep(0.4)

    _kill(viewer_proc)
    _kill(routine_proc)


# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser(prog="exec.py", description="Run experiment routine + optional viewer.")
    ap.add_argument("routine", help="Module (Routines.test934) OR path (Routines/test934.py)")

    # Viewer settings
    ap.add_argument("--viewer", default="GUI.viewer_xy", help="Viewer module (default: GUI.viewer_xy)")
    ap.add_argument("--no-viewer", action="store_true", help="Don’t launch viewer.")
    ap.add_argument("--viewer-axis-mode", default="dynamic", choices=["dynamic", "locked"],
                    help="Pass-through to viewer (--axis_mode).")
    ap.add_argument("--viewer-r0", type=int, default=None,
                    help="Override viewer --r0 explicitly. If omitted, may inherit from routine --r0.")

    # Networking / runtime
    ap.add_argument("--host", default="127.0.0.1", help="Host for UDP (default: 127.0.0.1)")
    ap.add_argument("--viz-port", type=int, default=None,
                    help="Viz UDP port. Default: try 9999 else random.")
    ap.add_argument("--python", default=sys.executable, help="Python executable to use.")

    # Flag names (keep these unless you rename args)
    ap.add_argument("--routine-viz-flag", default="--viz-port",
                    help="Flag name your routine expects for viz port (default: --viz-port).")
    ap.add_argument("--viewer-port-flag", default="--port",
                    help="Flag name your viewer expects for port (default: --port).")

    # Extra help flag: combined help for exec + viewer + routine
    # (We parse it from sys.argv because argparse would exit early on --help)
    ap.add_argument("--help-all", action="store_true",
                    help="Show combined help for exec.py + viewer + routine, then exit.")

    args, passthru = ap.parse_known_args()

    project_root = Path(__file__).resolve().parent  # /Inpipe
    routine_mod = resolve_routine_arg(args.routine, project_root)

    if args.help_all:
        print("\n========== exec.py help ==========\n")
        ap.print_help()

        # viewer help
        print("\n========== viewer help ==========\n")
        try:
            subprocess.run([args.python, "-m", args.viewer, "--help"], cwd=str(project_root))
        except Exception as e:
            print(f"(failed to run viewer help: {e})")

        # routine help
        print("\n========== routine help ==========\n")
        try:
            subprocess.run([args.python, "-m", routine_mod, "--help"], cwd=str(project_root))
        except Exception as e:
            print(f"(failed to run routine help: {e})")

        return


    # If routine got an --r0, mirror it into viewer (unless viewer-r0 override is set)
    routine_r0 = extract_int_flag(passthru, "--r0")
    viewer_r0_effective = args.viewer_r0 if args.viewer_r0 is not None else routine_r0

    # Decide viz port
    viz_port = args.viz_port
    if viz_port is None:
        viz_port = 9999 if is_udp_port_free(args.host, 9999) else pick_free_udp_port(args.host)

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    # Build routine command (forward passthru args too)
    routine_cmd = [args.python, "-m", routine_mod, args.routine_viz_flag, str(viz_port)] + passthru
    print("RUN:", " ".join(routine_cmd))
    routine_proc = subprocess.Popen(
        routine_cmd,
        env=env,
        cwd=str(project_root),
        start_new_session=True,  # new process group for reliable signals
    )

    viewer_proc = None
    if not args.no_viewer:
        viewer_cmd = [
            args.python, "-m", args.viewer,
            "--host", args.host,
            args.viewer_port_flag, str(viz_port),
            "--axis_mode", args.viewer_axis_mode,
        ]
        if viewer_r0_effective is not None:
            viewer_cmd += ["--r0", str(viewer_r0_effective)]

        print("RUN:", " ".join(viewer_cmd))
        viewer_proc = subprocess.Popen(
            viewer_cmd,
            env=env,
            cwd=str(project_root),
            start_new_session=True,
        )

    try:
        while True:
            rc = routine_proc.poll()
            if rc is not None:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        terminate_processes(viewer_proc, routine_proc, routine_grace_s=6.0)


if __name__ == "__main__":
    main()
