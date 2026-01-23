import socket
import math
import time
from collections import deque
from Misc.deg import deg

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# -----------------------------
# USER OPTIONS
# -----------------------------
AXIS_MODE = "dynamic"     # "locked" or "dynamic"
DISPLAY_RADIUS_INIT = deg(270)
# Examples:
#   DISPLAY_RADIUS_INIT = 303750   # force big axes (good for reverse spiral starting at 0,0)
#   DISPLAY_RADIUS_INIT = None     # auto: use first received rr (or grow if dynamic)

DYNAMIC_MARGIN = 1.10    # axis padding factor
TRAIL_POINTS = 3000      # longer trail for spirals

HOST, PORT = "0.0.0.0", 9999
# -----------------------------


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
sock.setblocking(False)

xs = deque(maxlen=TRAIL_POINTS)
ys = deque(maxlen=TRAIL_POINTS)

cur_x = None
cur_y = None
cur_state = "?"
r_now = None

# axis control state
r_fixed = DISPLAY_RADIUS_INIT  # locked radius for axes (if locked mode)
axis_set = False               # whether xlim/ylim have been initialized

last_pkt_time = None
pkt_count = 0

fig, ax = plt.subplots()
ax.set_aspect("equal", adjustable="box")
ax.set_title("XY Trace (ticks)")

(circle_line,) = ax.plot([], [])                          # reference circle (optional)
(trail_line,) = ax.plot([], [])                           # trail
(marker_line,) = ax.plot([], [], marker="o", linestyle="")  # current marker
state_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")


def draw_circle(rr: float):
    n = 600
    t = [i * (2 * math.pi / n) for i in range(n + 1)]
    cx = [rr * math.cos(a) for a in t]
    cy = [rr * math.sin(a) for a in t]
    circle_line.set_data(cx, cy)


def set_axes_for_radius(rr: float):
    ax.set_xlim(-rr * DYNAMIC_MARGIN, rr * DYNAMIC_MARGIN)
    ax.set_ylim(-rr * DYNAMIC_MARGIN, rr * DYNAMIC_MARGIN)


def consider_axes_update(rr: float):
    """
    Updates axes depending on AXIS_MODE and DISPLAY_RADIUS_INIT/r_fixed.
    - locked: axes never change after initialization (unless user provided DISPLAY_RADIUS_INIT)
    - dynamic: axes expand or shrink to rr (or to max(|x|,|y|) if rr is tiny/0)
    """
    global r_fixed, axis_set

    # If user forced an initial display radius, use it to initialize axes immediately.
    if not axis_set and r_fixed is not None:
        set_axes_for_radius(r_fixed)
        axis_set = True

    if AXIS_MODE == "locked":
        # If we still haven't set axes, lock to the first received rr (or fallback).
        if not axis_set:
            r_fixed = rr if rr is not None else 1.0
            set_axes_for_radius(r_fixed)
            axis_set = True
        return

    # AXIS_MODE == "dynamic"
    # Initialize if needed, using either forced r_fixed, else first rr.
    if not axis_set:
        base = r_fixed if r_fixed is not None else (rr if rr is not None else 1.0)
        set_axes_for_radius(base)
        axis_set = True
        return

    # Dynamic mode: follow rr, but be robust if rr is 0 or not informative.
    # Use whichever is larger: rr or current point radius.
    if cur_x is not None and cur_y is not None:
        point_r = max(abs(cur_x), abs(cur_y))
    else:
        point_r = 0.0

    target_r = max(rr if rr is not None else 0.0, point_r, 1.0)
    set_axes_for_radius(target_r)


def drain_udp():
    global cur_x, cur_y, cur_state, r_now, last_pkt_time, pkt_count

    while True:
        try:
            data, _addr = sock.recvfrom(2048)
        except BlockingIOError:
            break

        s = data.decode(errors="ignore").strip()
        if not s:
            continue

        # expected: x,y,r,state
        parts = s.split(",")
        if len(parts) < 3:
            continue

        try:
            x = float(parts[0])
            y = float(parts[1])
            rr = float(parts[2])
            st = parts[3] if len(parts) >= 4 else "?"
        except ValueError:
            continue

        cur_x, cur_y, cur_state = x, y, st
        r_now = rr

        xs.append(x)
        ys.append(y)

        pkt_count += 1
        last_pkt_time = time.time()

        # axes update
        consider_axes_update(rr)

        # circle display:
        # - In locked mode: draw the locked radius if known (nice frame reference)
        # - In dynamic mode: draw current rr (if you want), else comment out
        if AXIS_MODE == "locked":
            if r_fixed is not None:
                draw_circle(r_fixed)
        else:
            draw_circle(max(rr, 1.0))

        if pkt_count <= 5:
            print(f"[pkt {pkt_count}] x={x:.0f} y={y:.0f} r={rr:.0f} state={st}")


def update(_frame):
    drain_udp()

    if xs:
        trail_line.set_data(list(xs), list(ys))

    if cur_x is not None and cur_y is not None:
        marker_line.set_data([cur_x], [cur_y])

    if last_pkt_time is None:
        state_text.set_text("waiting for UDP packets on port 9999...")
    else:
        age = time.time() - last_pkt_time
        state_text.set_text(
            f"axis_mode: {AXIS_MODE}\n"
            f"state: {cur_state}\n"
            f"trail pts: {len(xs)}\n"
            f"r(now): {int(r_now) if r_now is not None else '?'} ticks\n"
            f"r(fixed/init): {int(r_fixed) if r_fixed is not None else '?'} ticks\n"
            f"last packet: {age:.3f}s ago\n"
            f"pkts: {pkt_count}"
        )

    return (circle_line, trail_line, marker_line, state_text)


ani = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
plt.show()
