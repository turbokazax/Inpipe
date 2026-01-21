# Re-vibecoded as of Jan 21
import socket
import math
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


HOST, PORT = "0.0.0.0", 9999

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
sock.setblocking(False)

TRAIL_POINTS = 1200
xs = deque(maxlen=TRAIL_POINTS)
ys = deque(maxlen=TRAIL_POINTS)

cur_x = None
cur_y = None
cur_state = "?"
r = None          # latest reported radius
r_fixed = None    # axis limits are based on this and NEVER shrink

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


def set_radius(rr: float):
    """
    rr = current radius from sender.
    We lock axes to the FIRST (largest) radius, so the plot never zooms in.
    """
    global r, r_fixed
    r = rr

    if r_fixed is None:
        r_fixed = rr
        ax.set_xlim(-r_fixed * 1.10, r_fixed * 1.10)
        ax.set_ylim(-r_fixed * 1.10, r_fixed * 1.10)

    # Optional: keep circle showing the *current* radius (spiral reference)
    draw_circle(rr)

    # If you prefer showing ONLY the initial circle instead, replace the line above with:
    # if rr == r_fixed: draw_circle(rr)


def drain_udp():
    global cur_x, cur_y, cur_state, last_pkt_time, pkt_count

    while True:
        try:
            data, _addr = sock.recvfrom(2048)
        except BlockingIOError:
            break

        s = data.decode(errors="ignore").strip()
        if not s:
            continue

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

        # set radius, but DO NOT shrink axes
        if r is None:
            set_radius(rr)
        else:
            # Update displayed circle if radius changes; axes stay fixed
            if rr != r:
                set_radius(rr)

        cur_x, cur_y, cur_state = x, y, st
        xs.append(x)
        ys.append(y)

        pkt_count += 1
        last_pkt_time = time.time()

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
            f"state: {cur_state}\n"
            f"trail pts: {len(xs)}\n"
            f"r(now): {int(r) if r is not None else '?'} ticks\n"
            f"r(fixed): {int(r_fixed) if r_fixed is not None else '?'} ticks\n"
            f"last packet: {age:.3f}s ago\n"
            f"pkts: {pkt_count}"
        )

    return (circle_line, trail_line, marker_line, state_text)


ani = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
plt.show()
