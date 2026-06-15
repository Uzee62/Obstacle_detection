"""
visualise.py
============
Real-time polar plot of the world model.
Reads /tmp/world_model.json every 100ms and updates the display.

Run alongside the pipeline:
    python visualise.py

Or point at a different world model file:
    python visualise.py /path/to/world_model.json
"""

import json
import math
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

WORLD_MODEL_PATH = Path("world_model.json")
MAX_RANGE_M      = 32.0    # plot radius
REFRESH_MS       = 100     # how often to redraw (ms)
VESSEL_SIZE      = 0.3     # size of vessel marker at centre

# colours
COL_DYNAMIC  = "#D85A30"   # moving obstacles — orange-red
COL_STATIC   = "#534AB7"   # static obstacles — purple
COL_COASTING = "#BA7517"   # coasting tracks — amber
COL_VESSEL   = "#1D9E75"   # own vessel — green
COL_GRID     = "#404040"   # grid lines

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

fig = plt.figure(figsize=(8, 8), facecolor="#1a1a1a")
ax  = fig.add_subplot(111, projection="polar", facecolor="#1a1a1a")
fig.suptitle("Maritime Perception — World Model", color="white", fontsize=13)

# polar plot setup
# bearing 0° = vessel bow = top of plot
# matplotlib polar: 0° = right, increases counter-clockwise
# we want: 0° = top, increases clockwise (maritime convention)
ax.set_theta_zero_location("N")      # 0° at top
ax.set_theta_direction(-1)           # clockwise
ax.set_rlim(0, MAX_RANGE_M)

# grid styling
ax.set_facecolor("#1a1a1a")
ax.grid(color=COL_GRID, linewidth=0.5, linestyle="--", alpha=0.5)
ax.tick_params(colors="gray", labelsize=8)
ax.spines["polar"].set_color(COL_GRID)

# range rings labels
ax.set_rticks([5, 10, 15, 20, 25, 30])
ax.set_yticklabels(["5m", "10m", "15m", "20m", "25m", "30m"],
                   color="gray", fontsize=7)

# bearing labels
ax.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
ax.set_xticklabels(["N/Bow", "045°", "Port", "135°",
                    "Aft",   "225°", "Stbd", "315°"],
                   color="gray", fontsize=8)

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

# scatter plots — one per category so we can control colours
scat_dynamic  = ax.scatter([], [], s=80,  c=COL_DYNAMIC,  zorder=5,
                           label="Moving",  marker="o", alpha=0.9)
scat_static   = ax.scatter([], [], s=60,  c=COL_STATIC,   zorder=4,
                           label="Static",  marker="s", alpha=0.8)
scat_coasting = ax.scatter([], [], s=50,  c=COL_COASTING, zorder=3,
                           label="Coasting",marker="^", alpha=0.7)

# vessel marker at centre
ax.scatter([0], [0], s=120, c=COL_VESSEL, zorder=10, marker="*")

# text annotations — track IDs and speed
annotations = []

# status text in the corner
status_text = ax.text(
    0.01, 0.99, "",
    transform    = ax.transAxes,
    color        = "white",
    fontsize     = 8,
    verticalalignment = "top",
    fontfamily   = "monospace",
)

# legend
legend = ax.legend(
    loc            = "lower right",
    facecolor      = "#2a2a2a",
    edgecolor      = "gray",
    labelcolor     = "white",
    fontsize       = 8,
    framealpha     = 0.8,
)

plt.tight_layout()


# ---------------------------------------------------------------------------
# Read world model
# ---------------------------------------------------------------------------

def read_world_model(path: Path) -> dict | None:
    try:
        text = path.read_text(encoding="utf-8")
        return json.loads(text)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


# ---------------------------------------------------------------------------
# Update function — called every REFRESH_MS
# ---------------------------------------------------------------------------

def update(_):
    global annotations

    # clear old annotations
    for ann in annotations:
        ann.remove()
    annotations = []

    data = read_world_model(WORLD_MODEL_PATH)

    if data is None:
        status_text.set_text("waiting for world model...")
        scat_dynamic.set_offsets(np.empty((0, 2)))
        scat_static.set_offsets(np.empty((0, 2)))
        scat_coasting.set_offsets(np.empty((0, 2)))
        return

    objects = data.get("objects", [])

    dynamic_pts  = []
    static_pts   = []
    coasting_pts = []

    for obj in objects:
        bearing_rad = math.radians(obj["bearing_deg"])
        range_m     = obj["range_m"]
        coasting    = obj.get("coasting", False)
        dynamic     = obj.get("dynamic", False)
        conf        = obj.get("confidence", 0)
        speed       = obj.get("speed_ms", 0)
        oid         = obj.get("id", "?")
        safety_r    = obj.get("safety_radius_m", 0)

        pt = [bearing_rad, range_m]

        if coasting:
            coasting_pts.append(pt)
        elif dynamic:
            dynamic_pts.append(pt)
        else:
            static_pts.append(pt)

        # draw safety radius circle
        theta = np.linspace(0, 2 * math.pi, 60)
        # convert safety radius from metres to plot units
        # in polar plot, radius is in metres, so we draw a circle
        # centred at (bearing_rad, range_m) in cartesian then convert
        cx = range_m * math.sin(bearing_rad)
        cy = range_m * math.cos(bearing_rad)
        circle_x = cx + safety_r * np.cos(theta)
        circle_y = cy + safety_r * np.sin(theta)
        # convert back to polar
        circle_r   = np.hypot(circle_x, circle_y)
        circle_th  = np.arctan2(circle_x, circle_y)
        ax.plot(circle_th, circle_r,
                color    = COL_DYNAMIC if dynamic else COL_STATIC,
                linewidth= 0.6,
                alpha    = 0.3,
                zorder   = 2)

        # draw velocity arrow for moving objects
        if dynamic and speed > 0.1:
            vx  = obj["velocity"]["x"]
            vy  = obj["velocity"]["y"]
            # scale arrow: 1 second of travel
            ex  = cx + vx
            ey  = cy + vy
            er  = math.hypot(ex, ey)
            eth = math.atan2(ex, ey)
            ann = ax.annotate(
                "",
                xy       = (eth, er),
                xytext   = (bearing_rad, range_m),
                arrowprops = dict(
                    arrowstyle = "->",
                    color      = COL_DYNAMIC,
                    lw         = 1.2,
                ),
                zorder = 6,
            )
            annotations.append(ann)

        # track ID and speed label
        label = f"#{oid}"
        if dynamic:
            label += f"\n{speed:.1f}m/s"

        ann = ax.annotate(
            label,
            xy         = (bearing_rad, range_m),
            xytext     = (5, 5),
            textcoords = "offset points",
            color      = "white",
            fontsize   = 7,
            fontfamily = "monospace",
            zorder     = 7,
        )
        annotations.append(ann)

    # update scatter plots
    def to_array(pts):
        if pts:
            return np.array(pts)
        return np.empty((0, 2))

    scat_dynamic.set_offsets(to_array(dynamic_pts))
    scat_static.set_offsets(to_array(static_pts))
    scat_coasting.set_offsets(to_array(coasting_pts))

    # status panel
    scan_id    = data.get("scan_id", 0)
    latency    = data.get("latency_ms", 0)
    n_objects  = data.get("object_count", 0)
    timestamp  = data.get("header", {}).get("timestamp_ns", 0)

    status_text.set_text(
        f"scan     {scan_id:06d}\n"
        f"objects  {n_objects}\n"
        f"latency  {latency:.1f}ms\n"
        f"path     {WORLD_MODEL_PATH.name}"
    )


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

from matplotlib.animation import FuncAnimation

ani = FuncAnimation(
    fig,
    update,
    interval = REFRESH_MS,
    cache_frame_data = False,
)

plt.show()