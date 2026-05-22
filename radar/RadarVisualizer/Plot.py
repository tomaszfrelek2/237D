import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D plotting

CSV_FILE = "radar_points.csv"

MAX_RANGE_M = 3.0

# Number of recent frames to show together
WINDOW_FRAMES = 10

# Axis limits
SIDE_LIMIT_M = 3.0
FORWARD_MIN_M = 0.0
FORWARD_MAX_M = 3.0
HEIGHT_MIN_M = -1.5
HEIGHT_MAX_M = 2.5

# Load radar CSV
df = pd.read_csv(CSV_FILE)

# Basic filtering
df = df[df["range_m"] <= MAX_RANGE_M]
df = df[df["forward_m"] >= FORWARD_MIN_M]
df = df[df["forward_m"] <= FORWARD_MAX_M]

frames = sorted(df["frame"].unique())

if len(frames) == 0:
    raise ValueError("No radar points found after filtering.")


# Frame helpers
def get_window_data(index):
    """
    Returns all points from the last WINDOW_FRAMES frames.
    This makes the radar point cloud easier to see.
    """
    start_index = max(0, index - WINDOW_FRAMES + 1)
    selected_frames = frames[start_index:index + 1]

    current_frame = frames[index]
    frame_df = df[df["frame"].isin(selected_frames)]

    return current_frame, frame_df

# Plot setup
fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection="3d")

plt.subplots_adjust(bottom=0.23)

current_index = 0


def draw_frame(index):
    ax.cla()

    frame_num, frame_df = get_window_data(index)

    xs = frame_df["side_m"]
    ys = frame_df["forward_m"]
    zs = frame_df["height_m"]

    # Radar points
    scatter = ax.scatter(xs, ys, zs, s=25, alpha=0.75)

    # Radar origin
    ax.scatter([0], [0], [0], s=100, marker="^")
    ax.text(0, 0, 0, " Radar")

    ax.set_title(
        f"3D Radar Point Cloud | Frame {frame_num} | "
        f"Showing last {WINDOW_FRAMES} frames | Points: {len(frame_df)}"
    )

    # Coordinate meaning:
    # X = side-to-side
    # Y = forward distance
    # Z = height
    ax.set_xlabel("Side / left-right (m)")
    ax.set_ylabel("Forward (m)")
    ax.set_zlabel("Height (m)")

    ax.set_xlim(-SIDE_LIMIT_M, SIDE_LIMIT_M)
    ax.set_ylim(FORWARD_MIN_M, FORWARD_MAX_M)
    ax.set_zlim(HEIGHT_MIN_M, HEIGHT_MAX_M)

    # Keeps the 3D box proportions reasonable
    ax.set_box_aspect((
        SIDE_LIMIT_M * 2,
        FORWARD_MAX_M - FORWARD_MIN_M,
        HEIGHT_MAX_M - HEIGHT_MIN_M
    ))

    ax.grid(True)


draw_frame(current_index)


# Slider to view frames
slider_ax = plt.axes([0.15, 0.11, 0.7, 0.03])

frame_slider = Slider(
    ax=slider_ax,
    label="Frame",
    valmin=0,
    valmax=len(frames) - 1,
    valinit=current_index,
    valstep=1
)


def update_slider(val):
    index = int(frame_slider.val)
    draw_frame(index)
    fig.canvas.draw_idle()


frame_slider.on_changed(update_slider)


# Previous / Next buttons
prev_ax = plt.axes([0.15, 0.04, 0.15, 0.05])
next_ax = plt.axes([0.70, 0.04, 0.15, 0.05])

prev_button = Button(prev_ax, "Previous")
next_button = Button(next_ax, "Next")


def previous_frame(event):
    current = int(frame_slider.val)
    if current > 0:
        frame_slider.set_val(current - 1)


def next_frame(event):
    current = int(frame_slider.val)
    if current < len(frames) - 1:
        frame_slider.set_val(current + 1)


prev_button.on_clicked(previous_frame)
next_button.on_clicked(next_frame)


plt.show()