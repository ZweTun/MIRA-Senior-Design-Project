import serial
import numpy as np
import matplotlib.pyplot as plt

ROWS = 4
COLS = 4

ser = serial.Serial('COM9', 250000, timeout=1)

plt.ion()
fig, ax = plt.subplots()

data = np.zeros((ROWS, COLS))
heatmap = ax.imshow(
    data,
    cmap='hot',
    vmin=0,
    vmax=1023,
    interpolation='nearest',
    zorder=1
)
plt.colorbar(heatmap)

# Multi-target marker (scatter) — many circles
target_scatter = ax.scatter(
    [], [],
    s=200,
    facecolors='none',
    edgecolors='green',
    linewidths=3,
    zorder=2
)

while True:
    row_data = []
    targets = []  # list of (r, c)

    # Read one "frame" until separator ---
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        if line == "---":
            break

        # Target line: T,row,col
        if line.startswith("T,"):
            parts = line.split(",")
            if len(parts) == 3:
                _, r, c = parts
                r, c = int(r), int(c)
                # keep only in-bounds targets
                if 0 <= r < ROWS and 0 <= c < COLS:
                    targets.append((r, c))
            continue

        # Heatmap row
        values = [int(x) for x in line.split(",") if x != ""]
        if len(values) == COLS:
            row_data.append(values)

    if row_data:
        data = np.array(row_data, dtype=np.float32)
        if data.shape == (ROWS, COLS):
            heatmap.set_data(data)

        # Update multi-target marker
        if targets:
            # scatter expects [[x,y], ...] where x=col, y=row
            offsets = np.array([[c, r] for (r, c) in targets], dtype=np.float32)
            target_scatter.set_offsets(offsets)
        else:
            target_scatter.set_offsets(np.empty((0, 2)))

        plt.draw()
        plt.pause(0.01)