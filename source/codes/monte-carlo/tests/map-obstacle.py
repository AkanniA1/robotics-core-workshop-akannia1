#!/usr/bin/env python3
"""
make_map.py — generate a simple occupancy map + YAML for ROS 2 (Nav2/AMCL)
- Obstacles are black (occupied)
- Free space is white (free)
- Output: map.png, map.yaml
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import yaml

# -----------------------
# Map config (edit here)
# -----------------------
width_m     = 10.0        # map width in meters
height_m    = 10.0        # map height in meters
resolution  = 0.05        # meters per pixel (e.g., 5 cm)
origin_xyz  = [0.0, 0.0, 0.0]  # map origin in ROS (x, y, yaw)

# Obstacles to draw (in meters).
# Rectangles: (x, y, w, h) with (x,y) = bottom-left corner
rectangles_m = [
    (2.0, 2.0, 1.0, 3.0),
    (5.0, 5.5, 3.0, 1.0),
]

# Circles: (cx, cy, r)
circles_m = [
    (7.5, 3.0, 0.6),
]

# Borders around the map?
add_border = True

# Output file names
image_name = "map.png"
yaml_name  = "map.yaml"

# -----------------------
# Build the grid
# -----------------------
W = int(round(width_m  / resolution))
H = int(round(height_m / resolution))
grid_occ = np.zeros((H, W), dtype=np.uint8)  # 0 = free, 1 = occupied

def meters_to_px(x_m, y_m):
    """Convert (x,y) in meters to grid indices (col=x, row=y), origin at (0,0) bottom-left."""
    col = int(round(x_m / resolution))
    row = int(round(y_m / resolution))
    # Our grid index 0 is top-left; flip Y for drawing convenience later
    return col, row

# Add border walls
if add_border:
    grid_occ[0, :]  = 1
    grid_occ[-1, :] = 1
    grid_occ[:, 0]  = 1
    grid_occ[:, -1] = 1

# Draw rectangles
for (x, y, w, h) in rectangles_m:
    x0_px, y0_px = meters_to_px(x, y)
    x1_px, y1_px = meters_to_px(x + w, y + h)
    # Convert to row/col with image coords: row 0 is top, so flip vertically
    r0 = H - y1_px
    r1 = H - y0_px
    c0 = x0_px
    c1 = x1_px
    grid_occ[max(0,r0):min(H,r1), max(0,c0):min(W,c1)] = 1

# Draw circles
yy, xx = np.indices((H, W))
for (cx, cy, r) in circles_m:
    cx_px, cy_px = meters_to_px(cx, cy)
    # Flip cy to image row
    cy_row = H - cy_px
    mask = (xx - cx_px)**2 + (yy - cy_row)**2 <= (r / resolution)**2
    grid_occ[mask] = 1

# -----------------------
# Save image (PNG)
# For ROS maps: black=occupied (0), white=free (255)
# -----------------------
img = np.where(grid_occ == 1, 0, 255).astype(np.uint8)

# Use matplotlib to write PNG without axes
plt.imsave(image_name, img, cmap="gray", vmin=0, vmax=255)

# -----------------------
# Write YAML for Nav2/AMCL
# -----------------------
yaml_data = {
    "image": image_name,
    "resolution": float(resolution),
    "origin": [float(origin_xyz[0]), float(origin_xyz[1]), float(origin_xyz[2])],
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
    "negate": 0
}

with open(yaml_name, "w") as f:
    yaml.safe_dump(yaml_data, f, sort_keys=False)

print(f"✅ Wrote {image_name} ({W}x{H} px), resolution={resolution} m/px")
print(f"✅ Wrote {yaml_name}")
