import serial
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import threading
import time

# ===== CONFIG =====
SERIAL_PORT = 'COM7'      # Windows: COM7
# SERIAL_PORT = '/dev/ttyACM0'  # Linux
BAUD_RATE = 115200

# ===== SERIAL + THREADING =====
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# shared data (protected by lock)
data_lock = threading.Lock()
last_pitch = 0.0
last_roll = 0.0
last_yaw = 0.0

# regex pattern
pattern = re.compile(
    r'Pitch=(-?\d+)\.(\d+).*Roll=(-?\d+)\.(\d+).*Yaw=(-?\d+)\.(\d+)'
)

# thread function: continuously read serial data
def read_serial():
    global last_pitch, last_roll, last_yaw
    while True:
        try:
            line = ser.readline().decode(errors='ignore')
            match = pattern.search(line)
            if match:
                pitch = float(match.group(1) + "." + match.group(2))
                roll  = float(match.group(3) + "." + match.group(4))
                yaw   = float(match.group(5) + "." + match.group(6))
                
                with data_lock:
                    last_pitch = pitch
                    last_roll = roll
                    last_yaw = yaw
        except Exception as e:
            time.sleep(0.01)  # avoid spin-lock

# start serial reader thread (daemon so it dies with main)
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# ===== PLOT SETUP =====
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# set up 3D axes
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(-1.5, 1.5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Drone Orientation")

# draw reference frame axes (RGB)
ax.quiver(0, 0, 0, 1.2, 0, 0, color='r', arrow_length_ratio=0.1, linewidth=2, label='X')
ax.quiver(0, 0, 0, 0, 1.2, 0, color='g', arrow_length_ratio=0.1, linewidth=2, label='Y')
ax.quiver(0, 0, 0, 0, 0, 1.2, color='b', arrow_length_ratio=0.1, linewidth=2, label='Z')

# define drone geometry - quadcopter with fuselage and 4 arms
# fuselage body (central box)
fuselage_verts = np.array([
    [-0.2, -0.12, -0.08], [-0.2, -0.12, 0.08], [-0.2, 0.12, -0.08], [-0.2, 0.12, 0.08],
    [0.2, -0.12, -0.08], [0.2, -0.12, 0.08], [0.2, 0.12, -0.08], [0.2, 0.12, 0.08]
])
# arm tips (4 quadcopter motors)
arm_tips = np.array([
    [0.9, 0.9, 0], [-0.9, 0.9, 0], [-0.9, -0.9, 0], [0.9, -0.9, 0]
])
# all vertices: fuselage + motor positions + origin center
origin = np.array([[0, 0, 0]])
box_verts = np.vstack([fuselage_verts, arm_tips, origin])
# fuselage faces + arm indicators from center origin
box_faces = [
    [0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4],
    [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5],
    # arm lines from center (vertex 12 = origin) to motor tips (8,9,10,11)
    [12, 8, 8, 12], [12, 9, 9, 12], [12, 10, 10, 12], [12, 11, 11, 12]
]

# create initial box collection
box_polys = [[box_verts[idx] for idx in face] for face in box_faces]
box_collection = Poly3DCollection(box_polys, facecolors='cyan', edgecolors='k', alpha=0.6, linewidth=1)
ax.add_collection3d(box_collection)

# removed: create drone arrow (will be updated)
# drone_arrow = ax.quiver(0, 0, 0, 1, 0, 0, color='cyan', arrow_length_ratio=0.15, linewidth=3)

# text for angles
text_angles = ax.text2D(0.05, 0.95, "", transform=ax.transAxes,
                        fontsize=12, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

ax.legend(loc='upper left')
fig.tight_layout()

# ===== UPDATE FUNCTION =====
def update(frame):
    global last_pitch, last_roll, last_yaw
    
    # read current values with thread-safe lock
    with data_lock:
        pitch = last_pitch
        roll = last_roll
        yaw = last_yaw
    
    # convert to radians
    p = np.deg2rad(pitch)
    r = np.deg2rad(roll)
    y = np.deg2rad(yaw)
    
    # rotation matrices for ZYX (yaw-pitch-roll)
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(r), -np.sin(r)],
                    [0, np.sin(r),  np.cos(r)]])
    R_y = np.array([[ np.cos(p), 0, np.sin(p)],
                    [0, 1, 0],
                    [-np.sin(p), 0, np.cos(p)]])
    R_z = np.array([[np.cos(y), -np.sin(y), 0],
                    [np.sin(y),  np.cos(y), 0],
                    [0, 0, 1]])
    
    # combined rotation: apply roll, pitch, then yaw
    R = R_z @ R_y @ R_x
    
    # rotate box vertices
    rotated_verts = (R @ box_verts.T).T
    new_faces = [[rotated_verts[idx] for idx in face] for face in box_faces]
    
    # update box collection
    box_collection.set_verts(new_faces)
    
    body_x = R @ np.array([1, 0, 0])
    body_y = R @ np.array([0, 1, 0])
    body_z = R @ np.array([0, 0, 1])
    
    if hasattr(update, 'bx'):
        update.bx.remove()
        update.by.remove()
        update.bz.remove()
    
    update.bx = ax.quiver(0, 0, 0, body_x[0]*0.6, body_x[1]*0.6, body_x[2]*0.6, color='red', arrow_length_ratio=0.2, linewidth=2.5)
    update.by = ax.quiver(0, 0, 0, body_y[0]*0.6, body_y[1]*0.6, body_y[2]*0.6, color='green', arrow_length_ratio=0.2, linewidth=2.5)
    update.bz = ax.quiver(0, 0, 0, body_z[0]*0.6, body_z[1]*0.6, body_z[2]*0.6, color='blue', arrow_length_ratio=0.2, linewidth=2.5)
    
    # update text
    text_angles.set_text(f"Pitch: {pitch:.1f}°\nRoll: {roll:.1f}°\nYaw: {yaw:.1f}°")
    
    return box_collection, text_angles, update.bx, update.by, update.bz


# ===== ANIMATION =====
ani = animation.FuncAnimation(
    fig,
    update,
    interval=20,  # 20ms = ~50 Hz (UI, not limited by serial anymore)
    blit=False
)

plt.show()
