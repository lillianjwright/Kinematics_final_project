

# arm_sim.py
# 6-DOF kinematic simulator (Standard DH, degrees)
# - Fixed axes centered at base (no recentering)
# - Sliders for q1..q6 (deg)
# - Play/Stop demo
# - Red line shows tool-frame X axis (visualizes wrist rotation)

# If your window doesn't appear, uncomment the two lines below and pip install a GUI backend:
# import matplotlib
# matplotlib.use('QtAgg')  # or 'TkAgg'
import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# -------------------- DH (Standard DH, degrees) --------------------
# [a, alpha_deg, d, thetaOffset_deg]
DH_deg = np.array([
    
    [   0.0,  90.0,   0.0, 0.0],   # 2
    [ 125.0,    0.0,   2.5, 0.0],   # 3
    [  125.5,  -90.0,   3.0, 0.0],   # 4

], dtype=float)

# joint limits (deg) — tune these to your hardware
JLIM = np.array([
    
    [0, 180],
    [0, 180],
    [-180, 0],
    
  
], dtype=float)

# scale for drawing bounds
TOTAL_LEN = float(np.sum(np.abs(DH_deg[:,0])) + np.sum(np.abs(DH_deg[:,2])))
if TOTAL_LEN < 1.0: TOTAL_LEN = 1.0
R = TOTAL_LEN * 1.2

# -------------------- kinematics helpers --------------------------
def dh_std(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ], dtype=float)

def fk_all(DH_deg, q_deg):
    """Returns:
       P : (7,3) points of frames {0..6} origins
       Ts: list of 7 transforms T0, T1, ..., T6 (each 4x4)
    """
    q = np.deg2rad(q_deg)
    a   = DH_deg[:,0]
    d   = DH_deg[:,2]
    alp = np.deg2rad(DH_deg[:,1])
    th0 = np.deg2rad(DH_deg[:,3])

    Ts = []
    T  = np.eye(4)
    Ts.append(T.copy())
    for i in range(3):
        T = T @ dh_std(a[i], alp[i], d[i], th0[i] + q[i])
        Ts.append(T.copy())
    P = np.vstack([Ti[:3,3] for Ti in Ts])
    return P, Ts

# -------------------- small drawing helpers -----------------------
def make_frame_artists(ax, n_frames, scale=20.0):
    """create 3*n Line3D artists (Rx,Gy,Bz) for triads"""
    arts = []
    for _ in range(n_frames):
        rx, = ax.plot([0,0],[0,0],[0,0], 'r-', lw=2)
        gy, = ax.plot([0,0],[0,0],[0,0], 'g-', lw=2)
        bz, = ax.plot([0,0],[0,0],[0,0], 'b-', lw=2)
        arts.append((rx,gy,bz))
    return arts

def update_frame_artists(arts, Ts, scale=20.0):
    """update triads using transforms Ts (len=7)"""
    for k, T in enumerate(Ts):
        o = T[:3,3]
        Rm = T[:3,:3]
        axes = Rm * scale
        # X (red)
        arts[k][0].set_data([o[0], o[0]+axes[0,0]], [o[1], o[1]+axes[1,0]])
        arts[k][0].set_3d_properties([o[2], o[2]+axes[2,0]])
        # Y (green)
        arts[k][1].set_data([o[0], o[0]+axes[0,1]], [o[1], o[1]+axes[1,1]])
        arts[k][1].set_3d_properties([o[2], o[2]+axes[2,1]])
        # Z (blue)
        arts[k][2].set_data([o[0], o[0]+axes[0,2]], [o[1], o[1]+axes[1,2]])
        arts[k][2].set_3d_properties([o[2], o[2]+axes[2,2]])

def tool_ring_points(Ttool, radius=15.0, n=64):
    """circle points in tool plane (Y–Z plane of tool frame) centered at tool origin"""
    th = np.linspace(0, 2*np.pi, n)
    # circle in tool frame: along Y and Z axes
    pts_local = np.vstack([np.zeros_like(th),
                           radius*np.cos(th),
                           radius*np.sin(th)]).T
    R = Ttool[:3,:3]; o = Ttool[:3,3]
    pts = (pts_local @ R.T) + o
    return pts

# -------------------- figure & ui --------------------------------
plt.close('all')
fig = plt.figure(figsize=(10.5, 6.2))
ax  = fig.add_subplot(121, projection='3d')
ui  = fig.add_subplot(122); ui.axis('off')

q0 = np.zeros(6)
P, Ts = fk_all(DH_deg, q0)

# base + links + joints
base_dot, = ax.plot([0],[0],[0], 'ko', ms=6)
(link_line,) = ax.plot(P[:,0], P[:,1], P[:,2], '-', lw=4, color='#246')
joints = ax.scatter(P[:,0], P[:,1], P[:,2], s=30, c='#135', depthshade=True)

# tool axis + tool ring
tool_axis, = ax.plot([P[-1,0], P[-1,0]+20], [P[-1,1], P[-1,1]], [P[-1,2], P[-1,2]], 'r-', lw=2)
ring_pts = tool_ring_points(Ts[-1], radius=15.0, n=100)
tool_ring, = ax.plot(ring_pts[:,0], ring_pts[:,1], ring_pts[:,2], color='0.6', lw=1.5, alpha=0.9)

# triads at all frames {0..6}
triad_scale = 18.0
triads = make_frame_artists(ax, n_frames=7, scale=triad_scale)
update_frame_artists(triads, Ts, scale=triad_scale)

# fixed bounds
ax.set_xlim([-R, R]); ax.set_ylim([-R, R]); ax.set_zlim([-R, R])
ax.set_box_aspect((1,1,1))
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('6-DOF Arm (Standard DH) — Triads show orientation')

# -------------------- sliders -----------------------------------
sliders = []
labels  = [f"q{i+1}" for i in range(6)]
y0 = 0.80
for i in range(3):
    rect = [0.60, y0 - i*0.10, 0.35, 0.04]
    s = Slider(fig.add_axes(rect), f"{labels[i]} (deg)", JLIM[i,0], JLIM[i,1], valinit=q0[i], valstep=1.0)
    sliders.append(s)

# -------------------- buttons -----------------------------------
ax_play = fig.add_axes([0.60, 0.05, 0.15, 0.06])
ax_stop = fig.add_axes([0.80, 0.05, 0.15, 0.06])
btn_play = Button(ax_play, 'Play'); btn_stop = Button(ax_stop, 'Stop')
is_anim = {'run': False}; phase = {'t': 0.0}

# -------------------- redraw ------------------------------------
def redraw():
    q = np.array([s.val for s in sliders], dtype=float)
    P, Ts = fk_all(DH_deg, q)

    # links & joints
    link_line.set_data(P[:,0], P[:,1]); link_line.set_3d_properties(P[:,2])
    joints._offsets3d = (P[:,0], P[:,1], P[:,2])

    # tool axis (tool X) & ring
    tip = P[-1]; Rm = Ts[-1][:3,:3]
    v = Rm[:,0] * 22.0
    tool_axis.set_data([tip[0], tip[0]+v[0]], [tip[1], tip[1]+v[1]])
    tool_axis.set_3d_properties([tip[2], tip[2]+v[2]])

    ring = tool_ring_points(Ts[-1], radius=15.0, n=120)
    tool_ring.set_data(ring[:,0], ring[:,1]); tool_ring.set_3d_properties(ring[:,2])

    # triads at every frame
    update_frame_artists(triads, Ts, scale=triad_scale)

    ax.set_xlim([-R, R]); ax.set_ylim([-R, R]); ax.set_zlim([-R, R])
    fig.canvas.draw_idle()

for s in sliders:
    s.on_changed(lambda _v: redraw())

# -------------------- animation ---------------------------------
def on_play(_e): is_anim['run'] = True
def on_stop(_e): is_anim['run'] = False
btn_play.on_clicked(on_play); btn_stop.on_clicked(on_stop)

def timer_cb(_evt):
    if not is_anim['run']: return
    phase['t'] += 0.06
    q_demo = []
    for i in range(3):
        mid = 0.5*(JLIM[i,0]+JLIM[i,1])
        amp = 0.45*(JLIM[i,1]-JLIM[i,0])
        q_demo.append(mid + amp*np.sin(phase['t'] + i*0.6))
    for i,s in enumerate(sliders): s.set_val(q_demo[i])  # triggers redraw

timer = fig.canvas.new_timer(interval=40); timer.add_callback(timer_cb, None); timer.start()

plt.tight_layout()
plt.show()