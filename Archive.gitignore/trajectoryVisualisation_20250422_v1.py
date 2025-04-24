import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from io import StringIO

# === Franka Emika Panda DH parameters ===
DH_PARAMS = [
    [0,        0,       0.333,  0],
    [0,    -np.pi,    0,      0],
    [0,     np.pi/2,    0.316,  0],
    [0.0825, np.pi/2,   0,      0],
    [-0.0825,-np.pi/2,  0.384,  0],
    [0,      np.pi/2,   0,      0],
    [0.088,  np.pi/2,   0.107,  0]
]

def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics_franka(joint_angles):
    T = np.eye(4)
    positions = [T[:3, 3]]
    for i, (a, alpha, d, _) in enumerate(DH_PARAMS):
        T = T @ dh_transform(a, alpha, d, joint_angles[i])
        positions.append(T[:3, 3])
    return np.array(positions)

# === Load CSV File with Multiple Trajectories ===
def load_trajectories(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        raw = f.read()

    chunks = [chunk for chunk in raw.strip().split('\n\n') if chunk.strip()]
    print(f"Loaded {len(chunks)} trajectory blocks")

    trajectories = []
    for chunk in chunks:
        df = pd.read_csv(StringIO(chunk), sep=';')
        df.columns = df.columns.str.strip().str.lower()
        if df.shape[1] >= 10 and df.shape[0] > 1:
            joint_cols = df.columns[1:8]  # joint_1 to joint_7
            xyz_cols = df.columns[8:11]   # x, y, z
            df_combined = pd.concat([
                df[joint_cols].apply(pd.to_numeric, errors='coerce'),
                df[xyz_cols].apply(pd.to_numeric, errors='coerce')
            ], axis=1).dropna()
            trajectories.append(df_combined.reset_index(drop=True))
    return trajectories

# === Load Your Data ===
csv_path = "gelenkwinkel_mehrere.csv"  # adjust path as needed
trajectories = load_trajectories(csv_path)

# === Initial Trajectory ===
current_traj = trajectories[0]
joint_angle_data = current_traj.iloc[:, :7].to_numpy()
xyz_data = current_traj.iloc[:, 7:10].to_numpy()

# === Plotting ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
line_fk, = ax.plot([], [], [], 'o-', lw=2, color='blue', label="FK Arm")
line_xyz, = ax.plot([], [], [], '-', lw=1.5, color='orange', label="Recorded x,y,z")

ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([0.0, 1.5])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Franka FK vs. Recorded Trajectory")
ax.legend()

# === Display Joint Angles and XYZ ===
joint_text = fig.text(0.02, 0.75, '', fontsize=10, family='monospace')
xyz_text = fig.text(0.02, 0.65, '', fontsize=10, family='monospace')

# === Sliders ===
ax_slider_step = plt.axes([0.2, 0.04, 0.65, 0.03])
slider_step = Slider(ax_slider_step, 'Step', 0, len(joint_angle_data)-1, valinit=0, valstep=1)

ax_slider_traj = plt.axes([0.2, 0.00, 0.65, 0.03])
slider_traj = Slider(ax_slider_traj, 'Trajektorie', 0, len(trajectories)-1, valinit=0, valstep=1)

# === Update Plot ===
def update_plot():
    idx = int(slider_step.val)
    angles = joint_angle_data[idx]
    fk_points = forward_kinematics_franka(angles)
    real_xyz = xyz_data[:idx+1]

    # Update lines
    line_fk.set_data(fk_points[:, 0], fk_points[:, 1])
    line_fk.set_3d_properties(fk_points[:, 2])

    line_xyz.set_data(real_xyz[:, 0], real_xyz[:, 1])
    line_xyz.set_3d_properties(real_xyz[:, 2])

    # === Update text boxes ===
    joint_display = '\n'.join([f'joint_{i+1}: {angle:.3f} rad' for i, angle in enumerate(angles)])
    joint_text.set_text(f"Joint Angles (rad):\n{joint_display}")

    xyz = xyz_data[idx]
    xyz_display = f"x: {xyz[0]:.4f}\ny: {xyz[1]:.4f}\nz: {xyz[2]:.4f}"
    xyz_text.set_text(f"Recorded EEF Position:\n{xyz_display}")

    fig.canvas.draw_idle()

def on_step_change(val):
    update_plot()

def on_traj_change(val):
    global current_traj, joint_angle_data, xyz_data
    current_traj = trajectories[int(val)]
    joint_angle_data = current_traj.iloc[:, :7].to_numpy()
    xyz_data = current_traj.iloc[:, 7:10].to_numpy()
    slider_step.valmax = len(joint_angle_data) - 1
    slider_step.set_val(0)
    slider_step.ax.set_xlim(slider_step.valmin, slider_step.valmax)
    update_plot()

slider_step.on_changed(on_step_change)
slider_traj.on_changed(on_traj_change)

update_plot()
plt.show()
