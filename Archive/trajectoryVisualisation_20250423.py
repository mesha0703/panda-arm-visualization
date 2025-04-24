import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

# === Franka Emika Panda DH parameters ===
DH_PARAMS = [
    [0,        0,       0.0,  0],
    [0,    -np.pi/2,    0.333,      0],
    [0,     np.pi/2,    0.0,  0],
    [0.0825, np.pi/2,   0.316,      0],
    [-0.0825, -np.pi/2,  0.0,  0],
    [0.0,      np.pi/2,   0.384,      0],
    [0.088,  np.pi/2,   0.0,  0]
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
    orientations = [T[:3, :3]]
    for i, (a, alpha, d, _) in enumerate(DH_PARAMS):
        theta = joint_angles[i]
        T = T @ dh_transform(a, alpha, d, theta)
        positions.append(T[:3, 3])
        orientations.append(T[:3, :3])
    return np.array(positions), orientations


def load_trajectories(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        raw = f.read()

    raw_chunks = [chunk for chunk in raw.strip().split('\n\n') if chunk.strip()]
    print(f"Loaded {len(raw_chunks) - 1} trajectory blocks")

    # First block contains column names
    column_names = raw_chunks[0].strip().split(';')
    raw_trajectories = raw_chunks[1:]  # Skip header block

    trajectories = []

    err_num = 0

    for raw_traj in raw_trajectories:
        traj = []
        for line in raw_traj.strip().split('\n'):
            raw_data = line.strip().split(';')

            if len(raw_data) < 11:
                continue  # skip incomplete lines

            try:
                timestamp = float(raw_data[0])
                joint_angles = np.array([float(x) for x in raw_data[1:8]])
                eef_pose = np.array([float(x) for x in raw_data[8:13]])
            except ValueError:
                err_num += 1
                continue  # skip invalid numeric values

            frame = {
                'timestamp': timestamp,
                'joint_angles': joint_angles,
                'eef_pose': eef_pose
            }

            traj.append(frame)

        trajectories.append(traj)

    print(f"Loading trajectories finished. Number of errors: {err_num}")
    return trajectories

     

# === Load Data ===
csv_path = "gelenkwinkel_mehrere.csv"  # adjust path as needed
trajectories = load_trajectories(csv_path)

# === Initial Trajectory ===
current_traj = trajectories[0]
current_df = current_traj[0]
joint_angles = current_df['joint_angles']
eef_pose = current_df['eef_pose']

# === Plotting ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
line_fk, = ax.plot([], [], [], 'o-', lw=2, color='blue', label="FK Arm")
line_xyz, = ax.plot([], [], [], '-', lw=1.5, color='orange', label="Recorded x,y,z")
joint_angle_labels = []

ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([0.0, 1.5])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Franka FK Trajectory Visualiser")
ax.legend()

# === Display Joint Angles and XYZ ===
joint_text = fig.text(0.02, 0.75, '', fontsize=10, family='monospace')
xyz_text = fig.text(0.02, 0.65, '', fontsize=10, family='monospace')

# === Sliders ===
ax_slider_step = plt.axes([0.2, 0.04, 0.65, 0.03])
slider_step = Slider(ax_slider_step, 'Step', 0, len(current_traj)-1, valinit=0, valstep=1)

ax_slider_traj = plt.axes([0.2, 0.00, 0.65, 0.03])
slider_traj = Slider(ax_slider_traj, 'Trajectory', 0, len(trajectories)-1, valinit=0, valstep=1)

def update_plot():
    global current_traj, joint_angle_labels

    idx = int(slider_step.val)
    current_df = current_traj[idx]
    joint_angles = current_df['joint_angles'].copy()
    #joint_angles[[1, 2]] = joint_angles[[2, 1]] # Swap joint 2 and 3, because coppeliasim has another joint order
    #joint_angles[[3, 4]] = joint_angles[[4, 3]]
    #joint_angles[[5, 6]] = joint_angles[[6, 5]]
    eef_pose = current_df['eef_pose'][:3]

    fk_points, orientations = forward_kinematics_franka(joint_angles)

    # Remove previous angle labels and quivers
    for label in joint_angle_labels:
        label.remove()
    joint_angle_labels.clear()
    while hasattr(ax, '_quiver_artists') and ax._quiver_artists:
        q = ax._quiver_artists.pop()
        q.remove()

    # Build all past EEF positions as trajectory
    all_xyz = np.empty((0, 3))
    for i in range(idx + 1):
        point = current_traj[i]['eef_pose'][:3]
        all_xyz = np.vstack([all_xyz, point])

    # Update lines
    line_fk.set_data(fk_points[:, 0], fk_points[:, 1])
    line_fk.set_3d_properties(fk_points[:, 2])
    line_xyz.set_data(all_xyz[:, 0], all_xyz[:, 1])
    line_xyz.set_3d_properties(all_xyz[:, 2])

    # === Add angle labels and z-axis at each joint ===
    for i, (x, y, z) in enumerate(fk_points):
        if i == 0:
            continue  # skip base
        # Use the joint's z-axis for a small label offset (avoids overlap)
        axis = orientations[i][:, 2]
        offset = 0.06 * axis
        label = ax.text(
            x + offset[0], y + offset[1], z + offset[2],
            f"{joint_angles[i-1]:.2f} rad",
            color='black', fontsize=8, ha='center'
        )
        joint_angle_labels.append(label)
        q = ax.quiver(x, y, z, axis[0], axis[1], axis[2], length=0.05, color='red')
        if not hasattr(ax, '_quiver_artists'):
            ax._quiver_artists = []
        ax._quiver_artists.append(q)

    # === Update text boxes ===
    joint_display = '\n'.join([f'joint_{i+1}: {angle:.3f} rad' for i, angle in enumerate(joint_angles)])
    joint_text.set_text(f"Joint Angles (rad):\n{joint_display}")

    xyz_display = f"x: {eef_pose[0]:.4f}\ny: {eef_pose[1]:.4f}\nz: {eef_pose[2]:.4f}"
    xyz_text.set_text(f"Recorded EEF Position:\n{xyz_display}")

    fig.canvas.draw_idle()


def on_step_change(val):
    update_plot()

def on_traj_change(val):
    global current_traj
    
    current_traj = trajectories[int(val)]
    slider_step.valmax = len(current_traj) - 1
    slider_step.set_val(0)
    slider_step.ax.set_xlim(slider_step.valmin, slider_step.valmax)
    update_plot()

slider_step.on_changed(on_step_change)
slider_traj.on_changed(on_traj_change)

update_plot()
plt.show()
