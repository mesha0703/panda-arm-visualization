import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from numpy import sin, cos

# Corrected Franka Emika Panda DH parameters (modified DH convention)
# a, alpha, d, theta
DH_PARAMS = [
    [0,        0,          0.333,  0],
    [0,        -np.pi/2,   0,      0],
    [0,        np.pi/2,    0.316,  0],
    [0.0825,   np.pi/2,    0,      0],
    [-0.0825,  -np.pi/2,   0.384,  0],
    [0.0,      np.pi/2,    0,      0],
    [0.088,    np.pi/2,    0.107,  0]
]

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

def dh_transform(a, alpha, d, theta):
    return np.array([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles):
    """Computes the (x, y, z) positions of each joint given joint angles."""
    T = np.eye(4)
    positions = [T[:3, 3]]  # Include base position
    i = 0
    
    for (a, alpha, d, _) in DH_PARAMS:
        theta = joint_angles[i]
        T = T @ dh_transform(a, alpha, d, theta)
        positions.append(T[:3, 3])
        i += 1
        
    return np.array(positions)
def update(val):
    traj_idx = int(traj_slider.val)
    frame_idx = int(frame_slider.val)
    
    # Update frame slider max value based on selected trajectory
    current_traj = trajectories[traj_idx]
    frame_slider.valmax = len(current_traj) - 1
    frame_slider.ax.set_xlim(frame_slider.valmin, frame_slider.valmax)
    
    # Get current joint angles and EEF positions
    joint_angles = current_traj[frame_idx]['joint_angles']
    
    # Update arm plot
    positions = forward_kinematics(joint_angles)
    arm_line.set_data(positions[:, 0], positions[:, 1])
    arm_line.set_3d_properties(positions[:, 2])
    
    # Update EEF trajectory plot
    eef_positions = np.array([frame['eef_pose'][:3] for frame in current_traj[:frame_idx+1]])
    if len(eef_positions) > 0:
        eef_line.set_data(eef_positions[:, 0], eef_positions[:, 1])
        eef_line.set_3d_properties(eef_positions[:, 2])
    else:
        eef_line.set_data([], [])
        eef_line.set_3d_properties([])
    
    fig.canvas.draw_idle()

# Load trajectories
filepath = "gelenkwinkel_mehrere.csv"
trajectories = load_trajectories(filepath)

# Create initial plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
initial_joints = trajectories[0][0]['joint_angles']
initial_positions = forward_kinematics(initial_joints)

# Plot arm and EEF trajectory
arm_line, = ax.plot(initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2], 
                   'o-', lw=3, color='blue', label='Arm')
eef_line, = ax.plot([], [], [], 'r--', lw=1, alpha=0.7, label='EEF Path')

ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([0.0, 1.0])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Franka Emika Panda Arm Visualization with EEF Trajectory')
ax.legend()

# Adjust the main plot to make room for the sliders
plt.subplots_adjust(bottom=0.25)

# Create trajectory slider
ax_traj = plt.axes([0.25, 0.15, 0.65, 0.03])
traj_slider = Slider(
    ax=ax_traj,
    label='Trajectory',
    valmin=0,
    valmax=len(trajectories) - 1,
    valinit=0,
    valstep=1
)

# Create frame slider
ax_frame = plt.axes([0.25, 0.1, 0.65, 0.03])
frame_slider = Slider(
    ax=ax_frame,
    label='Frame',
    valmin=0,
    valmax=len(trajectories[0]) - 1,
    valinit=0,
    valstep=1
)

# Register update function with sliders
traj_slider.on_changed(update)
frame_slider.on_changed(update)

plt.show()
