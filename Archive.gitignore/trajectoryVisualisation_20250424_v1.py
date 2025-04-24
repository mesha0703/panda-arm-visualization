import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from numpy import sin, cos

# Corrected Franka Emika Panda DH parameters (modified DH convention)
DH_PARAMS = [
    [0,        0,          0.333, 0],  # Joint1: a=0, alpha=0, d=0.333, theta=q1
    [0,        -np.pi/2,   0,      0],  # Joint2: a=0, alpha=-pi/2, d=0, theta=q2
    [0,        np.pi/2,    0.316,  0],  # Joint3: a=0, alpha=pi/2, d=0.316, theta=q3
    [0.0825,   np.pi/2,    0,      0],  # Joint4: a=0.0825, alpha=pi/2, d=0, theta=q4
    [-0.0825,  -np.pi/2,   0.384,  0],  # Joint5: a=-0.0825, alpha=-pi/2, d=0.384, theta=q5
    [0.0,      np.pi/2,    0,      0],  # Joint6: a=0, alpha=pi/2, d=0, theta=q6
    [0.088,    np.pi/2,    0.107,  0]   # Joint7: a=0.088, alpha=pi/2, d=0.107, theta=q7
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
    frame_slider.valmax = len(trajectories[traj_idx]) - 1
    frame_slider.ax.set_xlim(frame_slider.valmin, frame_slider.valmax)
    
    # Get current joint angles
    current_frame = trajectories[traj_idx][frame_idx]
    joint_angles = current_frame['joint_angles']
    
    # Update plot
    positions = forward_kinematics(joint_angles)
    line.set_data(positions[:, 0], positions[:, 1])
    line.set_3d_properties(positions[:, 2])
    fig.canvas.draw_idle()

# Load trajectories
filepath = "gelenkwinkel_mehrere.csv"
trajectories = load_trajectories(filepath)

# Create initial plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
initial_joints = trajectories[0][0]['joint_angles']
positions = forward_kinematics(initial_joints)
line, = ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'o-', lw=3)
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([0.0, 1.0])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Franka Emika Panda Arm Visualization')

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
