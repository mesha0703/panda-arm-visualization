import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
from numpy import sin, cos

# Franka Emika Panda DH parameters (modified DH convention)
DH_PARAMS = [
    [0,        0,          0.333, 0],  # Joint1
    [0,        -np.pi/2,   0,      0],  # Joint2
    [0,        np.pi/2,    0.316,  0],  # Joint3
    [0.0825,   np.pi/2,    0,      0],  # Joint4
    [-0.0825,  -np.pi/2,   0.384,  0],  # Joint5
    [0.0,      np.pi/2,    0,      0],  # Joint6
    [0.088,    np.pi/2,    0.107,  0]   # Joint7
]

def dh_transform(a, alpha, d, theta):
    return np.array([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles):
    """Computes joint positions for visualization"""
    T = np.eye(4)
    positions = [T[:3, 3]]
    
    for i in range(7):
        a, alpha, d, _ = DH_PARAMS[i]
        theta = joint_angles[i]
        T = T @ dh_transform(a, alpha, d, theta)
        positions.append(T[:3, 3])
        
    return np.array(positions)

# Load and parse CSV data
content = """gelenkwinkel_mehrere.csv"""
trajectories = []
current_traj = []
header = None

for line in content.split('\n'):
    line = line.strip()
    if not line:
        if current_traj:
            trajectories.append(pd.DataFrame(current_traj, columns=header))
            current_traj = []
        continue
    
    if not header:
        header = line.split(';')
    else:
        current_traj.append([float(x) for x in line.split(';')])

if current_traj:
    trajectories.append(pd.DataFrame(current_traj, columns=header))

# Create visualization
fig = plt.figure(figsize=(14, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.25, bottom=0.25)

# Create sliders
ax_traj = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_step = plt.axes([0.25, 0.05, 0.65, 0.03])
traj_slider = Slider(ax_traj, 'Trajectory', 0, len(trajectories)-1, valinit=0, valstep=1)
step_slider = Slider(ax_step, 'Step', 0, 100, valinit=0, valstep=1)

def update(val):
    ax.clear()
    traj_idx = int(traj_slider.val)
    traj = trajectories[traj_idx]
    
    step_idx = min(int(step_slider.val), len(traj)-1)
    step_slider.valmax = len(traj)-1
    
    # Get joint angles and EEF position
    joint_angles = traj[[f'joint_{i+1}' for i in range(7)]].iloc[step_idx].values
    eef_pos = traj[['x', 'y', 'z']].values
    
    # Calculate joint positions
    joint_positions = forward_kinematics(joint_angles)
    
    # Plot robot arm
    ax.plot(joint_positions[:,0], joint_positions[:,1], joint_positions[:,2], 
            'o-', color='#4682b4', markersize=8, linewidth=3)
    
    # Plot full trajectory
    ax.plot(eef_pos[:,0], eef_pos[:,1], eef_pos[:,2], 
            'gray', alpha=0.3, linewidth=1)
    
    # Plot trajectory progress
    ax.plot(eef_pos[:step_idx+1,0], eef_pos[:step_idx+1,1], eef_pos[:step_idx+1,2], 
            'b', linewidth=2)
    
    # Plot current position
    ax.scatter(eef_pos[step_idx,0], eef_pos[step_idx,1], eef_pos[step_idx,2],
               c='r', s=100, edgecolor='k', zorder=10)
    
    # Formatting
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1.5])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Trajectory {traj_idx+1} | Step {step_idx+1}/{len(traj)}')
    plt.draw()

traj_slider.on_changed(update)
step_slider.on_changed(update)
update(None)
plt.show()
