import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos

# Franka Emika Panda DH parameters
DH_PARAMS = [
    [0,        np.pi/2,         0.0,   0],
    [0,   -np.pi/2,      0.333, 0],
    [0,    -np.pi/2,      0.0,   0],
    [0.0825, np.pi/2,    0.316, 0],
    [-0.0825, -np.pi/2,  0.0,   0],
    [0.0,    np.pi/2,    0.384, 0],
    [0.088,  np.pi/2,    0.0,   0]
]

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
        print(a, alpha, d, theta)
        T = T @ dh_transform(a, alpha, d, theta)
        positions.append(T[:3, 3])
        i += 1
        print()
        
    return np.array(positions)

def plot_arm(joint_angles):
    """Plot Franka Emika Panda in 3D given joint angles."""
    positions = forward_kinematics(joint_angles)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'o-', lw=3)
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0.0, 1.0])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Franka Emika Panda (input joint angles)')
    plt.show()

# Example usage: enter your 7 joint angles (in radians) here
example_angles = [0,0,1.0,0,0,0,0]
plot_arm(example_angles)
