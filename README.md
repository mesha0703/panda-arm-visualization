# Franka Emika Panda Arm Visualization

3D visualization of Franka Emika Panda robot arm with trajectory playback using DH parameters.

## Features
- Interactive slider controls for trajectory navigation
- End-effector path visualization
- Forward kinematics implementation

## Requirements
- Python 3.8+
- NumPy
- Matplotlib

## Usage
1. Install requirements: 'pip install -r requirements.txt'
2. Run: 'python franka_visualization.py'
3. Use sliders to control trajectory playback

## Required data format
The trajectories are read from a .csv-file ('trajectories.csv').
First row are the row names:
Timestamp | Joint 1 | Joint 2 | ... | Joint 7 | X* | Y* | Z* | roll* | pitch* | jaw*

All joint angles are in radians.
* of end-effector.
Roll, pitch and jaw are not used in the simulation.