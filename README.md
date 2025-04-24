# Franka Emika Panda Arm Visualization

3D visualization of the Franka Emika Panda robot arm with trajectory playback using Denavit-Hartenberg (DH) parameters.

![Franka Visualization Example](./images/example_image.png)

## Features

- Interactive slider controls for trajectory navigation
- End-effector path visualization
- Forward kinematics implementation

## Requirements

- Python 3.8+
- NumPy
- Matplotlib

## Installation

1. Clone this repository.
2. Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```

## Usage

```bash
python franka_visualization.py
```
- Use the sliders to control trajectory playback.

## Data Format

Trajectories are read from `trajectories.csv` with the following structure:

| Timestamp | Joint 1 | Joint 2 | ... | Joint 7 | X | Y | Z | roll | pitch | yaw |
|-----------|---------|---------|-----|---------|---|---|---|------|-------|-----|

- **Joint angles:** In radians (columns 2-8).
- **End-effector position:** X, Y, Z coordinates (columns 9-11).
- **End-effector orientation:** Roll, pitch, yaw (columns 12-14, not used in simulation).

### File Structure

- First row contains column headers.
- Each trajectory is a set of rows; different trajectories are separated by empty rows.

**Example:**

```csv
Timestamp;Joint 1;Joint 2;Joint 3;Joint 4;Joint 5;Joint 6;Joint 7;X;Y;Z;roll;pitch;yaw
<empty row here to separate trajectories>
0.0;0.1;0.2;0.3;0.4;0.5;0.6;0.7;0.5;-0.2;0.3;0.1;0.0;0.0
0.0;0.11;0.23;0.34;0.46;0.58;0.61;0.72;0.54;-0.26;0.38;0.1;0.0;0.0
<empty row here to separate trajectories>
0.0;0.1;0.2;0.3;0.4;0.5;0.6;0.7;0.5;-0.2;0.3;0.1;0.0;0.0
0.0;0.11;0.23;0.34;0.46;0.58;0.61;0.72;0.54;-0.26;0.38;0.1;0.0;0.0
