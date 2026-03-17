# ABDN-Kinematics-Project - GEMINI Context

This project provides a MATLAB-based kinematics and dynamics model for the **Dobot Magician Lite** robot. It focuses on forward and inverse kinematics using Denavit-Hartenberg (DH) parameters, symbolic Jacobian analysis, and interactive 3D visualization.

## Project Overview

- **Core Technologies:** MATLAB (R2025b+), Symbolic Math Toolbox, MATLAB Unit Test Framework.
- **Robot Model:** The robot is modeled as an open chain (simplifying the 4-bar linkages) with a passive joint to maintain the end effector parallel to the ground.
- **Key Features:**
    - Forward Kinematics (FK) using DH parameters.
    - Inverse Kinematics (IK) with branch selection (elbow-up/elbow-down).
    - Jacobian analysis (Geometric and Analytical).
    - Singularity detection and analysis.
    - Interactive UI for visualization and testing.

## Building and Running

### Prerequisites
- MATLAB with the **Symbolic Math Toolbox** installed.

### Execution Commands
- **Launch Interactive UI:**
  ```matlab
  ui_robot_v2
  ```
- **Run Unit Tests:**
  ```matlab
  runtests('dobot_tests')
  ```
- **Exploration Script:**
### Terminal Testing
To verify syntax and runtime integrity of a script without launching the full GUI, use:
```bash
matlab -nodisplay -r "filename; exit"
```

## Key Files

- `dobot.m`: Defines the `dobot` class, encapsulating the robot's physical properties, DH transformations, FK/IK solvers, and Jacobian calculations.
- `ui_robot_v2.m`: A self-contained programmatic UI using `uifigure` and `uiaxes` for real-time interaction with the robot model.
- `dobot_tests.m`: `matlab.unittest.TestCase` class verifying the accuracy of the IK/FK solvers and singularity conditions.
- `dobot_play.m`: A sandbox script for running manual experiments and viewing symbolic/numerical outputs.

## Dobot Class Reference

The `dobot` class is the core of the project, providing the following properties and methods:

### Properties
- `Theta1`, `Theta2`, `Theta3`, `Theta4`: Joint angles in radians.
- `L2`, `L3`: Link lengths (default: 0.15m each).

### Key Methods
- `dobot(theta1, theta2, theta3, theta4)`: Constructor for initializing joint angles.
- `elbowUp()`: Returns `true` if the robot is in an elbow-up configuration ($\theta_3 < 0$).
- `setEndEffector(xyz, elbowUp)`: Inverse kinematics solver that sets joint angles for a target $(x, y, z)$ position and branch selection.
- `xyz()` / `o(n)`: Returns the Cartesian coordinates of the end effector or a specific joint.
- `transform(i, j)`: Calculates the homogeneous transformation matrix from frame $i$ to frame $j$.
- `jacobian()`: Computes the 6x4 Geometric Jacobian.
- `analyticalJacobian()`: Computes the 4x4 Analytical Jacobian for $[x, y, z, \psi]^T$.
- `singularityDet()`: Returns the determinant of the Analytical Jacobian for singularity analysis.
- `isSingular(tol)`: Checks if the current configuration is near a singularity.
- `singularityConditions(tol)`: Returns boolean flags for specific singularity conditions ($sin(\theta_3) \approx 0$ or reach limits).

### Kinematic Details
- Uses **Standard DH Parameters** via an internal `dh` helper function.
- Includes a **Passive Joint** (frame 'p') between joint 3 and 4 to maintain the end effector parallel to the ground, reflecting the physical robot's parallel linkage.

## Development Conventions

- **Units:**
    - **Angles:** Radians (Internal logic) / Degrees (UI/Display).
    - **Distances:** Metres.
- **Coordinate Frames:** Uses standard DH parameter conventions. The robot model assumes a passive joint at the wrist to simplify the 4-bar linkage dynamics.
- **Testing:** New kinematic features or robot variants should be accompanied by tests in `dobot_tests.m`.
- **UI:** Prefer updating `ui_robot_v2.m` for UI changes as it is more modular and readable than the original `ui_robot.m`.
