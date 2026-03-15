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

## Development Conventions

- **Units:**
    - **Angles:** Radians (Internal logic) / Degrees (UI/Display).
    - **Distances:** Metres.
- **Coordinate Frames:** Uses standard DH parameter conventions. The robot model assumes a passive joint at the wrist to simplify the 4-bar linkage dynamics.
- **Testing:** New kinematic features or robot variants should be accompanied by tests in `dobot_tests.m`.
- **UI:** Prefer updating `ui_robot_v2.m` for UI changes as it is more modular and readable than the original `ui_robot.m`.
