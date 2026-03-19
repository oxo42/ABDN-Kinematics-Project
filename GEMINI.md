# ABDN-Kinematics-Project - GEMINI Context

This project provides a MATLAB-based kinematics and dynamics model for the **Dobot Magician Lite** robot. It focuses on forward and inverse kinematics using Denavit-Hartenberg (DH) parameters, symbolic Jacobian analysis, and interactive 3D visualization.

## Project Overview

- **Core Technologies:** MATLAB (R2025b+), Symbolic Math Toolbox, MATLAB Unit Test Framework, LaTeX (for reporting).
- **Robot Model:** The robot is modeled as an open chain (simplifying the 4-bar linkages) with a passive joint to maintain the end effector parallel to the ground.
- **Key Features:**
    - Forward Kinematics (FK) using DH parameters.
    - Inverse Kinematics (IK) with branch selection (elbow-up/elbow-down).
    - Jacobian analysis (Geometric and Analytical).
    - Singularity detection and analysis (Determinant and specific conditions).
    - Interactive UI for visualization and testing.
    - Automated test logging and summary generation.

## Building and Running

### Prerequisites
- MATLAB with the **Symbolic Math Toolbox** installed.
- LaTeX distribution (MacTeX or TeX Live) for building the report.

### Execution Commands
- **Launch Interactive UI:**
  ```matlab
  ui_robot
  ```
- **Run Unit Tests:**
  ```matlab
  runtests('dobot_tests')
  ```
- **Run Test Summary & Log:**
  ```matlab
  dobot_test_summary
  ```
  This script runs a comprehensive battery of tests and saves the output to `test_log.txt`.

### Terminal Testing
To verify syntax and runtime integrity of a script without launching the full GUI, use:
```bash
matlab -nodisplay -r "filename; exit"
```

### Building the Report
The report source is located in the `report/` directory.
```bash
cd report
pdflatex report.tex && bibtex report && pdflatex report.tex && pdflatex report.tex
```

## Key Files

- `dobot.m`: Defines the `dobot` class, encapsulating physical properties, DH transformations, solvers, and Jacobian calculations.
- `ui_robot.m`: Programmatic UI using `uifigure` and `uiaxes` for real-time interaction.
- `dobot_tests.m`: Unit tests verifying IK/FK accuracy and singularity conditions.
- `dobot_test_summary.m`: Script to execute and log all major kinematic tests.
- `dobot_play.m`: Sandbox script for manual experiments.
- `report/report.tex`: LaTeX source for the project's technical report.
- `ui wireframe.svg`: UI design wireframe.
- `robot_plot.png`: Documentation image showing the robot model.

## Dobot Class Reference

### Properties
- `Theta1`, `Theta2`, `Theta3`, `Theta4`: Joint angles in radians.
- `L1`, `L2`, `L3`: Link lengths (Base: 0.107m, Links: 0.15m).
- `JointLimits`: 4x2 matrix defining the [min, max] range for each joint.

### Key Methods
- `dobot(theta1, theta2, theta3, theta4)`: Constructor.
- `elbowUp()`: Returns `true` if $\theta_3 < 0$.
- `validateJointLimits(thetas)`: Throws an error if angles are out of range.
- `setEndEffector(xyz, elbowUp)`: IK solver with branch selection.
- `xyz()` / `o(n)`: Returns Cartesian coordinates of the end effector or joint $n$.
- `transform(i, j)`: Returns the homogeneous transformation matrix from frame $i$ to $j$.
- `jacobian()`: Computes the 6x4 Geometric Jacobian.
- `analyticalJacobian()`: Computes the 4x4 Analytical Jacobian for $[x, y, z, \psi]^T$.
- `singularityDet()`: Returns the determinant of the Analytical Jacobian.
- `isSingular(tol)`: Checks if the configuration is near a singularity.
- `singularityConditions(tol)`: Returns boolean flags:
    - `cond1`: $sin(\theta_3) \approx 0$ (Elbow singularity).
    - `cond2`: Reach limit/alignment singularity.

### Kinematic Details
- Uses **Standard DH Parameters** via an internal `dh` helper function.
- **Passive Joint:** Frame 'p' is inserted between joint 3 and 4 to keep the end effector parallel to the ground, mimicking the physical robot's parallel linkage mechanism.

## Development Conventions

- **Units:** Radians for internal logic; Degrees for UI/Display. Distances in Metres.
- **Coordinate Frames:** Standard DH conventions.
- **Testing:** New features must be verified in `dobot_tests.m` and logged via `dobot_test_summary.m`.
- **UI Formatting:** Do not use `evalc(disp(...))` directly in UI update loops. Instead, use the `formattedDisplayText` helper function to sanitize and format matrix/object output for display.
