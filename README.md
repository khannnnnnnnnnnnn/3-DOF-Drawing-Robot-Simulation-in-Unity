# 3-DOF-Drawing-Robot-Simulation-in-Unity

This project simulates a 3-DOF planar manipulator that draws letters in 3D space using Unity. The system uses forward and inverse kinematics to control the robot's motion, along with a trajectory planner to convert uppercase alphabet characters into movement paths. The simulation is visualized in real-time in Unity, providing an interactive way to understand the manipulator's behavior.



https://github.com/user-attachments/assets/536df1b7-ed42-4e5e-a718-1f2538f02e3b



This project is part of the FRA333 Robot Kinematics course at the Institute of Field Robotics, King Mongkut’s University of Technology Thonburi.

---

## Collabolator
- Supasakorn Wora-Urai (66340500056) 
- Sasish Kaewsing (66340500076)

---

## Table of Contents
1. [Introduction](#introduction)
   - [Project Objectives](#project-objectives)
   - [Scope of Work](#scope-of-work)
2. [Theory and Related Work](#theory-and-related-work)
   - [Overview of 3-DOF Manipulators](#overview-of-3-dof-manipulators)
   - [Kinematics Modeling: DH Parameters](#kinematics-modeling-dh-parameters)
   - [Forward and Inverse Kinematics](#forward-and-inverse-kinematics)
   - [Differential Kinematics & Jacobian Matrix](#differential-kinematics-jacobian-matrix)
   - [Trajectory Planning and Time Scaling](#trajectory-planning-and-time-scaling)
   - [Continuous Motion Selector and Model Alignment](#continuous-motion-selector-and-model-alignment)
3. [System Overview](#system-overview)
   - [System Diagram](#system-diagram)
   - [Key Components](#key-components)
4. [Installation](#installation)
5. [How to Use](#how-to-use)
6. [Algorithms and Methods](#algorithms-and-methods)
7. [Testing and Results](#testing-and-results)
   - [Simulation Results and Visual Output](#simulation-results-and-visual-output)
   - [Performance Evaluation](#performance-evaluation)
8. [Future Work](#future-work)
   - [Potential Improvements](#potential-improvements)
   - [Extensions and New Features](#extensions-and-new-features)
9. [References](#references)

---

## Introduction

### Project Objectives
The goal of this project is to simulate the motion of a 3-DOF planar manipulator for drawing letters. This involves:
- Developing a kinematic model of a 3-DOF R-R-P manipulator.
- Implementing forward and inverse kinematics for controlling the robot's movements.
- Creating a trajectory planner to convert alphabetic input into movement paths.
- Simulating and visualizing the manipulator's motion in Unity.

### Scope of Work
- Focuses on a 3-DOF manipulator with an R-R-P chain configuration.
- Includes developing a trajectory planner that converts uppercase English letters into path coordinates.
- Uses Unity for visual simulation and real-time drawing of letters.

---

## Theory and Related Work

### SCARA Robot Architecture

SCARA (Selective Compliance Assembly Robot Arm) robots are widely used in industrial automation for tasks requiring high-speed planar motion with vertical compliance. The typical SCARA configuration consists of:

- **Two parallel rotational joints** (θ₁, θ₂) that control horizontal positioning
- **One vertical prismatic joint** (d₃ or z) that controls height
- **High rigidity in the vertical direction** while maintaining compliance in the horizontal plane

This configuration is ideal for pick-and-place operations, assembly tasks, and, as demonstrated in this project, automated drawing applications.

**Advantages of SCARA Configuration:**
- High speed and repeatability in the horizontal plane
- Simple kinematic structure allowing for closed-form inverse kinematics solutions
- Well-defined workspace (annular region)
- Minimal coupling between vertical and horizontal motion

### Kinematics Foundation

The robot's motion is described using coordinate transformations that relate the base frame to the end-effector frame. For our SCARA robot:

**Base Frame Alignment:**
- The robot base is rotated -90° around the X-axis in Unity's coordinate system
- This rotation maps:
  - Unity's X-axis → Robot's Local X (Right)
  - Unity's Y-axis → Robot's Local Y (Forward/Depth)
  - Unity's Z-axis → Robot's Local Z (Height/Up)

**Link Parameters:**
- L₁ = Length of first link (shoulder to elbow) = 230 units
- L₂ = Length of second link (elbow to end-effector) = 135 units
- Combined reach: L₁ + L₂ = 365 units
- Minimum reach: |L₁ - L₂| = 95 units

**Joint Variables:**
- θ₁: Shoulder rotation angle (around vertical axis)
- θ₂: Elbow rotation angle (around vertical axis)
- d₃ or h: Vertical displacement of prismatic joint

### Inverse Kinematics for R-R-P Configuration

The inverse kinematics problem for this SCARA robot can be decomposed into two independent sub-problems:

#### 1. Planar (Horizontal) Solution

Given a target position (xₜ, yₜ, zₜ) in world space, we first transform it to the robot's local coordinate frame. The coordinate transformation projects the target vector onto the robot's coordinate axes to obtain local coordinates (x, depth, height).

The planar distance from shoulder to target:

$$D = \sqrt{x^2 + \text{depth}^2}$$

**Elbow Angle Solution:**
Using the law of cosines for the triangle formed by L₁, L₂, and D:

$$\cos(\theta_2) = \frac{D^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

$$\theta_2 = \arccos\left(\text{clamp}\left(\frac{D^2 - L_1^2 - L_2^2}{2 L_1 L_2}, -1, 1\right)\right)$$

**Shoulder Angle Solution:**
The shoulder angle is computed using the target direction and the internal triangle geometry:

$$\beta = \arctan2(\text{depth}, x)$$

$$\phi = \arctan2(L_2 \sin(\theta_2), L_1 + L_2 \cos(\theta_2))$$

$$\theta_1 = \beta - \phi$$

Where:
- β is the angle from the shoulder to the target
- φ is the internal angle of the triangle at the shoulder joint

#### 2. Vertical (Prismatic) Solution

The prismatic joint displacement is simply:

$$d_3 = \text{clamp}(\text{height}, h_{\min}, h_{\max})$$

Where h_min = 0 and h_max = 50 units in this implementation.

**Workspace Constraints:**
- Maximum reach: D ≤ L₁ + L₂ - 0.001 (365 units)
- Minimum reach: D ≥ |L₁ - L₂| + 0.001 (95 units)
- Height range: 0 ≤ h ≤ 50 units

The implementation includes small epsilon values (0.001) to prevent numerical singularities at workspace boundaries.

### Workspace Analysis

The SCARA robot's workspace forms an annular (ring-shaped) region in the horizontal plane:

**Outer Boundary (Maximum Reach):**
$$r_{\text{outer}} = L_1 + L_2 = 365 \text{ units}$$

**Inner Boundary (Minimum Reach):**
$$r_{\text{inner}} = |L_1 - L_2| = 95 \text{ units}$$

The workspace visualization displays these boundaries as red circular rings, helping users understand the valid operating region. The workspace is centered at the shoulder pivot point, not the robot base, ensuring accurate representation of the reachable area.

**Vertical Workspace:**
- The prismatic joint provides vertical displacement from 0 to 50 units
- This creates a three-dimensional annular cylinder workspace

### Trajectory Planning

The trajectory planning system converts high-level drawing commands (letters) into low-level motion primitives (waypoints).

**Path Representation:**
Each letter is defined as a collection of strokes, where each stroke is a list of 3D waypoints. The path generator processes text input and returns a nested list structure representing all strokes needed to draw the complete text.

**Stroke Types:**

1. **Linear Strokes:** Defined by consecutive waypoints connected by straight lines
   - Example: Letter 'E' consists of 4 connected line segments
   - Used for straight edges and angular features

2. **Arc Strokes:** Generated using parametric circle equations
   - Center: (cₓ, cᵧ)
   - Radii: (rₓ, rᵧ) for elliptical arcs
   - Angular span: [θ_start, θ_end]
   - Resolution: 20 interpolated points per arc

**Arc Generation Formula:**

$$x(t) = c_x + r_x \cos\left(\theta_{\text{start}} + t(\theta_{\text{end}} - \theta_{\text{start}})\right)$$

$$z(t) = c_z + r_z \sin\left(\theta_{\text{start}} + t(\theta_{\text{end}} - \theta_{\text{start}})\right)$$

where t ∈ [0, 1] is the interpolation parameter.

**Coordinate Transformation Pipeline:**
The path generation system applies scaling and offset transformations to convert normalized letter coordinates (defined on a 4×5 grid) into world space coordinates. This ensures that letter definitions remain resolution-independent and can be easily scaled or repositioned.

---


## Installation

You can run the 3-DOF Drawing Robot Simulation without opening the Unity project by downloading the exported game build from Google Drive.

### 1. Download the Simulation
Click the link below to download the Unity build:

🔗 **[Download Simulation (Google Drive)](https://drive.google.com/drive/folders/19nDmi6M2uW3-X1BZ3VPevm-cb-oeUpgr?usp=sharing)**

<img width="1883" height="677" alt="image" src="https://github.com/user-attachments/assets/159d9817-6f15-4001-8fba-e991559aba77" />

### 2. Extract the Files
After downloading:
1. Right-click the ZIP file  
2. Select **Extract All**  
3. Extract the folder named **Kinematics**

### 3. Run the Simulation
Open the extracted **Kinematics** folder and run the executable:

- **Windows:**  
  `Scara.exe`
<img width="1882" height="652" alt="image" src="https://github.com/user-attachments/assets/7d17cfe0-a329-4924-b1d8-cea8b5748b15" />


## How to Use

- Enter the desired uppercase alphabet in the input field.
  
<img width="1615" height="907" alt="image" src="https://github.com/user-attachments/assets/77cafb69-0376-4337-9794-38b69e47cde6" />

- Click Start to begin the drawing motion.
  
<img width="1612" height="906" alt="image" src="https://github.com/user-attachments/assets/b26235a1-613e-4111-aaf8-08667a8102f7" />

- Watch the 3-DOF robot draw the selected character in real-time.
  
https://github.com/user-attachments/assets/8d55a4c1-0fec-4e27-85f7-b75ef5e82995

- Toggle the view if you want to observe the robot from different angles.
  
https://github.com/user-attachments/assets/a6331f78-85c7-4e6b-90cb-6fc346642f6c

---

## Algorithms and Methods

This section details the primary algorithms used to enable the 3-DOF R-R-P manipulator simulation in Unity, focusing on the inverse kinematics (IK) solution and the custom path generation pipeline.

### Inverse Kinematics (IK) Solver

The simulation uses a geometric approach to solve the Inverse Kinematics problem for the R-R chain, determining the required joint angles ($\theta_1$, $\theta_2$) for any given Cartesian target point $(x, y)$. The third DOF (the Prismatic joint, $d_3$) directly controls the $z$-coordinate (height) and is solved linearly.

The core steps are:

- Coordinate Mapping: The target point, $P_{target} = (x_t, y_t, z_t)$, is first transformed into the local coordinate frame of the robot's base. The height, $z_t$, is directly mapped to the prismatic joint extension $d_3$. The planar coordinates, $(x, y)$, are used for angular calculation.

- Elbow Angle ($\theta_2$): The angle of the second joint is calculated using the Law of Cosines, based on the triangle formed by the two links ($L_1$, $L_2$) and the distance to the target ($D = \sqrt{x^2 + y^2}$):

$$
\cos(\theta_2) = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

$$
\theta_2 = \arccos\left(\frac{D^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right)
$$

- Shoulder Angle ($\theta_1$): The angle of the first joint is calculated by finding the angle to the target and subtracting the internal angle formed by the elbow joint:

$$
\theta_1 = \text{atan2}(y, x) - \text{atan2}(L_2 \sin(\theta_2), L_1 + L_2 \cos(\theta_2))
$$

- Continuous Motion Selector: The $\theta_1$ and $\theta_2$ solutions are continuously applied in the Unity $\text{Update()}$ loop. The use of $\text{atan2}$ inherently provides robust quadrant handling, and the geometric method avoids singularities common in matrix-based solvers within the workspace.

### Manual Path Generator (Trajectory Planning)

The Trajectory Planner converts user-inputted text into a continuous sequence of 3D waypoints. To guarantee precise and reproducible motion for both straight and curved segments, a Manual Stroke Definition algorithm is employed:

- Stroke Definition: Each uppercase letter ($A$ through $Z$) is decomposed into a list of constituent strokes. These strokes are hard-coded as sequences of coordinate points $(x, z)$ on a normalized $4 \times 5$ grid.

- Curved Segment Generation (Arc Interpolation): For letters requiring smooth curves (e.g., $C, S, G, O$), the definition uses the custom function $\text{CreateArc}(c_x, c_z, r_x, r_z, \text{startAngle}, \text{endAngle})$. This function mathematically generates a dense set of intermediate points ($\approx 20$ points) between the start and end angles using $\cos$ and $\sin$, ensuring the robot follows a smooth, continuous curve rather than jagged line segments.

- Scaling and Offset: The generated normalized points are then scaled by the user-defined scale factor and translated by the current letter's $x$-offset before being converted to world coordinates via $\text{TransformPoint}$.

### Optimized Motion Control (Pen Up/Down)

A state machine embedded within the WriteRoutine coroutine manages the prismatic joint ($d_3$) to ensure efficient drawing:

- Continuity Check: Before starting a new stroke, the algorithm checks if the new starting point ($P_{start}$) is equal to the end point of the previous stroke ($P_{end}$). This is done using a positional tolerance ($\approx 0.01\text{m}$).

- Discontinuous Motion (Lift Required): If $P_{start} \neq P_{end}$, the robot performs a lift-and-move sequence:

   - Pen Up: Move $d_3$ to $\text{penUpHeight}$. ($\text{TrailRenderer}$ disabled).

   - Hover: Move end-effector (GhostTarget) to $P_{start}$.

   - Pen Down: Move $d_3$ to $\text{penDownHeight}$. ($\text{TrailRenderer}$ enabled).

- Continuous Motion (Draw Directly): If $P_{start} = P_{end}$, the motion controller skips the Pen Up/Hover/Pen Down sequence, allowing the robot to transition immediately into drawing the next segment, eliminating unnecessary vertical movements and saving execution time.

---

## Testing and Results

### Simulation Results and Visual Output

| Test Case | Input | Expected Output | Actual Visual Result | Observations | 
 | ----- | ----- | ----- | ----- | ----- | 
| **Reach Boundary** | Target at $L_1 + L_2 + 10$ | Clamp at Max Reach (365 units) | End effector stops at the red outer workspace ring. | IK successfully applies limits to prevent singularities and out-of-bounds errors. | 
| **Straight Line Accuracy** | Letter 'H' (Spine) | Perfect vertical/horizontal lines. | Straight segments show minimal deviation from ideal path. | Confirms accuracy of coordinate system alignment and IK solution for linear motions. | 
| **Curved Trajectory** | Letter 'S' (Compound Arc) | Smooth, continuous double-arc path. | The path is a fluid curve, confirming the success of the $\text{CreateArc}$ function in generating high-resolution, smooth waypoints. | The use of trigonometric interpolation provides superior path quality compared to simple linear segment approximation. | 
| **Motion Continuity** | Word "HI" | **H**: 3 lifts. **I**: 3 lifts. | Pen lifts between H-I strokes. Confirms the continuity selector is functional for separating letters. |  | 
| **Continuous Stroke** | Letter 'M' (Connected lines) | No pen lift between connected segments. | Pen remains down for the entire duration of the 'M' stroke (5 movements), demonstrating efficient motion control. |

### Performance Evaluation

The simulation uses Geometric Inverse Kinematics, which is computationally efficient. The system maintains a consistent frame rate, with minimal processing delay for the IK solution, resulting in a smooth and real-time visualization of the robot's motion. The primary resource usage is dominated by the rendering of the 3D models and the Trail Renderer (ink trail).

Analysis of Curved Trajectories: The successful drawing of letters like 'S' and 'G' without polygon artifacts demonstrates that the Manual Path Generator combined with $\text{CreateArc}$ is superior to texture-based pathfinding (e.g., Unity Colliders), which often simplify curves into low-vertex polygons.

---

## Future Work

- **Allow Custom Letter Size**:  
  Implement a feature that allows users to select the size of the letter to be drawn. This could involve adding a slider or input field to dynamically adjust the scaling of the drawn character.

- **Support for Lowercase Letters**:  
  Extend the functionality to allow the robot to draw lowercase letters. This would require adding the corresponding path and motion for each lowercase character, ensuring the robot handles them with the same precision as uppercase characters.

- **Speed Control for Drawing**:  
  Introduce an option to control the speed of the drawing process. Users could adjust the speed based on their preferences, allowing for both faster and slower motion for demonstration or testing purposes.

- **Improve Trajectory Planning**:  
  Enhance the trajectory planning algorithm to create smoother curves for more complex characters or patterns. This could be especially useful for drawing more intricate designs or letters with rounded edges.

- **Add Support for Drawing Shapes or Custom Text**:  
  Allow users to input custom shapes or text (beyond just alphabet letters) to be drawn by the robot. This could be achieved by enabling a free-text input or uploading an image that the robot can convert to a drawing path.

- **Real-Time Feedback on Drawing Process**:  
  Implement a feature that shows the robot's current progress during the drawing, such as a percentage or a visual progress bar, so users can track how much of the drawing has been completed.

- **Enhance User Interface**:  
  Improve the user interface by adding more intuitive controls, such as buttons to clear the drawing or reset the system to its starting state. Additional options could include changing colors or selecting different pen tools.

- **Save and Load Drawings**:  
  Allow users to save the drawn output to a file or load pre-existing drawings for the robot to replicate. This would enable users to save custom drawings and reload them in future sessions.

## References

- SPONG, M. W., HUTCHINSON, S., & VIDYASAGAR, M. ROBOT MODELING AND CONTROL. WILEY, 2020.
- MURRAY, R. M., LI, Z., & SASTRY, S. S. A MATHEMATICAL INTRODUCTION TO ROBOTIC MANIPULATION. CRC PRESS, 1994.
- CRAIG, J. J. INTRODUCTION TO ROBOTICS: MECHANICS AND CONTROL, 4TH EDITION. PEARSON EDUCATION, 2017.
- PAUL, R. P. ROBOT MANIPULATORS: MATHEMATICS, PROGRAMMING, AND CONTROL. MIT PRESS, 1981.
- BARRETO, C. A., & ALMEIDA, L. A. INVERSE KINEMATICS OF ROBOTIC MANIPULATORS: A REVIEW. JOURNAL OF THE BRAZILIAN SOCIETY OF MECHANICAL SCIENCES AND ENGINEERING, 2011.
- ALIMARDANI, M., & KERMANI, A. A REVIEW OF TRAJECTORY PLANNING ALGORITHMS FOR ROBOTIC MANIPULATORS. INTERNATIONAL JOURNAL OF ROBOTICS AND AUTOMATION, 34(2), 137-156, 2019.
- TAN, C., & WANG, H. REAL-TIME MOTION SIMULATION FOR 3D ROBOTIC MANIPULATION USING UNITY. PROCEEDINGS OF THE IEEE/RSJ INTERNATIONAL CONFERENCE ON INTELLIGENT ROBOTS AND SYSTEMS (IROS), 2015.
- BODAK, J., & ZIVOTOFSKY, A. Z. TRAJECTORY PLANNING AND PATH GENERATION FOR ROBOTIC DRAWING SYSTEMS. INTERNATIONAL JOURNAL OF ADVANCED ROBOTIC SYSTEMS, 13(1), 73-81, 2016.
