# 3-DOF-Drawing-Robot-Simulation-in-Unity

This project simulates a 3-DOF planar manipulator that draws letters in 3D space using Unity. The system uses forward and inverse kinematics to control the robot's motion, along with a trajectory planner to convert uppercase alphabet characters into movement paths. The simulation is visualized in real-time in Unity, providing an interactive way to understand the manipulator's behavior.



https://github.com/user-attachments/assets/536df1b7-ed42-4e5e-a718-1f2538f02e3b



This project is part of the FRA333 Robot Kinematics course at the Institute of Field Robotics, King Mongkut’s University of Technology Thonburi.

---

## Collabolator
- Supassakorn Wora-Urai (66340500056) 
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
   - [Prerequisites](#prerequisites)
   - [Setup Instructions](#setup-instructions)
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

### Overview of 3-DOF Manipulators
A 3-DOF planar manipulator typically uses rotational joints to perform tasks in a two-dimensional plane. In this configuration, the manipulator's movement is determined by three rotational degrees of freedom (DOF), which control the position and orientation of the end effector. This section explores the geometric and mathematical background necessary to understand the robot's kinematics.

A common 3-DOF manipulator is the **R-R-P (Revolute-Revolute-Prismatic) chain**, where two joints are rotational (revolute) and one is translational (prismatic). The configuration is used for tasks such as drawing, where precision and control over the position and orientation of the end effector are required.

### Kinematics Modeling: DH Parameters
The Denavit-Hartenberg (DH) parameterization is widely used for modeling the kinematics of robotic manipulators. It allows for a systematic way to derive transformation matrices that describe the relationship between the coordinate frames of each link in the manipulator.

Each link of a manipulator is associated with four DH parameters:
- $$\( \theta_i \)$$: Joint angle (for revolute joints).
- $$\( d_i \)$$: Link offset (for prismatic joints).
- $$\( a_i \)$$: Link length.
- $$\( \alpha_i \)$$: Link twist.

The transformation matrix from the \(i-1\)-th to the \(i\)-th link is given by:

$$
T_i^{i-1} = 
\begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i)\cos(\alpha_i) & \sin(\theta_i)\sin(\alpha_i) & a_i\cos(\theta_i) \\
\sin(\theta_i) & \cos(\theta_i)\cos(\alpha_i) & -\cos(\theta_i)\sin(\alpha_i) & a_i\sin(\theta_i) \\
0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Where:
- $$\( T_i^{i-1} \)$$ is the transformation matrix from frame $$\( i-1 \)$$ to frame $$\( i \)$$.
- The parameters $$\( \theta_i \), \( d_i \), \( a_i \), and \( \alpha_i \)$$ are specific to each link and joint type.

Using the DH parameters, we can calculate the overall transformation matrix from the base to the end effector by multiplying the transformation matrices for each link:

$$
T = T_1^0 \cdot T_2^1 \cdot T_3^2 \cdot \dots \cdot T_n^{n-1}
$$

This transformation matrix describes the position and orientation of the end effector relative to the base frame.

### Forward and Inverse Kinematics
- **Forward Kinematics**: Given the joint parameters $$\( \theta_1, \theta_2, \dots, \theta_n \)$$ (or \$$( d_i \)$$ for prismatic joints), forward kinematics calculates the position of the end effector in space. The end-effector position is obtained by applying the cumulative transformation from the base to the end effector using the DH parameter model.

$$
\text{End Effector Position} = \begin{bmatrix} x \\ y \\ z \end{bmatrix} = T_1^0 \cdot T_2^1 \cdot T_3^2 \cdot \dots \cdot T_n^{n-1} \cdot \begin{bmatrix} x_0 \\ y_0 \\ z_0 \end{bmatrix}
$$

Where 

$$
\begin{bmatrix}
x_0 \\
y_0 \\
z_0
\end{bmatrix}
$$


 is the position of the end effector in its local frame.

- **Inverse Kinematics**: Given the desired position of the end effector $$\( (x, y, z) \)$$, inverse kinematics calculates the joint parameters $$\( \theta_1, \theta_2, \dots, \theta_n \)$$ (or $$\( d_i \)$$ for prismatic joints) that achieve this position. For a 3-DOF manipulator, inverse kinematics can have multiple solutions, especially for revolute joints (elbow-up vs. elbow-down configurations). 

For a planar manipulator with two revolute joints (R-R) and one prismatic joint (P), the inverse kinematics equations can be derived using geometric methods or numerical solvers. A common method for solving inverse kinematics for the R-R chain involves using the law of cosines and the sine rule.

$$
\theta_2 = \cos^{-1}\left( \frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2} \right)
$$

$$
\theta_1 = \tan^{-1}\left( \frac{y}{x} \right) - \tan^{-1}\left( \frac{L_2 \sin(\theta_2)}{L_1 + L_2 \cos(\theta_2)} \right)
$$

Where $$\( L_1 \)$$ and $$\( L_2 \)$$ are the lengths of the links, and $$\( (x, y) \)$$ is the position of the end effector.

### Differential Kinematics & Jacobian Matrix
The Jacobian matrix $$\( J \)$$ relates the joint velocities $$\( \dot{\theta_1}, \dot{\theta_2}, \dots, \dot{\theta_n} \)$$ (for revolute joints) and $$\( \dot{d_3} \)$$ (for prismatic joints) to the end-effector velocities $$\( \dot{x}, \dot{y}, \dot{z} \)$$ in Cartesian space:


$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{z}
\end{bmatrix}
\text{ = }
J \cdot
\begin{bmatrix}
\dot{\theta_1} \\
\dot{\theta_2} \\
\dot{d_3}
\end{bmatrix}
$$


Where the Jacobian matrix is derived by differentiating the forward kinematics equations with respect to time. The Jacobian matrix is crucial for controlling the velocity of the end effector in task space and is used to compute the required joint velocities for a desired end-effector velocity.

For a 3-DOF manipulator, the Jacobian can be computed based on the specific geometry of the robot:

$$
J = \begin{bmatrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} & \frac{\partial x}{\partial d_3} \\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2} & \frac{\partial y}{\partial d_3} \\
\frac{\partial z}{\partial \theta_1} & \frac{\partial z}{\partial \theta_2} & \frac{\partial z}{\partial d_3}
\end{bmatrix}
$$

### Trajectory Planning and Time Scaling
Trajectory planning involves generating smooth paths for the robot’s end effector. The path is typically represented by a sequence of points in Cartesian space, and time scaling ensures that the robot moves along this path at a constant velocity or with a predefined speed profile.

A common trajectory profile is the **cubic spline**, which is used to interpolate between waypoints. The cubic spline ensures that the velocity and acceleration are continuous along the path:

$$
q(t) = a_0 + a_1t + a_2t^2 + a_3t^3
$$

Where $$\(q(t)\)$$ is the position of the end effector at time $$\(t\)$$, and the coefficients $$\(a_0, a_1, a_2, a_3\)$$ are determined based on boundary conditions (such as initial and final positions, velocities, and accelerations).

### Continuous Motion Selector and Model Alignment
When solving inverse kinematics for manipulator configurations with multiple possible solutions (e.g., elbow-up and elbow-down), it is crucial to select the solution that ensures smooth and continuous motion. The **Continuous Motion Selector** algorithm ensures that the robot transitions between kinematic solutions smoothly, minimizing abrupt changes in joint angles that would otherwise cause jerky motion.

Additionally, alignment issues between the kinematic model and the Unity coordinate system can arise. These issues are handled by applying an **Angular Offset** to correct for any misalignments, ensuring that the robot’s motion in Unity matches the desired kinematic model.

---


## System Overview
This section provides an overview of the 3-DOF Drawing Robot System, including its key components and how they interact to achieve the goal of drawing predefined shapes. Below is a diagram that illustrates the structure and workflow of the system.

### System Diagram
<img width="1892" height="607" alt="image" src="https://github.com/user-attachments/assets/9ee08021-51c7-4a36-872a-c1318d871712" />


### Key Components 

**Input Section:**
- **Link Parameters (L1, L2, L3):** Define the lengths of the robot’s links. These are constants that determine the manipulator's physical size.
- **Target Alphabet (Character):** The user inputs an uppercase letter, which is processed by the trajectory planner to generate the drawing path.

**Toolbox:**
- **Forward Kinematics (DH Parameters):** Calculates the end-effector position based on joint angles using Denavit-Hartenberg parameters.
- **Trajectory Planner (Path Generation):** Converts the target alphabet and drawing parameters into a set of target coordinates `[x(t), y(t), z(t)]` that define the drawing path.
- **Inverse Kinematics Solver:** Solves for joint angles (`θ1`, `θ2`) and linear distance (`d3`) from the target coordinates to move the robot to the desired position.
- **Continuous Motion Selector:** Ensures smooth and continuous movement by selecting the best inverse kinematic solution, avoiding abrupt changes in joint angles.

**Output Section:**
- **Joint Parameters (θ1, θ2, d3) & Joint Velocities (θ1̇, θ2̇, d3̇):** Control the robot’s movement by defining joint positions and velocities.
- **Visualization & Simulation (Unity):** Sends joint parameters and velocities to Unity for 3D visualization of the robot’s motion as it draws the letter, allowing real-time tracking of movements and speed.

---

## Installation

You can run the 3-DOF Drawing Robot Simulation without opening the Unity project by downloading the exported game build from Google Drive.

### 1. Download the Simulation
Click the link below to download the Unity build:

🔗 **[Download Simulation (Google Drive)](https://drive.google.com/drive/folders/19nDmi6M2uW3-X1BZ3VPevm-cb-oeUpgr?usp=sharing)**

### 2. Extract the Files
After downloading:
1. Right-click the ZIP file  
2. Select **Extract All**  
3. Extract the folder named **Kinematics**

### 3. Run the Simulation
Open the extracted **Kinematics** folder and run the executable:

- **Windows:**  
  `Scara.exe`

## How to Use

- Enter the desired uppercase alphabet in the input field.
- Click Start to begin the drawing motion.
- Watch the 3-DOF robot draw the selected character in real-time.
- Toggle the view if you want to observe the robot from different angles.

## Algorithms and Methods

## Testing and Results

## Future Work  

## References

- SPONG, M. W., HUTCHINSON, S., & VIDYASAGAR, M. ROBOT MODELING AND CONTROL. WILEY, 2020.
- MURRAY, R. M., LI, Z., & SASTRY, S. S. A MATHEMATICAL INTRODUCTION TO ROBOTIC MANIPULATION. CRC PRESS, 1994.
- CRAIG, J. J. INTRODUCTION TO ROBOTICS: MECHANICS AND CONTROL, 4TH EDITION. PEARSON EDUCATION, 2017.
- PAUL, R. P. ROBOT MANIPULATORS: MATHEMATICS, PROGRAMMING, AND CONTROL. MIT PRESS, 1981.
