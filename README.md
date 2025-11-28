# 3-DOF-Drawing-Robot-Simulation-in-Unity

This project simulates a 3-DOF planar manipulator that draws letters in 3D space using Unity. The system uses forward and inverse kinematics to control the robot's motion, along with a trajectory planner to convert uppercase alphabet characters into movement paths. The simulation is visualized in real-time in Unity, providing an interactive way to understand the manipulator's behavior.

<Insert your YouTube video link here>

This project is part of the FRA333 Robot Kinematics course at the Institute of Field Robotics, King Mongkut’s University of Technology Thonburi.

## Table of Contents
1. [Introduction](#introduction)
   - [Project Objectives](#project-objectives)
   - [Scope of Work](#scope-of-work)
2. [Theory and Related Work](#theory-and-related-work)
   - [Overview of 3-DOF Manipulators](#overview-of-3-dof-manipulators)
   - [Kinematics Modeling: DH Parameters](#kinematics-modeling-dh-parameters)
   - [Forward and Inverse Kinematics](#forward-and-inverse-kinematics)
   - [Differential Kinematics & Jacobian Matrix](#differential-kinematics-&-jacobian-matrix)
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
A 3-DOF planar manipulator typically uses rotational joints to perform tasks in a two-dimensional plane. This section explores the geometrical and mathematical background necessary to understand the robot's kinematics.

### Kinematics Modeling: DH Parameters
The Denavit-Hartenberg (DH) parameters are used to derive the transformation matrices for the robot's links and joints. This model allows the manipulator's position and orientation to be calculated from the joint angles and link lengths.

### Forward and Inverse Kinematics
- **Forward Kinematics**: Given the joint parameters, this method calculates the end-effector's position in space.
- **Inverse Kinematics**: Given the desired position of the end-effector, inverse kinematics solves for the necessary joint angles.

### Differential Kinematics & Jacobian Matrix
The Jacobian matrix links joint velocities to end-effector velocities, providing a way to calculate the required joint speeds to follow a desired trajectory.

### Trajectory Planning and Time Scaling
This section explains how to plan smooth motion paths and scale the trajectory to ensure constant velocity and avoid jerk during motion.

### Continuous Motion Selector and Model Alignment
Incorporates a mechanism to ensure smooth motion transitions, particularly when multiple inverse kinematic solutions are possible. Additionally, it addresses alignment issues between the kinematic model and Unity’s coordinate system.

---

## System Overview

### System Diagram
<img width="1919" height="642" alt="image" src="https://github.com/user-attachments/assets/3b532c49-fa46-4e50-a9fd-5e041be1e645" />

### Key Components 

**Input Section:**
- **Link Parameters (L1, L2, L3):** Define the lengths of the robot’s links. These are constants that determine the manipulator's physical size.
- **Target Alphabet (Character):** The user inputs an uppercase letter, which is processed by the trajectory planner to generate the drawing path.
- **Drawing Parameters:** Include size, starting position, and drawing speed, allowing control over the letter's scale and robot speed.

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
### Prerequisites
- Unity 3D

### Setup Instructions

## How to Use

## Algorithms and Methods

## Testing and Results

## Future Work  

## References

- SPONG, M. W., HUTCHINSON, S., & VIDYASAGAR, M. ROBOT MODELING AND CONTROL. WILEY, 2020.
- MURRAY, R. M., LI, Z., & SASTRY, S. S. A MATHEMATICAL INTRODUCTION TO ROBOTIC MANIPULATION. CRC PRESS, 1994.
- CRAIG, J. J. INTRODUCTION TO ROBOTICS: MECHANICS AND CONTROL, 4TH EDITION. PEARSON EDUCATION, 2017.
- PAUL, R. P. ROBOT MANIPULATORS: MATHEMATICS, PROGRAMMING, AND CONTROL. MIT PRESS, 1981.

## Collabolator
- Sasish Kaewsing (Khan)
- Supassakorn Wora-Urai (Kevin) 
