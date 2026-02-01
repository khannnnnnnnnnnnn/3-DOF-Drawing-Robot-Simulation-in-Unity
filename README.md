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

# Algorithms and Methods

This section details the primary algorithms used to enable the 3-DOF R-R-P manipulator simulation in Unity, focusing on the inverse kinematics solution and the custom path generation pipeline.

## Inverse Kinematics Solver

The IK solver in `ScaraController.cs` implements a geometric approach optimized for the R-R-P configuration. This solver runs every frame in the `Update()` loop when the robot is in AutomaticIK mode, continuously calculating the joint angles needed to reach the target position.

### Algorithm Steps:

**Step 1: Coordinate Frame Transformation**

The first challenge is that Unity's coordinate system doesn't align with the robot's local coordinate system. The robot base is rotated -90° around the X-axis, so we need to transform the target position from Unity's world space into the robot's local space.

```csharp
// Calculate shoulder pivot position in world space
Vector3 shoulderWorldPos = transform.position;
shoulderWorldPos.x = shoulderRotationBone.position.x;
shoulderWorldPos.z = shoulderRotationBone.position.z;

// Project target onto robot's coordinate axes
Vector3 diffWorld = targetPos - shoulderWorldPos;
float x = Vector3.Dot(diffWorld, transform.right);      // Local X
float depth = Vector3.Dot(diffWorld, transform.up);     // Local Y
float height = Vector3.Dot(diffWorld, transform.forward); // Local Z
```

**Explanation:**
- First, we calculate the shoulder pivot's world position. This is crucial because the robot's rotation happens around the shoulder, not the base.
- We use only the X and Z coordinates from the shoulder bone's position, keeping the base's Y coordinate. This ensures we're calculating from the correct pivot point in the horizontal plane.
- `diffWorld` is the vector from the shoulder pivot to the target position.
- We then use the dot product to project this vector onto the robot's three local axes:
  - `transform.right` gives us the local X component (left-right movement)
  - `transform.up` gives us the local Y component (forward-backward, called "depth")
  - `transform.forward` gives us the local Z component (height)
- This transformation is essential because the inverse kinematics equations work in the robot's local coordinate frame, not Unity's global frame.

---

**Step 2: Planar Distance Calculation**

Now that we have the target in local coordinates, we calculate the 2D distance in the horizontal plane (ignoring height). This distance will be used to solve for the arm's joint angles.

```csharp
float dist = Mathf.Sqrt(x*x + depth*depth);
float combinedLength = L1 + L2;

// Workspace clamping to prevent singularities
if (dist > combinedLength) dist = combinedLength - 0.001f;
if (dist < Mathf.Abs(L1 - L2)) dist = Mathf.Abs(L1 - L2) + 0.001f;
```

**Explanation:**
- `dist` is the straight-line distance from the shoulder to the target in the horizontal plane, calculated using the Pythagorean theorem.
- `combinedLength` is the robot's maximum reach (when the arm is fully extended).
- **Workspace Clamping:** The robot cannot reach targets outside its physical workspace. We clamp the distance to prevent two problems:
  - If the target is beyond maximum reach (dist > L1 + L2), we clamp it to just inside the boundary
  - If the target is inside the minimum reach (dist < |L1 - L2|), we clamp it to just outside this inner dead zone
- The small epsilon value (0.001) prevents division by zero and numerical instability at the exact workspace boundaries.
- This clamping ensures the IK solver always gets valid inputs and never produces undefined (NaN) angles.

---

**Step 3: Elbow Angle Solution (Law of Cosines)**

With the distance known, we can now solve for the elbow angle using the law of cosines. This is the angle between the two arm segments.

```csharp
float cosT2 = (dist*dist - L1*L1 - L2*L2) / (2 * L1 * L2);
float theta2 = Mathf.Acos(Mathf.Clamp(cosT2, -1f, 1f));
```

**Explanation:**
- This uses the law of cosines for triangles: c² = a² + b² - 2ab·cos(C)
- Rearranged to solve for the angle: cos(θ₂) = (dist² - L1² - L2²) / (2·L1·L2)
- The triangle is formed by:
  - Side a = L1 (shoulder to elbow)
  - Side b = L2 (elbow to end-effector)
  - Side c = dist (shoulder to target)
  - Angle C = θ₂ (elbow angle)
- `Mathf.Clamp(cosT2, -1f, 1f)` ensures the value stays within the valid range for arccosine [-1, 1]. Without this, floating-point errors could cause the acos function to return NaN.
- `theta2` is always positive in this implementation, representing the elbow bend angle.
- This geometric solution is much faster than iterative numerical methods and gives exact results in a single calculation.

---

**Step 4: Shoulder Angle Solution (Geometric Decomposition)**

The shoulder angle is more complex because it depends on both the direction to the target and the elbow configuration. We decompose this into two components.

```csharp
float baseAngle = Mathf.Atan2(depth, x);
float innerAngle = Mathf.Atan2(
    L2 * Mathf.Sin(theta2),
    L1 + L2 * Mathf.Cos(theta2)
);
float theta1 = baseAngle - innerAngle;
```

**Explanation:**
- **baseAngle (β):** This is the direction from the shoulder to the target in the horizontal plane, calculated using `atan2(depth, x)`. This gives us the angle in the correct quadrant.
- **innerAngle (φ):** This is the angle inside the triangle at the shoulder joint. It represents how much the elbow configuration affects the shoulder rotation.
  - The numerator `L2 * sin(θ₂)` is the perpendicular component of the second link
  - The denominator `L1 + L2 * cos(θ₂)` is the parallel component (first link plus projection of second link)
  - Together, these form a right triangle that lets us calculate the internal angle
- **theta1 (θ₁):** The final shoulder angle is the difference between the target direction and the internal angle.
  - If we only used baseAngle, the shoulder would point directly at the target, but the elbow bend would throw off the end-effector position
  - Subtracting innerAngle compensates for the elbow configuration
- This method automatically handles the "elbow-up" configuration. In a full implementation, you could add logic to choose between "elbow-up" and "elbow-down" solutions.

---

**Step 5: Apply Joint Rotations**

Now we convert the calculated angles from radians to degrees and apply them to the Unity Transform components.

```csharp
float t1Deg = theta1 * Mathf.Rad2Deg;
float t2Deg = theta2 * Mathf.Rad2Deg;

// Apply inversion flags if needed
if (!invertShoulderRotation) t1Deg = -t1Deg;
if (invertElbowRotation) t2Deg = -t2Deg;

// Set joint rotations using quaternions
shoulderRotationBone.localRotation = 
    _shoulderStartRot * Quaternion.AngleAxis(t1Deg + shoulderOffset, Vector3.up);
elbowJoint.localRotation = 
    _elbowStartRot * Quaternion.AngleAxis(t2Deg + elbowOffset, Vector3.up);
```

**Explanation:**
- **Radian to Degree Conversion:** Unity's Quaternion.AngleAxis expects degrees, so we multiply by `Mathf.Rad2Deg` (approximately 57.2958).
- **Inversion Flags:** Different robot models may have joints rotating in opposite directions. These boolean flags let us flip the rotation direction without changing the core IK math.
  - `invertShoulderRotation` and `invertElbowRotation` are public inspector variables
  - This makes the code reusable for different SCARA robot configurations
- **Quaternion Rotation:**
  - `_shoulderStartRot` and `_elbowStartRot` store the initial rotations from the Start() method
  - We multiply this initial rotation by a new rotation around the vertical axis (Vector3.up)
  - This preserves any initial offset in the 3D model while applying our calculated joint angles
- **Offset Calibration:** `shoulderOffset` and `elbowOffset` are adjustable parameters that let us fine-tune the robot alignment if the 3D model doesn't perfectly match our kinematic model.
- We use `localRotation` instead of `rotation` because we want to rotate relative to the parent bone, not in world space.

---

**Step 6: Prismatic Joint Control**

The vertical (Z-axis) motion is controlled by the prismatic joint, which is independent of the planar arm motion.

```csharp
float h = Mathf.Clamp(height, minHeight, maxHeight);
ApplyLift(h);
```

**Explanation:**
- `height` was calculated in Step 1 from the Z-component of the target position.
- `Mathf.Clamp` restricts the height to the valid range (0 to 50 units in this implementation).
- `ApplyLift(h)` is a helper function that sets the vertical position of the shoulder lift bone:
  ```csharp
  void ApplyLift(float val)
  {
      if (invertLift) val = -val;
      Vector3 finalPos = _liftStartPos;
      switch (liftAxis) { 
          case Axis.X: finalPos.x += val; break; 
          case Axis.Y: finalPos.y += val; break; 
          case Axis.Z: finalPos.z += val; break; 
      }
      shoulderLiftBone.localPosition = finalPos;
  }
  ```
- The `liftAxis` enum allows the prismatic joint to work along different axes depending on how the 3D model is constructed.
- `invertLift` handles cases where positive motion should move down instead of up.
- This separation of vertical and horizontal motion is a key advantage of the SCARA configuration—the height can be controlled independently without affecting the XY positioning.

**Advantages of this Approach:**
- **Computational Efficiency**: Closed-form solution (no iterative solving)
- **Deterministic**: Same input always produces same output
- **Real-time Performance**: Suitable for every-frame execution
- **Singularity Handling**: Workspace clamping prevents undefined states

## Path Generation System

The `ManualPathGenerator.cs` implements a stroke-based letter representation system. This system converts text characters into sequences of 3D coordinates that the robot can follow to draw letters.

### Letter Definition Structure:

Each letter in the alphabet is defined as a switch case that adds one or more strokes to a list. Each stroke is a collection of connected points.

**Example: Letter 'A'**
```csharp
case 'A':
    strokes.Add(Stroke(0,0, 2,5));        // Left diagonal
    strokes.Add(Stroke(2,5, 4,0));        // Right diagonal
    strokes.Add(Stroke(1,2.5f, 3,2.5f));  // Horizontal bar
    break;
```

**Explanation:**
- Letter 'A' consists of three separate strokes (three pen movements)
- **First stroke** (left diagonal): Starts at bottom-left (0,0) and goes to the top center (2,5)
- **Second stroke** (right diagonal): Starts at top center (2,5) and goes to bottom-right (4,0)
  - Note: This shares the same starting point as the end of the first stroke, but they're defined as separate strokes. The motion control system will detect this continuity and keep the pen down.
- **Third stroke** (horizontal bar): Draws from left (1, 2.5) to right (3, 2.5) at the middle height
- All coordinates are on a normalized 4×5 grid, which will be scaled later
- The first number in each pair is the X coordinate (horizontal), the second is the Z coordinate (vertical on the drawing surface)

---

### Stroke Creation Helpers:

**1. Linear Stroke Generator**

This helper function creates straight-line paths by accepting a variable number of coordinate pairs.

```csharp
List<Vector3> Stroke(params float[] coords)
{
    List<Vector3> points = new List<Vector3>();
    for (int i = 0; i < coords.Length; i += 2)
    {
        points.Add(new Vector3(coords[i], 0, coords[i+1]));
    }
    return points;
}
```

**Explanation:**
- `params float[] coords` allows you to pass any number of coordinate values as parameters
- The function iterates through coordinates in pairs (x, z)
- For each pair, it creates a Vector3 with:
  - X = coords[i] (horizontal position)
  - Y = 0 (always zero in the local coordinate system; height is controlled separately)
  - Z = coords[i+1] (vertical position on the drawing surface)
- **Example usage:** `Stroke(0,0, 2,5, 4,0)` creates three points: (0,0,0), (2,0,5), (4,0,0)
  - This represents a continuous stroke connecting three points (like drawing a 'V' shape)
- The robot will move smoothly through all these points in sequence without lifting the pen

---

**2. Parametric Arc Generator**

This is the most sophisticated part of the path generator. It creates smooth circular or elliptical arcs using trigonometric functions.

```csharp
List<Vector3> CreateArc(float cx, float cz, float w, float h, 
                        float startAng, float endAng, int res = 20)
{
    List<Vector3> points = new List<Vector3>();
    for (int i = 0; i <= res; i++)
    {
        float t = i / (float)res;
        float ang = Mathf.Lerp(startAng, endAng, t) * Mathf.Deg2Rad;
        
        float x = cx + Mathf.Cos(ang) * w;
        float z = cz + Mathf.Sin(ang) * h;
        
        points.Add(new Vector3(x, 0, z));
    }
    return points;
}
```

**Parameters:**
- `cx, cz`: Center point of the arc
- `w, h`: Width and height radii (for elliptical arcs; use same value for circles)
- `startAng, endAng`: Start and end angles in degrees
  - 0° = right (positive X direction)
  - 90° = up (positive Z direction)
  - 180° = left (negative X direction)
  - 270° or -90° = down (negative Z direction)
- `res`: Resolution (number of line segments to approximate the arc, default 20)

**Explanation:**
- **Loop through resolution:** For i from 0 to 20 (21 points total for default resolution)
- **Interpolation parameter t:** Goes from 0.0 to 1.0 across the loop
  - At i=0: t=0.0 (arc start)
  - At i=10: t=0.5 (arc midpoint)
  - At i=20: t=1.0 (arc end)
- **Angle interpolation:** `Mathf.Lerp(startAng, endAng, t)` smoothly interpolates between start and end angles
  - Example: startAng=0, endAng=90, t=0.5 → ang=45°
  - `* Mathf.Deg2Rad` converts degrees to radians for the trig functions
- **Parametric circle equation:**
  - `x = cx + cos(ang) * w`: Horizontal position along the arc
  - `z = cz + sin(ang) * h`: Vertical position along the arc
- **Why this works:** Cosine and sine trace out a circle when the angle varies. By using different radii (w and h), we can create ellipses.
- **Result:** Instead of a jagged approximation, we get 20 smooth segments that closely approximate a perfect curve

---

**Example: Letter 'S' Implementation**

The letter 'S' demonstrates how to combine multiple arcs to create complex curved shapes.

```csharp
case 'S':
    // Top arc: starts at 45°, goes counterclockwise to 270°
    strokes.Add(CreateArc(2f, 3.75f, 2f, 1.25f, 45, 270));
    
    // Bottom arc: starts at 90°, goes clockwise to -135°
    strokes.Add(CreateArc(2f, 1.25f, 2f, 1.25f, 90, -135));
    break;
```

**Explanation:**
- **Top arc:**
  - Center: (2, 3.75) - positioned in the upper portion
  - Radii: width=2, height=1.25 (elliptical, wider than tall)
  - Starts at 45° (upper-right) and sweeps counterclockwise to 270° (bottom)
  - Angle span: 270° - 45° = 225° of rotation
  - This creates the curved top portion of the 'S'
  - End point is at the middle-left of the arc
- **Bottom arc:**
  - Center: (2, 1.25) - positioned in the lower portion
  - Radii: width=2, height=1.25 (same ellipse size as top arc)
  - Starts at 90° (top of arc) and sweeps clockwise to -135°
  - Angle span: 90° to -135° = 225° in the opposite direction
  - This creates the curved bottom portion of the 'S'
- **Key design:** The end point of the top arc (270° on the top arc) is at approximately the same location as the start point of the bottom arc (90° on the bottom arc)
- The continuity detection system will recognize these points are connected and won't lift the pen between them, creating a smooth 'S' shape

---

**Example: Letter 'G' Implementation**

Letter 'G' shows how to combine an arc with a straight line.

```csharp
case 'G':
    // Main circular arc: 45° to 360° (315° span)
    strokes.Add(CreateArc(2.5f, 2.5f, 2.5f, 2.5f, 45, 360));
    
    // Horizontal hook connecting at (5.0, 2.5)
    strokes.Add(Stroke(5.0f, 2.5f, 2.5f, 2.5f));
    break;
```

**Explanation:**
- **Main arc:**
  - Center: (2.5, 2.5) - centered in the letter
  - Radii: 2.5 for both width and height (perfect circle)
  - Starts at 45° (upper-right)
  - Ends at 360° (same as 0°, which is the right side)
  - Angle span: 315° (almost a full circle, leaving a gap on the upper-right)
  - End point calculation: x = 2.5 + cos(360°)×2.5 = 2.5 + 2.5 = 5.0, z = 2.5 + sin(360°)×2.5 = 2.5
- **Hook stroke:**
  - Starts at (5.0, 2.5) - exactly where the arc ended
  - Ends at (2.5, 2.5) - the center of the circle
  - This draws a horizontal line inward, creating the characteristic 'G' hook
- **Perfect connection:** The arc's end point and the hook's start point have identical coordinates, so the motion controller keeps the pen down for a continuous stroke

---

### Coordinate Processing Pipeline:

After defining the letter geometry, we need to transform these normalized coordinates into real world positions.

```csharp
private List<Vector3> ProcessPoints(List<Vector3> rawPoints, float xOffset)
{
    List<Vector3> processed = new List<Vector3>();
    foreach (Vector3 p in rawPoints)
    {
        // Apply character offset and scale
        float x = (p.x + xOffset) * scale;
        float z = p.z * scale;
        
        // Transform to world coordinates
        Vector3 worldPos = transform.TransformPoint(
            new Vector3(x, 0, z) + offset
        );
        processed.Add(worldPos);
    }
    return processed;
}
```

**Explanation:**
- **Input:** `rawPoints` contains the normalized coordinates (on the 4×5 grid), `xOffset` is the horizontal offset for letter spacing
- **Character offset:** `p.x + xOffset` shifts each letter to the right based on how many letters have been drawn before
  - First letter: xOffset = 0
  - Second letter: xOffset = 4 + spacing
  - Third letter: xOffset = 2 × (4 + spacing)
- **Scaling:** Multiplying by `scale` (default 20) converts the small normalized coordinates to robot-sized coordinates
  - Example: A point at (2, 5) becomes (40, 100) with scale=20
- **Add base offset:** The `offset` parameter lets you shift the entire text to a different location on the drawing surface
- **World transformation:** `transform.TransformPoint()` converts from local space to world space
  - Takes into account the PathGenerator object's position, rotation, and scale in the Unity scene
  - This allows you to position and rotate the text by moving the PathGenerator GameObject
- **Result:** A list of world-space Vector3 coordinates ready for the robot to follow

## Motion Control Strategy

The `RobotWriter.cs` implements an optimized pen control algorithm that minimizes unnecessary pen lifts and creates smooth, efficient drawing motions. This is implemented as a coroutine-based state machine.

### Stroke Continuity Detection:

Before executing each stroke, the system checks whether it connects to the next stroke. This allows the pen to stay down when drawing connected segments.

```csharp
// Check if next stroke connects to current stroke
bool nextStrokeIsContinuous = false;
if (strokeIndex < strokes.Count - 1)
{
    Vector3 currentEnd = currentStroke[currentStroke.Count - 1];
    Vector3 nextStart = strokes[strokeIndex + 1][0];
    
    // Compare positions (ignoring height)
    Vector3 endFlat = new Vector3(currentEnd.x, 0, currentEnd.z);
    Vector3 nextStartFlat = new Vector3(nextStart.x, 0, nextStart.z);
    
    if (Vector3.Distance(endFlat, nextStartFlat) < 0.01f)
    {
        nextStrokeIsContinuous = true;
    }
}
```

**Explanation:**
- **Check if there's a next stroke:** `strokeIndex < strokes.Count - 1` ensures we don't try to access an out-of-bounds stroke
- **Get endpoint positions:**
  - `currentEnd`: The last point of the current stroke (where the pen will be after drawing this stroke)
  - `nextStart`: The first point of the next stroke (where the pen needs to be to start the next stroke)
- **Flatten to 2D:** We create `endFlat` and `nextStartFlat` by setting the Y component to 0
  - This ignores the height difference and only compares horizontal (XZ) positions
  - We don't care if one point is at penUpHeight and another at penDownHeight; we only care if they're in the same XZ location
- **Distance check:** `Vector3.Distance(endFlat, nextStartFlat) < 0.01f`
  - If the horizontal distance is less than 0.01 units, we consider them "the same point"
  - The threshold of 0.01 is small enough to catch intentional connections but large enough to handle floating-point rounding errors
- **Set flag:** If the strokes connect, we set `nextStrokeIsContinuous = true`
- **Purpose:** This information is used later to decide whether to lift the pen after completing the current stroke
  - If continuous: Keep pen down and immediately start drawing the next stroke
  - If not continuous: Lift pen, move to the new position, then lower pen again

---

### State Machine Logic:

The drawing sequence for each stroke follows a state machine with 4 possible states. The system intelligently skips states when they're not needed.

```csharp
// Determine if pen is already at start position
bool isAlreadyAtStartPoint = Vector3.Distance(
    scaraController.targetObj.position, 
    new Vector3(startP.x, scaraController.targetObj.position.y, startP.z)
) < 0.01f;

// State 1: Move to start position (if not already there)
if (strokeIndex > 0 && !isAlreadyAtStartPoint)
{
    yield return MoveTo(new Vector3(startP.x, penUpHeight, startP.z));
}
else if (strokeIndex == 0)
{
    // First stroke always starts at safe height
    yield return MoveTo(new Vector3(startP.x, penUpHeight, startP.z));
}

// State 2: Lower pen to drawing surface
if (!isAlreadyAtStartPoint)
{
    yield return MoveTo(new Vector3(startP.x, penDownHeight, startP.z));
}
if (_trail) _trail.emitting = true;

// State 3: Draw stroke
for (int i = 1; i < currentStroke.Count; i++)
{
    Vector3 p = currentStroke[i];
    yield return MoveTo(new Vector3(p.x, penDownHeight, p.z));
}

// State 4: Lift pen (only if next stroke is discontinuous)
if (!nextStrokeIsContinuous)
{
    if (_trail) _trail.emitting = false;
    yield return MoveTo(new Vector3(endP.x, penUpHeight, endP.z));
}
```

**Explanation:**

**Pre-State: Check Current Position**
- `isAlreadyAtStartPoint` checks if the robot is already horizontally positioned at the stroke's start point
- We only check XZ position (horizontal), ignoring Y (height)
- This is true when the previous stroke ended at the same location as the current stroke starts
- **Example:** Drawing letter 'M' - stroke 2 starts where stroke 1 ended, so no horizontal movement is needed

**State 1: Move to Start Position (Hover)**
- **Condition 1:** `if (strokeIndex > 0 && !isAlreadyAtStartPoint)`
  - For strokes after the first one (strokeIndex > 0)
  - And only if we're not already at the start position
  - Move to the start point at the "pen up" height (hovering above the drawing surface)
- **Condition 2:** `else if (strokeIndex == 0)`
  - The very first stroke always needs to move to its start position
  - This ensures the robot starts from a safe home position
- **Why this matters:** Prevents the pen from dragging across the surface when moving between disconnected strokes
- **Optimization:** If the previous stroke ended where this one starts, this entire state is skipped

**State 2: Lower Pen**
- **Condition:** `if (!isAlreadyAtStartPoint)`
  - Only lower the pen if we moved to a new position
  - If we're already at the right XZ position from a continuous stroke, the pen is already down
- **Action:** Move vertically from `penUpHeight` to `penDownHeight`
- **Trail renderer:** `_trail.emitting = true` starts drawing the visible ink line
- **Example:** For letter 'A' stroke 1, this lowers the pen from height 20 to height 0

**State 3: Draw the Stroke**
- **Loop:** `for (int i = 1; i < currentStroke.Count; i++)`
  - Start at index 1 (not 0) because we're already at the first point
  - Move through all remaining points in the stroke
- **Action:** `MoveTo(new Vector3(p.x, penDownHeight, p.z))`
  - Keep the pen at drawing height (penDownHeight) for all points
  - Only the XZ position changes as we trace out the letter
- **Result:** A continuous line is drawn connecting all points in the stroke
- **Example:** For a 20-point arc, this executes 19 MoveTo commands

**State 4: Lift Pen**
- **Condition:** `if (!nextStrokeIsContinuous)`
  - Only lift the pen if the next stroke is NOT connected to this one
  - This is the key optimization—we skip this state for connected strokes
- **Actions:**
  - `_trail.emitting = false` stops drawing the visible ink line
  - Move vertically from penDownHeight to penUpHeight
  - Stay at the current XZ position (the end of the stroke)
- **Example:** 
  - Letter 'A': Lift pen after each of the 3 separate strokes
  - Letter 'M': Don't lift pen between the 5 connected line segments

**State Machine Flow Examples:**

*Letter 'M' (connected strokes):*
1. Stroke 1: State 1 → State 2 → State 3 → ~~State 4 (skipped)~~
2. Stroke 2: ~~State 1 (skipped)~~ → ~~State 2 (skipped)~~ → State 3 → ~~State 4 (skipped)~~
3. Stroke 3: ~~State 1 (skipped)~~ → ~~State 2 (skipped)~~ → State 3 → ~~State 4 (skipped)~~
4. Stroke 4: ~~State 1 (skipped)~~ → ~~State 2 (skipped)~~ → State 3 → ~~State 4 (skipped)~~
5. Stroke 5: ~~State 1 (skipped)~~ → ~~State 2 (skipped)~~ → State 3 → State 4

Result: Pen is only lowered once and lifted once for the entire letter!

*Letter 'A' (disconnected strokes):*
1. Stroke 1: State 1 → State 2 → State 3 → State 4
2. Stroke 2: State 1 → State 2 → State 3 → State 4
3. Stroke 3: State 1 → State 2 → State 3 → State 4

Result: Pen is lowered and lifted for each separate stroke.

---

### Smooth Interpolation:

The `MoveTo` coroutine provides frame-independent, smooth linear interpolation between positions.

```csharp
IEnumerator MoveTo(Vector3 destination)
{
    Transform target = scaraController.targetObj;
    float dist = Vector3.Distance(target.position, destination);
    if (writeSpeed <= 0.1f) writeSpeed = 1f;

    float duration = dist / writeSpeed;
    float elapsed = 0f;
    Vector3 start = target.position;

    while (elapsed < duration)
    {
        // Linear interpolation with time-based progression
        target.position = Vector3.Lerp(start, destination, elapsed / duration);
        elapsed += Time.deltaTime;
        yield return null; // Wait for next frame
    }
    
    // Ensure exact final position
    target.position = destination;
}
```

**Explanation:**

**Setup Phase:**
- `target = scaraController.targetObj` - Get reference to the target object (the invisible point the robot follows)
- `dist = Vector3.Distance(target.position, destination)` - Calculate the straight-line distance to travel
- **Safety check:** `if (writeSpeed <= 0.1f) writeSpeed = 1f`
  - Prevents division by zero or near-zero values
  - Ensures motion always completes in finite time
- `duration = dist / writeSpeed` - Calculate how long the movement should take
  - Example: distance=100 units, speed=20 units/sec → duration=5 seconds
  - This ensures constant speed regardless of distance

**Interpolation Loop:**
- `elapsed = 0f` - Initialize the time counter
- `start = target.position` - Store the starting position
- **While loop:** Continues until we've spent `duration` seconds moving
- `target.position = Vector3.Lerp(start, destination, elapsed / duration)`
  - **Lerp** (Linear Interpolation) smoothly blends between start and destination
  - The third parameter `elapsed / duration` is the interpolation factor (0 to 1)
  - At elapsed=0: factor=0 → position=start
  - At elapsed=duration/2: factor=0.5 → position=halfway between start and destination
  - At elapsed=duration: factor=1 → position=destination
- `elapsed += Time.deltaTime` - Increment the timer by the time since the last frame
  - `Time.deltaTime` is approximately 0.0167 seconds at 60 FPS
  - This makes the motion frame-rate independent (same speed at 30 FPS or 120 FPS)
- `yield return null` - Pause the coroutine and resume on the next frame
  - This is what makes the movement animated rather than instant
  - The robot moves a little bit each frame, creating smooth motion

**Finalization:**
- `target.position = destination` - Snap to the exact final position
- **Why this is necessary:** Due to floating-point precision and timing, the loop might end slightly before reaching factor=1.0
- This guarantees we end up exactly at the destination, which is crucial for stroke continuity detection

**Key Advantages:**
- **Constant velocity:** The speed is the same regardless of distance (unlike exponential interpolation)
- **Frame-rate independent:** Runs at the same real-world speed on different computers
- **Predictable timing:** You can calculate exactly when the motion will complete
- **Smooth visual:** No jerky movements or sudden stops

---

## Summary of Algorithm Integration

The three major components work together in a pipeline:

1. **Path Generation (ManualPathGenerator)** converts user text into world-space waypoints
   - Input: String "HELLO"
   - Output: List of stroke sequences with 3D coordinates

2. **Motion Control (RobotWriter)** orchestrates the drawing sequence
   - Manages pen up/down states
   - Detects stroke continuity to optimize pen lifts
   - Commands the target object to move through waypoints smoothly

3. **Inverse Kinematics (ScaraController)** follows the target in real-time
   - Runs every frame in Update() loop
   - Calculates joint angles (θ₁, θ₂, d₃) needed to reach target position
   - Applies rotations to Unity Transform components

**Execution Flow:**
```
User clicks "Start" 
  → RobotWriter receives text from UI
  → PathGenerator creates stroke list
  → WriteRoutine coroutine begins
    → For each stroke:
      → MoveTo(start position)
        → ScaraController.Update() runs continuously
          → SolveIK(targetObj.position)
          → Apply joint rotations
      → MoveTo(each waypoint in stroke)
        → Robot traces out the letter shape
      → Conditionally lift pen
  → Return to home position
```

This separation of concerns makes the code modular, testable, and easy to extend with new features.

**Benefits of This Approach:**
- **Reduced Air Time**: Eliminates unnecessary pen lifts for connected strokes
- **Smooth Motion**: Linear interpolation prevents jerky movements
- **Frame-Independent**: Movement speed remains consistent regardless of framerate
- **Predictable Timing**: Duration is calculated deterministically from distance

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
