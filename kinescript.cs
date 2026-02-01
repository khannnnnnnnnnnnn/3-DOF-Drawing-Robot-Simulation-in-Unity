using UnityEngine;
using UnityEngine.InputSystem; 

public class ScaraController : MonoBehaviour
{
    // การนิยาม ControlMode ที่ถูกต้อง (เก็บไว้)
    public enum ControlMode { AutomaticIK, ManualJoints, LinearJog }
    public enum Axis { X, Y, Z } 

    [Header("Control Mode")]
    public ControlMode currentMode = ControlMode.AutomaticIK;
    public float jogSpeed = 20.0f; 

    [Header("Joint Configuration")]
    public Transform shoulderRotationBone; 
    public Transform shoulderLiftBone; 
    public Transform elbowJoint; 

    [Header("Prismatic (Lift) Settings")]
    public bool usePrismaticHeight = true;
    public Axis liftAxis = Axis.Z; 
    public bool invertLift = true; 
    public float minHeight = 0f;
    public float maxHeight = 50f; 

    [Header("Arm Lengths")]
    public float L1 = 230f; 
    public float L2 = 135f; 

    [Header("Calibration")]
    [Range(-180, 180)] public float shoulderOffset = 0f;
    [Range(-180, 180)] public float elbowOffset = 0f;
    public bool invertShoulderRotation = false; 
    public bool invertElbowRotation = false; 

    [Header("Workspace Visualization")]
    public bool showWorkspace = true; 
    public Color workspaceColor = new Color(1, 0, 0, 0.5f); 
    public Vector3 workspaceCenterOffset = Vector3.zero; 
    private LineRenderer _outerRing;
    private LineRenderer _innerRing;

    [Header("Target & Manual")]
    public Transform targetObj;
    [Range(0, 100)] public float manualHeight = 0f;
    [Range(-180, 180)] public float manualShoulderAngle = 0f;
    [Range(-180, 180)] public float manualElbowAngle = 0f;

    // Internal Memory
    private Quaternion _shoulderStartRot;
    private Quaternion _elbowStartRot;
    private Vector3 _liftStartPos;

    // DEBUG VARIABLES
    private float _debugTheta1;
    private float _debugTheta2;
    private float _debugHeight;

    void Start()
    {
        if (shoulderRotationBone) _shoulderStartRot = shoulderRotationBone.localRotation;
        if (elbowJoint) _elbowStartRot = elbowJoint.localRotation;
        if (shoulderLiftBone) _liftStartPos = shoulderLiftBone.localPosition;
        UpdateWorkspaceVisuals();
    }

    void OnValidate() { 
#if UNITY_EDITOR
        UnityEditor.EditorApplication.delayCall += () => { if (this != null) UpdateWorkspaceVisuals(); };
#endif
    }
    
    // โค้ดที่ซ้ำซ้อนถูกลบออกแล้ว

    // ใน ScaraController.cs

void Update()
{
    // โค้ดนี้จะทำงานเมื่อ TargetMouseControl สลับโหมดเป็น AutomaticIK (เมื่อคลิกเมาส์)
    if (currentMode == ControlMode.AutomaticIK) 
    { 
        if (targetObj != null) 
        {
            // ใช้ SolveIK เพื่อให้แขนหุ่นยนต์ตาม Target object
            SolveIK(targetObj.position);
        } 
    }
    else if (currentMode == ControlMode.LinearJog) 
    { 
        // โหมดนี้จะทำงานเมื่อ TargetMouseControl สลับโหมดกลับมา (เมื่อปล่อยเมาส์)
        HandleLinearJog(); 
    }
    else 
    { 
        ApplyManualControl(); 
    }
}
// ... (ส่วนอื่นๆ ของ ScaraController)

    // --- PIVOT-CORRECTED GIZMO ---
    void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        // 1. Math Calculation
        float t1 = _debugTheta1; 
        float t2 = _debugTheta2;
        float h = _debugHeight;

        // 2. Start at the Shoulder Pivot (Not the Base!)
        Vector3 startPoint = transform.position;
        if (shoulderRotationBone != null) 
        {
            startPoint.x = shoulderRotationBone.position.x;
            startPoint.z = shoulderRotationBone.position.z;
        }

        // 3. We need the "Forward" and "Right" vectors of the Base
        // because the robot is rotated -90 degrees.
        // Assuming:
        // Local X (Right) = Unity Right
        // Local Y (Forward) = Unity Up (because of -90 X rotation)
        Vector3 rightDir = transform.right; 
        Vector3 forwardDir = transform.up; 
        Vector3 upDir = transform.forward; // Z is Up

        // 4. Calculate Vector Offsets based on Angles
        // X component moves along RightDir, Y component moves along ForwardDir
        float x1 = L1 * Mathf.Cos(t1);
        float y1 = L1 * Mathf.Sin(t1);
        
        float x2 = L1 * Mathf.Cos(t1) + L2 * Mathf.Cos(t1 + t2);
        float y2 = L1 * Mathf.Sin(t1) + L2 * Mathf.Sin(t1 + t2);

        // 5. Apply to World Space
        // Shoulder -> Elbow
        Vector3 elbowPos = startPoint + (rightDir * x1) + (forwardDir * y1);
        // Shoulder -> Hand
        Vector3 handPos = startPoint + (rightDir * x2) + (forwardDir * y2);
        
        // Add Height (Prismatic)
        Vector3 heightOffset = upDir * h; 
        Vector3 shoulderLifted = startPoint + heightOffset;
        elbowPos += heightOffset;
        handPos += heightOffset;

        // 6. Draw
        Gizmos.color = Color.cyan; 
        Gizmos.DrawLine(transform.position, shoulderLifted); // Base to Shoulder
        Gizmos.DrawLine(shoulderLifted, elbowPos);           // Shoulder to Elbow
        Gizmos.DrawLine(elbowPos, handPos);                  // Elbow to Hand
        
        Gizmos.DrawSphere(shoulderLifted, 2f);
        Gizmos.DrawSphere(elbowPos, 5f);
        Gizmos.DrawSphere(handPos, 5f);

        Gizmos.color = Color.yellow;
        if(targetObj != null) Gizmos.DrawLine(handPos, targetObj.position);
    }

    // --- PIVOT-CORRECTED IK SOLVER ---
    void SolveIK(Vector3 targetPos)
    {
        // 1. Calculate Target Relative to SHOULDER (Not Base)
        Vector3 shoulderWorldPos = transform.position;
        if (shoulderRotationBone != null) 
        {
            shoulderWorldPos.x = shoulderRotationBone.position.x;
            shoulderWorldPos.z = shoulderRotationBone.position.z;
            // Keep Y the same as base for the planar calculation
            shoulderWorldPos.y = transform.position.y;
        }

        // Vector from Shoulder Pivot to Target
        Vector3 diffWorld = targetPos - shoulderWorldPos;

        // 2. Convert to Local Space (Handling -90 Rotation)
        // We project this vector onto the Base's axes
        float x = Vector3.Dot(diffWorld, transform.right); // Local X
        float depth = Vector3.Dot(diffWorld, transform.up); // Local Y (Forward)
        float height = Vector3.Dot(diffWorld, transform.forward); // Local Z (Height)

        // 3. Lift Logic
        if (usePrismaticHeight && shoulderLiftBone != null)
        {
            float h = Mathf.Clamp(height, minHeight, maxHeight);
            ApplyLift(h);
            manualHeight = h;
            _debugHeight = h;
        }

        // 4. IK Math (Same as before, but now X/Depth are pivot-corrected)
        float dist = Mathf.Sqrt(x*x + depth*depth);
        float combinedLength = L1 + L2;
        if (dist > combinedLength) dist = combinedLength - 0.001f;
        if (dist < Mathf.Abs(L1 - L2)) dist = Mathf.Abs(L1 - L2) + 0.001f;

        float cosT2 = (dist*dist - L1*L1 - L2*L2) / (2 * L1 * L2);
        float theta2 = Mathf.Acos(Mathf.Clamp(cosT2, -1f, 1f)); 

        float baseAngle = Mathf.Atan2(depth, x);
        float innerAngle = Mathf.Atan2(L2 * Mathf.Sin(theta2), L1 + L2 * Mathf.Cos(theta2));
        float theta1 = baseAngle - innerAngle;

        _debugTheta1 = theta1;
        _debugTheta2 = theta2;

        float t1Deg = theta1 * Mathf.Rad2Deg; 
        float t2Deg = theta2 * Mathf.Rad2Deg;

        if (!invertShoulderRotation) t1Deg = -t1Deg; 
        if (invertElbowRotation) t2Deg = -t2Deg;

        if (shoulderRotationBone != null)
            shoulderRotationBone.localRotation = _shoulderStartRot * Quaternion.AngleAxis(t1Deg + shoulderOffset, Vector3.up);
        
        if (elbowJoint != null)
            elbowJoint.localRotation = _elbowStartRot * Quaternion.AngleAxis(t2Deg + elbowOffset, Vector3.up);

        manualShoulderAngle = t1Deg;
        manualElbowAngle = t2Deg;
    }

    // (Rest of script functions: HandleLinearJog, ApplyManualControl, etc... stay the same)
    void HandleLinearJog()
    {
        if (targetObj == null || Keyboard.current == null) return;
        float speed = jogSpeed * Time.deltaTime;
        if (Keyboard.current.leftShiftKey.isPressed) speed *= 3f;

        float x = 0f; float z = 0f; float lift = 0f;
        if (Keyboard.current.leftArrowKey.isPressed) x = -speed;
        if (Keyboard.current.rightArrowKey.isPressed) x = speed;
        if (Keyboard.current.downArrowKey.isPressed) z = -speed;
        if (Keyboard.current.upArrowKey.isPressed) z = speed;
        if (Keyboard.current.qKey.isPressed) lift = speed;
        if (Keyboard.current.eKey.isPressed) lift = -speed;

        targetObj.Translate(x, lift, z, Space.World);
        SolveIK(targetObj.position);
    }

    void ApplyManualControl()
    {
        if (usePrismaticHeight && shoulderLiftBone != null)
        {
            float height = Mathf.Clamp(manualHeight, minHeight, maxHeight);
            ApplyLift(height);
        }
        if (shoulderRotationBone != null)
        {
            float sAngle = manualShoulderAngle + shoulderOffset;
            shoulderRotationBone.localRotation = _shoulderStartRot * Quaternion.AngleAxis(sAngle, Vector3.up);
        }
        if (elbowJoint != null)
        {
            float eAngle = manualElbowAngle + elbowOffset;
            elbowJoint.localRotation = _elbowStartRot * Quaternion.AngleAxis(eAngle, Vector3.up);
        }
    }
    
    void UpdateWorkspaceVisuals()
    {
        if (!showWorkspace) return;
        CreateRing(ref _outerRing, "Workspace_Outer", L1 + L2);
        CreateRing(ref _innerRing, "Workspace_Inner", Mathf.Abs(L1 - L2));
    }

    void CreateRing(ref LineRenderer lr, string name, float radius)
    {
        if (lr == null) {
            Transform t = transform.Find(name);
            if (t == null) { t = new GameObject(name).transform; t.SetParent(transform); }
            lr = t.gameObject.GetComponent<LineRenderer>();
            if (lr == null) lr = t.gameObject.AddComponent<LineRenderer>();
        }
        lr.gameObject.SetActive(true);
        lr.useWorldSpace = true; 
        lr.loop = true; lr.positionCount = 61; lr.startWidth = 2.0f; lr.endWidth = 2.0f;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = workspaceColor; lr.endColor = workspaceColor;

        Vector3 center = transform.position;
        if (shoulderRotationBone != null) { center.x = shoulderRotationBone.position.x; center.z = shoulderRotationBone.position.z; }
        Vector3 adjustedOffset = transform.TransformDirection(workspaceCenterOffset);
        center += adjustedOffset;
        center.y = transform.position.y + 0.1f; 

        for (int i = 0; i <= 60; i++) {
            float angle = (i / 60f) * 360f * Mathf.Deg2Rad;
            lr.SetPosition(i, new Vector3(center.x + Mathf.Cos(angle) * radius, center.y, center.z + Mathf.Sin(angle) * radius));
        }
    }

    void ApplyLift(float val)
    {
        if (invertLift) val = -val;
        Vector3 finalPos = _liftStartPos;
        switch (liftAxis) { case Axis.X: finalPos.x += val; break; case Axis.Y: finalPos.y += val; break; case Axis.Z: finalPos.z += val; break; }
        shoulderLiftBone.localPosition = finalPos;
    }
}