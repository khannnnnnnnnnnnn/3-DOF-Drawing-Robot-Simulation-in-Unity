using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem; 
using TMPro; // Added for the UI Manager script (to use TMP_InputField)

public class RobotWriter : MonoBehaviour
{
    [Header("References")]
    public ScaraController scaraController;
    // We reference the PathGenerator by its implemented interface, not its class name
    public ManualPathGenerator pathGenerator; 
    public Transform penTip; 

    [Header("Configuration")]
    public Font myFont; 
    
    // --- FIX: PUBLIC PROPERTY SETTER FOR UI MANAGER ---
    // Change the property back to a public field to match the error's expectation
    [HideInInspector] public string textToWrite { get => _internalTextToDraw; set => _internalTextToDraw = value; }
    // ----------------------------------------------------
    public float writeSpeed = 20f; 
    public float penUpHeight = 20.0f;   
    public float penDownHeight = 0.0f; 

    [Header("Home Position")]
    // Defines a point in the safe zone (e.g., fully extended straight forward)
    public Vector3 homePosition = new Vector3(300, 20, 0); 

    [Header("Preview Settings")]
    public bool showPreview = true;
    public Color previewColor = Color.yellow;

    private bool isWriting = false;
    private TrailRenderer _trail; 
    private string _internalTextToDraw = ""; // Internal variable to hold the text

    void Start()
    {
        // --- AUDIO LISTENER FIX ---
        EnsureSingleAudioListener();
        // ---------------------------

        if (penTip != null)
        {
            _trail = penTip.GetComponent<TrailRenderer>();
            if (_trail != null) _trail.emitting = false; 
        }
        
        // Ensure Home Position uses the correct Y height (Y is Up in Unity)
        homePosition.y = penUpHeight;

        // NEW: Move to home position immediately when the program starts
        StartCoroutine(MoveToHomeAtStart());
    }
    
    void EnsureSingleAudioListener()
    {
        AudioListener[] listeners = FindObjectsOfType<AudioListener>();
        if (listeners.Length > 1)
        {
            Debug.LogWarning($"Found {listeners.Length} Audio Listeners. Disabling duplicates.");
            // Disable all listeners except the first one found
            for (int i = 1; i < listeners.Length; i++)
            {
                listeners[i].enabled = false;
            }
        }
    }

    IEnumerator MoveToHomeAtStart()
    {
        // Give the physics/IK system one frame to stabilize
        yield return null; 
        
        Debug.Log("Initializing robot. Moving to Home Position.");
        // We use MoveTo to smoothly transition to the home position
        yield return MoveTo(homePosition);
    }

    // REMOVED: The Update() function that listens for Input.GetKeyDown(KeyCode.Space)

    [ContextMenu("Start Writing")] 
    public void StartWriting()
    {
        // Fallback for context menu if text isn't set via UI
        if (string.IsNullOrEmpty(_internalTextToDraw))
        {
            Debug.LogError("Error: Text is empty. Please type something in the UI.");
            return;
        }
        
        if (isWriting) return;
        StartCoroutine(WriteRoutine());
    }
    
    // --- OLD: FUNCTION TO RECEIVE TEXT FROM UI ---
    // Note: This method is now redundant, but kept for compatibility.
    public void ReceiveTextAndStart(string textFromInput)
    {
        _internalTextToDraw = textFromInput;
        StartWriting();
    }
    // ---------------------------------------------

    void OnDrawGizmos()
    {
        if (!showPreview || pathGenerator == null) return;
        try {
            // Use the internal text variable for the preview
            List<List<Vector3>> strokes = pathGenerator.GetPathsForText(_internalTextToDraw, myFont);
            Gizmos.color = previewColor;
            foreach (List<Vector3> stroke in strokes) {
                if (stroke.Count < 2) continue;
                for (int i = 0; i < stroke.Count - 1; i++) {
                    Vector3 floorOffset = new Vector3(0, 0.1f, 0); 
                    Gizmos.DrawLine(stroke[i] + floorOffset, stroke[i + 1] + floorOffset);
                }
            }
            // Draw a small sphere at the Home Position for visualization
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(homePosition, 10f);
        } catch {}
    }

    IEnumerator WriteRoutine()
    {
        isWriting = true;
        scaraController.currentMode = ScaraController.ControlMode.AutomaticIK;

        // 1. CLEAR OLD INK
        if (_trail != null)
        {
            _trail.emitting = false; 
            _trail.Clear();          
        }

        List<List<Vector3>> strokes = pathGenerator.GetPathsForText(_internalTextToDraw, myFont);
        
        if (strokes.Count == 0)
        {
            isWriting = false;
            yield break;
        }

        // Loop through strokes using index to peek ahead
        for (int strokeIndex = 0; strokeIndex < strokes.Count; strokeIndex++)
        {
            List<Vector3> currentStroke = strokes[strokeIndex];
            if (currentStroke.Count == 0) continue;
            
            Vector3 startP = currentStroke[0];
            Vector3 endP = currentStroke[currentStroke.Count - 1];

            // Determine if the next stroke is continuous
            bool nextStrokeIsContinuous = false;
            if (strokeIndex < strokes.Count - 1)
            {
                Vector3 nextStartP = strokes[strokeIndex + 1][0];
                
                // Compare the end of current stroke with the start of the next stroke
                Vector3 endFlat = new Vector3(endP.x, 0, endP.z);
                Vector3 nextStartFlat = new Vector3(nextStartP.x, 0, nextStartP.z);
                
                if (Vector3.Distance(endFlat, nextStartFlat) < 0.01f)
                {
                    nextStrokeIsContinuous = true;
                }
            }

            // --- MOVEMENT LOGIC ---

            // CHECK: Is the pen currently at the desired start position?
            // This is primarily for detecting if the end of the previous stroke seamlessly meets the start of this one.
            bool isAlreadyAtStartPoint = Vector3.Distance(scaraController.targetObj.position, new Vector3(startP.x, scaraController.targetObj.position.y, startP.z)) < 0.01f;


            // 1. PEN UP/HOVER MOTION (Skip if already at the start/continuous)
            if (strokeIndex > 0 && !isAlreadyAtStartPoint)
            {
                // We only need to move if the target isn't the current spot
                 yield return MoveTo(new Vector3(startP.x, penUpHeight, startP.z));
            } 
            else if (strokeIndex == 0)
            {
                // First stroke always starts at safe height
                yield return MoveTo(new Vector3(startP.x, penUpHeight, startP.z));
            }


            // 2. PEN DOWN MOTION
            // Only move down if the robot isn't already on the table (i.e., not a continuous move)
            if (!isAlreadyAtStartPoint)
            {
                yield return MoveTo(new Vector3(startP.x, penDownHeight, startP.z));
            }
            if (_trail) _trail.emitting = true;


            // 3. DRAW: Stay down and move continuously
            for (int i = 1; i < currentStroke.Count; i++)
            {
                Vector3 p = currentStroke[i];
                yield return MoveTo(new Vector3(p.x, penDownHeight, p.z));
            }

            // 4. LIFT LOGIC: Only lift if there is no continuous line coming up
            if (!nextStrokeIsContinuous)
            {
                if (_trail) _trail.emitting = false;
                // Move from the end point up to the hover height
                yield return MoveTo(new Vector3(endP.x, penUpHeight, endP.z)); 
            }
        }
        
        // 5. FINAL LIFT AND HOME
        if (_trail) _trail.emitting = false;
        
        // Ensure the robot is at safety height before moving home
        Vector3 finalEndP = strokes[strokes.Count - 1][strokes[strokes.Count - 1].Count - 1];
        yield return MoveTo(new Vector3(finalEndP.x, penUpHeight, finalEndP.z));

        Debug.Log("Writing finished. Returning to Home Position.");
        yield return MoveTo(homePosition);
        
        isWriting = false;
    }

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
            target.position = Vector3.Lerp(start, destination, elapsed / duration);
            elapsed += Time.deltaTime;
            yield return null;
        }
        target.position = destination;
    }
}