using UnityEngine;
using TMPro;

public class RobotUIManager : MonoBehaviour
{
    [Header("References")]
    public RobotWriter robotWriter;
    public TMP_InputField inputField;
    
    [Header("Camera Control")]
    public Camera mainCamera; // Drag your Main Camera here
    public Camera penCam;     // Drag your dedicated Pen Camera here

    // Flag to track which camera is currently active
    private bool isMainCamActive = true; 

    public void OnStartWritingButtonPressed()
    {
        if (robotWriter == null)
        {
            Debug.LogError("RobotWriter reference is missing in the UI Manager.");
            return;
        }

        string text = inputField.text;

        if (string.IsNullOrEmpty(text))
        {
            Debug.LogWarning("Input field is empty. Please type text to write.");
            return;
        }

        // --- Data Transfer ---
        // Note: You must ensure 'textToWrite' is public in RobotWriter.cs
        robotWriter.textToWrite = text; 
        
        // 2. Start the writing sequence
        robotWriter.StartWriting();
        
        // 3. Clear focus from the input field
        inputField.DeactivateInputField();
    }
    
    public void ToggleCameraView()
    {
        if (mainCamera == null || penCam == null)
        {
            Debug.LogError("Camera references are missing in the UI Manager.");
            return;
        }

        if (isMainCamActive)
        {
            // Switch to Pen View
            mainCamera.gameObject.SetActive(false);
            penCam.gameObject.SetActive(true);
            isMainCamActive = false;
        }
        else
        {
            // Switch back to Main View
            penCam.gameObject.SetActive(false);
            mainCamera.gameObject.SetActive(true);
            isMainCamActive = true;
        }
    }
}