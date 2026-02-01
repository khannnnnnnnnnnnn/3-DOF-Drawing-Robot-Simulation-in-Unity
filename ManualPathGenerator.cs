using System.Collections.Generic;
using UnityEngine;

public class ManualPathGenerator : MonoBehaviour
{
    [Header("Settings")]
    public float scale = 20f; 
    public float spacing = 1.5f; // Distance between letters (relative to scale)
    public Vector3 offset = new Vector3(0, 0, 0); 

    // Define standard width for letters so we know where to start the next one
    private float letterWidth = 4f; 

    public List<List<Vector3>> GetPathsForText(string text, Font font)
    {
        List<List<Vector3>> allStrokes = new List<List<Vector3>>();
        
        if (string.IsNullOrEmpty(text)) return allStrokes;

        string cleanText = text.ToUpper(); 
        float currentXOffset = 0f;

        foreach (char c in cleanText)
        {
            if (c == ' ')
            {
                currentXOffset += letterWidth;
                continue;
            }

            List<List<Vector3>> letterStrokes = GetLetterStrokes(c);
            
            foreach (var stroke in letterStrokes)
            {
                allStrokes.Add(ProcessPoints(stroke, currentXOffset));
            }

            currentXOffset += letterWidth + (spacing * 0.5f);
        }

        return allStrokes;
    }

    private List<Vector3> ProcessPoints(List<Vector3> rawPoints, float xOffset)
    {
        List<Vector3> processed = new List<Vector3>();
        foreach (Vector3 p in rawPoints)
        {
            float x = (p.x + xOffset) * scale;
            float z = p.z * scale; 
            Vector3 worldPos = transform.TransformPoint(new Vector3(x, 0, z) + offset);
            processed.Add(worldPos);
        }
        return processed;
    }

    // --- ALPHABET DEFINITIONS ---
    // Coordinates are roughly on a 4x5 grid (Width 4, Height 5)
    private List<List<Vector3>> GetLetterStrokes(char letter)
    {
        List<List<Vector3>> strokes = new List<List<Vector3>>();

        switch (letter)
        {
            case 'A':
                strokes.Add(Stroke(0,0, 2,5)); 
                strokes.Add(Stroke(2,5, 4,0)); 
                strokes.Add(Stroke(1,2.5f, 3,2.5f)); 
                break;
            case 'B':
                strokes.Add(Stroke(0,0, 0,5)); // Spine
                // Rounder Top Loop
                strokes.Add(CreateArc(0, 3.75f, 2.5f, 1.25f, 90, -90));
                // Rounder Bottom Loop
                strokes.Add(CreateArc(0, 1.25f, 3.0f, 1.25f, 90, -90));
                break;
            case 'C':
                // Full circular C
                strokes.Add(CreateArc(2.5f, 2.5f, 2.5f, 2.5f, 45, 315));
                break;
            case 'D':
                strokes.Add(Stroke(0,0, 0,5)); // Spine
                // Big Round D
                strokes.Add(CreateArc(0, 2.5f, 3.5f, 2.5f, 90, -90));
                break;
            case 'E':
                strokes.Add(Stroke(4,5, 0,5, 0,0, 4,0)); 
                strokes.Add(Stroke(0,2.5f, 3,2.5f)); 
                break;
            case 'F':
                strokes.Add(Stroke(0,0, 0,5, 4,5)); 
                strokes.Add(Stroke(0,2.5f, 3,2.5f)); 
                break;
            case 'G':
                // FIX FOR G: Start at top-right (45 deg), go CCW to right-middle (0 deg/360)
                strokes.Add(CreateArc(2.5f, 2.5f, 2.5f, 2.5f, 45, 360));
                // Horizontal Hook going INWARD: Connects perfectly at (5.0, 2.5)
                strokes.Add(Stroke(5.0f, 2.5f, 2.5f, 2.5f)); 
                break;
            case 'H':
                strokes.Add(Stroke(0,0, 0,5)); 
                strokes.Add(Stroke(4,0, 4,5)); 
                strokes.Add(Stroke(0,2.5f, 4,2.5f)); 
                break;
            case 'I':
                strokes.Add(Stroke(0,5, 4,5)); 
                strokes.Add(Stroke(2,5, 2,0)); 
                strokes.Add(Stroke(0,0, 4,0)); 
                break;
            case 'J':
                strokes.Add(Stroke(4,5, 4,1.5f)); 
                strokes.Add(CreateArc(2, 1.5f, 2f, 1.5f, 0, -180));
                break;
            case 'K':
                strokes.Add(Stroke(0,0, 0,5)); 
                strokes.Add(Stroke(4,5, 0,2.5f, 4,0)); 
                break;
            case 'L':
                strokes.Add(Stroke(0,5, 0,0, 4,0)); 
                break;
            case 'M':
                strokes.Add(Stroke(0,0, 0,5, 2,2.5f, 4,5, 4,0)); 
                break;
            case 'N':
                strokes.Add(Stroke(0,0, 0,5, 4,0, 4,5)); 
                break;
            case 'O':
                strokes.Add(CreateArc(2f, 2.5f, 2.5f, 2.5f, 90, 450));
                break;
            case 'P':
                strokes.Add(Stroke(0,0, 0,5)); // Spine
                strokes.Add(CreateArc(0, 3.75f, 3f, 1.25f, 90, -90));
                break;
            case 'Q':
                strokes.Add(CreateArc(2f, 2.5f, 2.5f, 2.5f, 90, 450));
                strokes.Add(Stroke(2.5f, 1.5f, 4.5f, -0.5f));
                break;
            case 'R':
                strokes.Add(Stroke(0,0, 0,5)); // Spine
                strokes.Add(CreateArc(0, 3.75f, 3f, 1.25f, 90, -90));
                strokes.Add(Stroke(1.5f, 2.5f, 4, 0)); 
                break;
            case 'S':
                // FIX FOR S: The two arcs must meet exactly in the middle.
                // Top Arc: Starts Top-Right(45), Goes CCW -> Top(90) -> Left(180) -> Middle(270)
                // Center(2, 3.75), Ends at (2, 2.5)
                strokes.Add(CreateArc(2f, 3.75f, 2f, 1.25f, 45, 270)); 
                
                // Bottom Arc: Starts Middle(90 relative to bottom center), Goes CW -> Right(0) -> Bottom(-90) -> Left(-135)
                // Center(2, 1.25), Starts at (2, 2.5)
                strokes.Add(CreateArc(2f, 1.25f, 2f, 1.25f, 90, -135)); 
                break;
            case 'T':
                strokes.Add(Stroke(0,5, 4,5)); 
                strokes.Add(Stroke(2,5, 2,0)); 
                break;
            case 'U':
                strokes.Add(Stroke(0,5, 0,1.5f)); 
                strokes.Add(CreateArc(2, 1.5f, 2f, 1.5f, 180, 360));
                strokes.Add(Stroke(4,1.5f, 4,5)); 
                break;
            case 'V':
                strokes.Add(Stroke(0,5, 2,0, 4,5)); 
                break;
            case 'W':
                strokes.Add(Stroke(0,5, 1,0, 2,2.5f, 3,0, 4,5)); 
                break;
            case 'X':
                strokes.Add(Stroke(0,0, 4,5)); 
                strokes.Add(Stroke(0,5, 4,0)); 
                break;
            case 'Y':
                strokes.Add(Stroke(0,5, 2,2.5f)); 
                strokes.Add(Stroke(4,5, 2,2.5f, 2,0)); 
                break;
            case 'Z':
                strokes.Add(Stroke(0,5, 4,5, 0,0, 4,0)); 
                break;
            // -- SHAPES --
            case '1': 
                strokes.Add(CreateArc(2f, 2.5f, 2f, 2f, 0, 360)); 
                break;
            case '2': 
                return CreateStar(); 
            case '3': 
                strokes.Add(Stroke(0,0, 0,5, 4,5, 4,0, 0,0));
                break;
        }
        return strokes;
    }

    // This helper creates continuous straight lines. The RobotWriter treats this as one stroke.
    List<Vector3> Stroke(params float[] coords)
    {
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < coords.Length; i += 2)
        {
            points.Add(new Vector3(coords[i], 0, coords[i+1]));
        }
        return points;
    }

    // UPDATED Helper to create smooth Arcs (Ellipses/Circles)
    // cx, cz: Center
    // w, h: Width Radius, Height Radius
    // startAng, endAng: Degrees (0=Right, 90=Up/Top, 180=Left, 270/-90=Bottom)
    List<Vector3> CreateArc(float cx, float cz, float w, float h, float startAng, float endAng, int res = 20)
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

    List<List<Vector3>> CreateStar()
    {
        List<Vector3> points = new List<Vector3>();
        // Hardcoded star points centered in 4x5 box
        points.Add(new Vector3(2, 5, 0)); 
        points.Add(new Vector3(2.5f, 3, 0));
        points.Add(new Vector3(4.5f, 3, 0)); 
        points.Add(new Vector3(3, 1.5f, 0));
        points.Add(new Vector3(4, -1, 0)); 
        points.Add(new Vector3(2, 0.5f, 0)); 
        points.Add(new Vector3(0, -1, 0)); 
        points.Add(new Vector3(1, 1.5f, 0));
        points.Add(new Vector3(-0.5f, 3, 0)); 
        points.Add(new Vector3(1.5f, 3, 0));
        points.Add(new Vector3(2, 5, 0)); 
        
        List<List<Vector3>> list = new List<List<Vector3>>();
        list.Add(points);
        return list;
    }
}