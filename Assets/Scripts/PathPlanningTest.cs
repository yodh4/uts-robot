using UnityEngine;

/// <summary>
/// A simple test script to demonstrate the integrated path planning and following system.
/// </summary>
public class PathPlanningTest : MonoBehaviour
{
    // References to required components
    public AStarPathPlanner pathPlanner;
    public PathFollower pathFollower;
    
    // Camera reference
    public Camera mainCamera;
    
    // Debug options
    public bool showDebugInfo = true;
    
    // Target position for testing
    private Vector3 targetPosition;
    private bool hasTarget = false;
    
    // UI elements for displaying status
    private GUIStyle guiStyle = new GUIStyle();
    
    private void Start()
    {
        // Find components if not assigned
        if (pathPlanner == null)
            pathPlanner = GetComponent<AStarPathPlanner>();
        
        if (pathFollower == null)
            pathFollower = GetComponent<PathFollower>();
            
        // Find the main camera if not set
        if (mainCamera == null)
            mainCamera = Camera.main;
            
        // Log error if no camera found
        if (mainCamera == null)
            Debug.LogError("No main camera found. Make sure your camera has the 'MainCamera' tag or assign it directly.");
        
        // Set up GUI style
        guiStyle.normal.textColor = Color.white;
        guiStyle.fontSize = 16;
        guiStyle.padding = new RectOffset(10, 10, 10, 10);
        
        // Generate cost map on start
        if (pathPlanner != null && pathPlanner.gridMapper != null)
        {
            pathPlanner.GenerateCostMap();
        }
        else
        {
            Debug.LogError("Path planner or grid mapper is missing. Make sure they're properly set up.");
        }
    }
    
    private void Update()
    {
        // Test input: Click to set target
        if (Input.GetMouseButtonDown(0) && mainCamera != null)
        {
            try
            {
                Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
                RaycastHit hit;
                
                // Log the ray for debugging
                Debug.DrawRay(ray.origin, ray.direction * 100, Color.red, 1.0f);
                
                // Cast ray against terrain or ground plane
                if (Physics.Raycast(ray, out hit))
                {
                    // Set the target position
                    SetTargetPosition(hit.point);
                }
                else
                {
                    Debug.Log("Click did not hit anything. Make sure there are colliders in your scene.");
                    
                    // Alternative: Project the ray onto a plane at y=0 if nothing is hit
                    Plane groundPlane = new Plane(Vector3.up, Vector3.zero);
                    float rayDistance;
                    
                    if (groundPlane.Raycast(ray, out rayDistance))
                    {
                        Vector3 hitPoint = ray.GetPoint(rayDistance);
                        SetTargetPosition(hitPoint);
                    }
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError("Error in click handling: " + e.Message);
            }
        }
        
        // Debug visualization of the target
        if (showDebugInfo && hasTarget)
        {
            Debug.DrawLine(
                targetPosition, 
                targetPosition + Vector3.up * 2.0f, 
                Color.magenta);
        }
    }
    
    /// <summary>
    /// Set a new target position in the world
    /// </summary>
    public void SetTargetPosition(Vector3 position)
    {
        if (pathFollower == null)
        {
            Debug.LogError("Path follower is not assigned!");
            return;
        }
        
        if (pathPlanner == null || pathPlanner.gridMapper == null)
        {
            Debug.LogError("Path planner or grid mapper is not set up correctly!");
            return;
        }
        
        // Store target position for visualization
        targetPosition = position;
        hasTarget = true;
        
        Debug.Log("Setting target position: " + position);
        
        // Tell the path follower to navigate to the target
        bool pathFound = pathFollower.SetTargetWorld(position);
        
        if (pathFound)
        {
            Debug.Log("Path found to target position");
        }
        else
        {
            Debug.LogWarning("No path found to target position");
        }
    }
    
    /// <summary>
    /// Draw status information on the screen
    /// </summary>
    private void OnGUI()
    {
        if (!showDebugInfo)
            return;
        
        string status = "Path Planning Test\n";
        status += "Click anywhere on the ground to set a target\n";
        
        if (hasTarget)
        {
            status += "Target Position: " + targetPosition + "\n";
            
            if (pathFollower != null)
            {
                if (pathFollower.IsFollowingPath())
                {
                    status += "Status: Following path\n";
                    status += "Remaining distance: " + pathFollower.GetRemainingDistance().ToString("F2") + "m";
                }
                else
                {
                    status += "Status: Idle (no path)";
                }
            }
        }
        else
        {
            status += "Status: No target set";
        }
        
        GUI.Box(new Rect(10, 10, 300, 100), "");
        GUI.Label(new Rect(10, 10, 300, 100), status, guiStyle);
    }
}