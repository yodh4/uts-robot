using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(FrontierSearch))]
public class FrontierExplorer : MonoBehaviour
{
    // References to required components
    public OccupancyGridMapper gridMapper;
    public PoseEstimator poseEstimator;
    public SimpleCarController carController;
    public AStarPathPlanner pathPlanner; // Added reference to A* path planner
    private FrontierSearch frontierSearch;

    // Path planning and navigation parameters
    public float updateRate = 1.0f;           // How often to recalculate frontiers (seconds)
    public float goalReachedThreshold = 1.5f; // Distance to consider a goal reached (meters)
    public bool enableExploration = true;     // Whether to automatically explore
    public float waypointReachedThreshold = 0.5f; // Distance to consider a waypoint reached (meters)
    
    // Frontier selection weights
    [Range(0f, 10f)]
    public float distanceWeight = 2.0f;       // Weight for distance to frontier
    [Range(0f, 10f)]
    public float sizeWeight = 1.0f;           // Weight for frontier size
    
    // Increased for more patience in finding frontiers
    public int maxFailedAttempts = 10;

    // Status and debugging
    public bool showDebugInfo = true;
    public Color pathColor = Color.yellow;
    public Color robotPosColor = new Color(1, 0.5f, 0, 1); // Orange
    
    // Internal state variables
    private float lastUpdateTime = 0f;
    private FrontierSearch.Frontier currentTarget;
    private List<Vector3> currentPath;
    private int currentPathIndex = 0;
    private int failedNavigationAttempts = 0;
    private bool isNavigating = false;
    private bool pathNeedsRecalculation = false;
    private float pathRecalculationTimer = 0f;
    private float pathRecalculationInterval = 5f; // Recalculate path every 5 seconds if needed

    private void Start()
    {
        // Get required components
        if (gridMapper == null)
            gridMapper = GetComponent<OccupancyGridMapper>();
            
        if (poseEstimator == null)
            poseEstimator = GetComponent<PoseEstimator>();
            
        if (carController == null)
            carController = GetComponent<SimpleCarController>();
        
        if (pathPlanner == null)
            pathPlanner = GetComponent<AStarPathPlanner>();
            
        frontierSearch = GetComponent<FrontierSearch>();
        
        // Initialize path
        currentPath = new List<Vector3>();
        
        // Set debug visualization on for the frontier search component
        if (frontierSearch != null)
            frontierSearch.showDebugInfo = showDebugInfo;
            
        // Debug setup
        if (carController == null)
        {
            Debug.LogError("CarController not found! Car won't move.");
        }
        
        if (pathPlanner == null)
        {
            Debug.LogError("AStarPathPlanner not found! Car won't be able to avoid obstacles.");
        }
        else
        {
            // Set reference to the grid mapper in the path planner if needed
            if (pathPlanner.gridMapper == null)
                pathPlanner.gridMapper = gridMapper;
        }
    }

    private void Update()
    {
        if (!enableExploration) return;
        
        // Check if we've reached the current goal
        if (isNavigating && currentTarget != null && 
            Vector2Distance(poseEstimator.position, new Vector2(currentTarget.centroid.x, currentTarget.centroid.z)) < goalReachedThreshold)
        {
            // We've reached the target frontier
            Debug.Log("Reached target frontier!");
            isNavigating = false;
            currentTarget = null;
            currentPath.Clear();
        }
        
        // Update frontier detection periodically
        if (Time.time - lastUpdateTime > updateRate)
        {
            lastUpdateTime = Time.time;
            DetectAndSelectFrontier();
        }
        
        // Check if path needs recalculation (periodically)
        if (isNavigating && pathNeedsRecalculation)
        {
            pathRecalculationTimer += Time.deltaTime;
            if (pathRecalculationTimer >= pathRecalculationInterval)
            {
                pathRecalculationTimer = 0f;
                RecalculatePath();
            }
        }
        
        // If we're navigating, follow the path
        if (isNavigating && currentPath != null && currentPath.Count > 0)
        {
            FollowPath();
        }
    }
    
    /// <summary>
    /// Calculate distance between two 2D points
    /// </summary>
    private float Vector2Distance(Vector2 a, Vector2 b) 
    {
        return Mathf.Sqrt(Mathf.Pow(a.x - b.x, 2) + Mathf.Pow(a.y - b.y, 2));
    }
    
    /// <summary>
    /// Detect frontiers and select the best one to explore
    /// </summary>
    private void DetectAndSelectFrontier()
    {
        // If we're already navigating to a frontier, don't change course
        if (isNavigating && currentTarget != null) 
            return;
            
        // Get the robot's current position as grid coordinates
        Vector2Int robotGridPosition = gridMapper.WorldToGrid(poseEstimator.position);
        
        Debug.Log($"Searching for frontiers from grid position: {robotGridPosition}, world position: {poseEstimator.position}");
        
        // Search for frontiers
        FrontierSearch.FrontierCollection frontiers = frontierSearch.SearchFrontiers(robotGridPosition);
        
        if (frontiers == null || frontiers.frontiers.Count == 0)
        {
            Debug.Log($"No frontiers found. Failed attempts: {failedNavigationAttempts+1}/{maxFailedAttempts}");
            isNavigating = false;
            failedNavigationAttempts++;
            
            if (failedNavigationAttempts >= maxFailedAttempts)
            {
                Debug.Log("Exploration complete! No more frontiers found after multiple attempts.");
                enableExploration = false;
            }
            return;
        }
        
        failedNavigationAttempts = 0;
        
        Debug.Log($"Found {frontiers.frontiers.Count} potential frontiers to explore");
        
        // Select the best frontier
        currentTarget = SelectBestFrontier(frontiers.frontiers);
        
        if (currentTarget != null)
        {
            // Plan path to the frontier
            PlanPath(new Vector3(poseEstimator.position.x, 0, poseEstimator.position.y), currentTarget.centroid);
            isNavigating = true;
        }
    }
    
    /// <summary>
    /// Select the best frontier based on distance and size
    /// </summary>
    /// <param name="frontiers">List of detected frontiers</param>
    /// <returns>The best frontier to explore</returns>
    private FrontierSearch.Frontier SelectBestFrontier(List<FrontierSearch.Frontier> frontiers)
    {
        if (frontiers.Count == 0)
            return null;
            
        FrontierSearch.Frontier bestFrontier = null;
        float bestScore = float.MinValue;
        
        foreach (FrontierSearch.Frontier frontier in frontiers)
        {
            // Use 2D distance for proper comparison with PoseEstimator's 2D position
            float distance = Vector2Distance(poseEstimator.position, 
                                            new Vector2(frontier.centroid.x, frontier.centroid.z));
            
            // Calculate distance score (closer is better, so we invert it)
            float distanceScore = 1f / (1f + distance);
            
            // Calculate size score (bigger is better)
            float sizeScore = frontier.size / 100f;  // Normalize size to [0,1] range assuming max size ~100
            
            // Calculate total score
            float score = (distanceWeight * distanceScore) + (sizeWeight * sizeScore);
            
            if (score > bestScore)
            {
                bestScore = score;
                bestFrontier = frontier;
            }
            
            Debug.Log($"Frontier at {frontier.centroid}, Size: {frontier.size}, Dist: {distance:F2}, Score: {score:F2}");
        }
        
        Debug.Log($"Selected frontier with size {bestFrontier.size} at {bestFrontier.centroid}");
        
        return bestFrontier;
    }
    
    /// <summary>
    /// Plan a path from start to goal
    /// </summary>
    /// <param name="start">Start position in world coordinates</param>
    /// <param name="goal">Goal position in world coordinates</param>
    private void PlanPath(Vector3 start, Vector3 goal)
    {
        // Clear the current path
        currentPath.Clear();
        currentPathIndex = 0;
        
        // Check if we have a path planner
        if (pathPlanner == null)
        {
            Debug.LogWarning("No A* path planner available, using direct path instead");
            // Fallback to direct path
            currentPath.Add(start);
            currentPath.Add(goal);
            return;
        }
        
        Debug.Log($"Planning A* path from {start} to frontier at {goal}");
        
        // Convert world coordinates to grid coordinates
        Vector2Int startCell = gridMapper.WorldToGrid(new Vector2(start.x, start.z));
        Vector2Int goalCell = gridMapper.WorldToGrid(new Vector2(goal.x, goal.z));
        
        // Ensure the cost map is generated
        if (!pathPlanner.costMapGenerated)
        {
            pathPlanner.GenerateCostMap();
        }
        
        // Find path using A* algorithm
        List<Vector2Int> gridPath = pathPlanner.FindPath(startCell, goalCell);
        
        // If path found, convert to world coordinates
        if (gridPath != null && gridPath.Count > 0)
        {
            List<Vector3> worldPath = pathPlanner.PathToWorld(gridPath);
            
            if (worldPath != null && worldPath.Count > 0)
            {
                currentPath = worldPath;
                Debug.Log($"A* path found with {currentPath.Count} waypoints");
                
                // Reset path recalculation flag
                pathNeedsRecalculation = false;
                pathRecalculationTimer = 0f;
            }
            else
            {
                Debug.LogWarning("Failed to convert grid path to world coordinates");
                // Fallback to direct path
                currentPath.Add(start);
                currentPath.Add(goal);
            }
        }
        else
        {
            Debug.LogWarning($"No path found from {startCell} to {goalCell}! Using direct path as fallback.");
            // Fallback to direct path
            currentPath.Add(start);
            currentPath.Add(goal);
            
            // Make sure we try to recalculate the path later
            pathNeedsRecalculation = true;
        }
    }
    
    /// <summary>
    /// Recalculate the path to the current target if needed
    /// </summary>
    private void RecalculatePath()
    {
        if (currentTarget == null) return;
        
        // Get current position
        Vector3 currentPos = new Vector3(poseEstimator.position.x, 0, poseEstimator.position.y);
        
        Debug.Log("Recalculating path due to obstacles or failed navigation");
        
        // Re-plan path from current position to target
        PlanPath(currentPos, currentTarget.centroid);
        
        // Reset path index
        currentPathIndex = 0;
        
        // Reset recalculation flag
        pathNeedsRecalculation = false;
    }
    
    /// <summary>
    /// Follow the current path
    /// </summary>
    private void FollowPath()
    {
        if (currentPathIndex >= currentPath.Count - 1)
            return;
            
        Vector3 currentTarget = currentPath[currentPathIndex + 1];
        
        // Get 2D direction to target (X,Z plane in Unity)
        Vector2 currentPos2D = poseEstimator.position;
        Vector2 target2D = new Vector2(currentTarget.x, currentTarget.z);
        Vector2 direction2D = target2D - currentPos2D;
        
        // Skip if we're too close to the target (prevents jittering)
        float distanceToTarget = direction2D.magnitude;
        if (distanceToTarget < 0.1f)
        {
            // Stop briefly at waypoint
            carController.SetWheelSpeeds(0, 0);
            
            // Move to next waypoint
            currentPathIndex++;
            Debug.Log($"Very close to waypoint, moving to next one. {currentPathIndex}/{currentPath.Count-1}");
            return;
        }
        
        // Calculate desired heading (in degrees)
        float targetAngle = Mathf.Atan2(direction2D.y, direction2D.x) * Mathf.Rad2Deg;
        
        // Normalize angle to 0-360 range
        if (targetAngle < 0)
            targetAngle += 360;
            
        // Calculate difference between current heading and desired heading
        float currentAngle = poseEstimator.heading;
        float angleDifference = Mathf.DeltaAngle(currentAngle, targetAngle);
        
        // Log navigation data occasionally
        if (Time.frameCount % 30 == 0)
        {
            Debug.Log($"Navigation: Current pos: {currentPos2D}, Target: {target2D}, " +
                    $"Current angle: {currentAngle}, Target angle: {targetAngle}, " +
                    $"Angle diff: {angleDifference}, Distance: {distanceToTarget:F2}");
        }
        
        // Determine if we need to make sharp turn to align with waypoint
        bool needsSharpTurn = Mathf.Abs(angleDifference) > 60f;
        
        // Adjust steering behavior based on distance to target
        float steerFactor = needsSharpTurn ? 15f : 20f; // More aggressive steering for sharp turns
        float steer = Mathf.Clamp(angleDifference / steerFactor, -1.8f, 1.8f);
        
        // Calculate forward speed based on alignment with target and distance
        float baseSpeed = 0.5f;
        float speed = baseSpeed;
        
        // Slow down when approaching waypoint for smoother navigation
        if (distanceToTarget < 2.0f)
        {
            speed *= 0.8f;
        }
        
        // Slow down when turning sharply
        if (Mathf.Abs(angleDifference) > 30f)
        {
            speed = Mathf.Max(0.15f, 0.5f - (Mathf.Abs(angleDifference) - 30f) / 60f);
        }
        
        // When making very sharp turns, prioritize turning over forward motion
        if (needsSharpTurn)
        {
            speed *= 0.5f; // Reduce speed significantly for sharp turns
        }
        
        // Minimum speed for reliable movement
        if (speed < 0.15f) speed = 0.15f;
        
        // Apply controls to the car controller
        if (carController != null)
        {
            // Convert the single speed and steer values to left and right wheel speeds
            float leftSpeed = speed;
            float rightSpeed = speed;
            
            // Apply differential steering (tank-like control)
            if (steer < 0) // Turn right
            {
                leftSpeed = speed;
                rightSpeed = speed * (1 + steer * 1.5f);
                if (rightSpeed < -0.3f) rightSpeed = -0.3f;
            }
            else if (steer > 0) // Turn left
            {
                leftSpeed = speed * (1 - steer * 1.5f);
                rightSpeed = speed;
                if (leftSpeed < -0.3f) leftSpeed = -0.3f;
            }
            
            // Debug wheel speeds
            if (Time.frameCount % 30 == 0)
            {
                Debug.Log($"Setting wheel speeds: Left={leftSpeed:F2}, Right={rightSpeed:F2}, Steer={steer:F2}, Speed={speed:F2}");
            }
            
            // Ensure minimum speed difference for turning
            float speedDiff = Mathf.Abs(leftSpeed - rightSpeed);
            if (Mathf.Abs(angleDifference) > 10f && speedDiff < 0.15f)
            {
                // Force more aggressive turning
                if (steer < 0) rightSpeed -= 0.15f;
                else leftSpeed -= 0.15f;
                
                Debug.Log($"Forcing turn: Left={leftSpeed:F2}, Right={rightSpeed:F2}");
            }
            
            // For very sharp turns (over 90 degrees), implement a spot turn by reversing one wheel
            if (Mathf.Abs(angleDifference) > 90f)
            {
                if (steer < 0) // Sharp right turn
                {
                    leftSpeed = 0.4f;
                    rightSpeed = -0.4f;
                }
                else // Sharp left turn
                {
                    leftSpeed = -0.4f;
                    rightSpeed = 0.4f;
                }
                Debug.Log($"Executing spot turn: Left={leftSpeed:F2}, Right={rightSpeed:F2}");
            }
            
            // Set wheel speeds
            carController.SetWheelSpeeds(leftSpeed, rightSpeed);
        }
        else
        {
            Debug.LogError("Car controller is null! Cannot move.");
        }
        
        // Check if we reached the current waypoint - use smaller threshold for intermediate waypoints
        float checkThreshold = waypointReachedThreshold;
        if (currentPathIndex < currentPath.Count - 2)
        {
            // Use smaller threshold for intermediate waypoints
            checkThreshold = waypointReachedThreshold;
        }
        else
        {
            // Use larger threshold for final destination
            checkThreshold = goalReachedThreshold;
        }
        
        // Check if reached waypoint
        if (Vector2Distance(currentPos2D, target2D) < checkThreshold)
        {
            currentPathIndex++;
            Debug.Log($"Reached waypoint {currentPathIndex-1}! Moving to next point. {currentPathIndex}/{currentPath.Count-1}");
            
            // If we reached the end of the path, stop navigating
            if (currentPathIndex >= currentPath.Count - 1)
            {
                Debug.Log("Reached end of path!");
            }
        }
        
        // Check for stuck state (not making progress toward waypoint)
        // Add collision detection and path recalculation
        if (IsStuckOrColliding())
        {
            pathNeedsRecalculation = true;
            Debug.Log("Detected stuck state or collision - flagging for path recalculation");
        }
    }
    
    /// <summary>
    /// Check if the robot is stuck or colliding with obstacles
    /// </summary>
    private bool IsStuckOrColliding()
    {
        // Simple implementation - can be expanded with more sophisticated checks
        
        // Check if any wheel is not in contact with ground
        if (carController != null)
        {
            bool allWheelsGrounded = 
                carController.IsWheelGrounded(carController.frontLeftWheel) &&
                carController.IsWheelGrounded(carController.frontRightWheel) &&
                carController.IsWheelGrounded(carController.rearLeftWheel) &&
                carController.IsWheelGrounded(carController.rearRightWheel);
                
            if (!allWheelsGrounded)
            {
                Debug.Log("Not all wheels are grounded - possible collision");
                return true;
            }
        }
        
        // Future enhancements:
        // 1. Track position over time to detect if robot isn't moving
        // 2. Check for obstacles in front using raycasts
        // 3. Monitor velocity to detect if robot is pushing against something
        
        return false;
    }
    
    /// <summary>
    /// Draw debug visualizations in the scene view
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!showDebugInfo)
            return;
        
        // Draw robot position and grid cell
        if (Application.isPlaying && gridMapper != null && poseEstimator != null)
        {
            // Draw sphere at robot position
            Vector3 robotPos3D = new Vector3(poseEstimator.position.x, 0.1f, poseEstimator.position.y);
            Gizmos.color = robotPosColor;
            Gizmos.DrawSphere(robotPos3D, 0.3f);
            
            // Draw direction indicator
            float headingRad = poseEstimator.heading * Mathf.Deg2Rad;
            Vector3 directionVector = new Vector3(
                Mathf.Cos(headingRad), 
                0, 
                Mathf.Sin(headingRad)
            ) * 0.5f;
            
            Gizmos.DrawLine(robotPos3D, robotPos3D + directionVector);
            
            // Draw the grid cell the robot is in
            Vector2Int robotCell = gridMapper.WorldToGrid(poseEstimator.position);
            Vector2 cellWorldPos = gridMapper.GridToWorld(robotCell);
            Vector3 cellPos3D = new Vector3(cellWorldPos.x, 0, cellWorldPos.y);
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireCube(cellPos3D, new Vector3(gridMapper.cellSize, 0.1f, gridMapper.cellSize));
        }
        
        // Draw the current path
        if (currentPath != null && currentPath.Count > 1)
        {
            Gizmos.color = pathColor;
            
            for (int i = 0; i < currentPath.Count - 1; i++)
            {
                Gizmos.DrawLine(currentPath[i], currentPath[i + 1]);
            }
        }
        
        // Draw current target
        if (currentTarget != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(currentTarget.centroid, 0.5f);
        }
    }
    
    /// <summary>
    /// Manually force exploration to start or restart
    /// </summary>
    public void StartExploration()
    {
        enableExploration = true;
        isNavigating = false;
        currentTarget = null;
        currentPath.Clear();
        failedNavigationAttempts = 0;
        
        // Force immediate update
        lastUpdateTime = -updateRate;
        
        Debug.Log("Exploration started!");
    }
    
    /// <summary>
    /// Stop exploration
    /// </summary>
    public void StopExploration()
    {
        enableExploration = false;
        isNavigating = false;
        
        // Stop the robot
        if (carController != null)
        {
            carController.SetWheelSpeeds(0, 0);
        }
        
        Debug.Log("Exploration stopped.");
    }
}