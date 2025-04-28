using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A component that handles robot movement along a calculated path.
/// This script works together with the AStarPathPlanner to move the robot along the calculated path.
/// </summary>
public class PathFollower : MonoBehaviour
{
    // Reference to the path planner
    public AStarPathPlanner pathPlanner;
    
    // Movement parameters
    public float moveSpeed = 2.0f;
    public float rotationSpeed = 120.0f;
    public float waypointReachedDistance = 0.3f;
    
    // Car controller reference
    public SimpleCarController carController;
    public float maxCarSpeed = 0.5f; // Maximum speed for car movement
    public float steeringFactor = 2.0f; // How sharply to turn
    
    // Path following state
    private List<Vector3> currentPath;
    private int currentWaypointIndex;
    private bool isFollowingPath = false;
    
    // Cached components
    private Rigidbody robotRigidbody;
    
    private void Start()
    {
        if (pathPlanner == null)
            pathPlanner = GetComponent<AStarPathPlanner>();
        
        if (carController == null)
            carController = GetComponent<SimpleCarController>();
            
        robotRigidbody = GetComponent<Rigidbody>();
        
        // If no rigidbody found, add one
        if (robotRigidbody == null)
        {
            robotRigidbody = gameObject.AddComponent<Rigidbody>();
            robotRigidbody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
            robotRigidbody.useGravity = true;
        }
    }
    
    private void Update()
    {
        // Debug visualization of the current target
        if (isFollowingPath && currentWaypointIndex < currentPath.Count)
        {
            Debug.DrawLine(transform.position, currentPath[currentWaypointIndex], Color.green);
        }
    }
    
    private void FixedUpdate()
    {
        // If we have a path to follow, follow it
        if (isFollowingPath)
        {
            FollowPathWithCar();
        }
        else
        {
            // Stop moving if we're not following a path
            if (carController != null)
            {
                carController.SetWheelSpeeds(0, 0);
            }
            else
            {
                robotRigidbody.velocity = Vector3.zero;
                robotRigidbody.angularVelocity = Vector3.zero;
            }
        }
    }
    
    /// <summary>
    /// Set a new grid path to follow
    /// </summary>
    public void SetPath(List<Vector2Int> gridPath)
    {
        // Convert grid path to world positions
        currentPath = pathPlanner.PathToWorld(gridPath);
        
        // Reset the following state
        currentWaypointIndex = 0;
        isFollowingPath = (currentPath != null && currentPath.Count > 0);
        
        Debug.Log("New path set with " + (currentPath?.Count ?? 0) + " waypoints.");
    }
    
    /// <summary>
    /// Calculate and set a path to the given grid target
    /// </summary>
    public bool SetTarget(Vector2Int gridTarget)
    {
        // Ensure the path planner exists
        if (pathPlanner == null || pathPlanner.gridMapper == null)
        {
            Debug.LogError("Path planner or grid mapper is null!");
            return false;
        }
        
        // Convert current position to grid coordinates
        Vector2Int currentGrid = pathPlanner.gridMapper.WorldToGrid(new Vector2(transform.position.x, transform.position.z));
        
        // Calculate a path to the target
        List<Vector2Int> path = pathPlanner.FindPath(currentGrid, gridTarget);
        
        // If we found a path, set it
        if (path != null && path.Count > 0)
        {
            SetPath(path);
            return true;
        }
        else
        {
            Debug.Log("No path found to target: " + gridTarget);
            return false;
        }
    }
    
    /// <summary>
    /// Calculate and set a path to the given world position
    /// </summary>
    public bool SetTargetWorld(Vector3 worldTarget)
    {
        // Convert world position to grid coordinates
        Vector2Int gridTarget = pathPlanner.gridMapper.WorldToGrid(new Vector2(worldTarget.x, worldTarget.z));
        
        // Set the target
        return SetTarget(gridTarget);
    }
    
    /// <summary>
    /// Stop following the current path
    /// </summary>
    public void StopFollowing()
    {
        isFollowingPath = false;
        
        if (carController != null)
        {
            carController.SetWheelSpeeds(0, 0);
        }
        else
        {
            robotRigidbody.velocity = Vector3.zero;
            robotRigidbody.angularVelocity = Vector3.zero;
        }
    }
    
    /// <summary>
    /// Follow the current path using car controller
    /// </summary>
    private void FollowPathWithCar()
    {
        // If we don't have a path or we've reached the end
        if (currentPath == null || currentWaypointIndex >= currentPath.Count)
        {
            StopFollowing();
            return;
        }
        
        // Get the current waypoint
        Vector3 waypoint = currentPath[currentWaypointIndex];
        
        // Calculate the direction to the waypoint
        Vector3 directionToWaypoint = waypoint - transform.position;
        directionToWaypoint.y = 0; // Ignore height difference
        float distanceToWaypoint = directionToWaypoint.magnitude;
        
        // Check if we've reached the waypoint
        if (distanceToWaypoint <= waypointReachedDistance)
        {
            // Move to the next waypoint
            currentWaypointIndex++;
            
            // If we've reached the end of the path
            if (currentWaypointIndex >= currentPath.Count)
            {
                // Stop following the path
                StopFollowing();
                Debug.Log("Path complete");
                return;
            }
        }
        
        // Use car controller if available
        if (carController != null)
        {
            // Calculate the angle between our current forward direction and the direction to the waypoint
            float angle = Vector3.SignedAngle(transform.forward, directionToWaypoint, Vector3.up);
            
            // Calculate wheel speeds based on the angle
            float forwardSpeed = Mathf.Clamp(maxCarSpeed * (1 - Mathf.Abs(angle) / 90.0f), 0.1f, maxCarSpeed);
            float steeringAmount = Mathf.Clamp(angle / 45.0f, -1.0f, 1.0f) * steeringFactor;
            
            // Apply differential steering:
            // - For right turns (negative angle), reduce right wheel speed
            // - For left turns (positive angle), reduce left wheel speed
            float leftWheelSpeed = forwardSpeed;
            float rightWheelSpeed = forwardSpeed;
            
            if (steeringAmount > 0) // Turn left
            {
                leftWheelSpeed -= steeringAmount * forwardSpeed;
            }
            else if (steeringAmount < 0) // Turn right
            {
                rightWheelSpeed += steeringAmount * forwardSpeed;
            }
            
            // Set the wheel speeds
            carController.SetWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
        }
        else
        {
            // Fallback to rigidbody movement if no car controller
            FollowPath();
        }
    }
    
    /// <summary>
    /// Follow the current path using rigidbody movement (fallback method)
    /// </summary>
    private void FollowPath()
    {
        // If we don't have a path or we've reached the end
        if (currentPath == null || currentWaypointIndex >= currentPath.Count)
        {
            StopFollowing();
            return;
        }
        
        // Get the current waypoint
        Vector3 waypoint = currentPath[currentWaypointIndex];
        
        // Calculate the distance to the waypoint
        Vector3 directionToWaypoint = waypoint - transform.position;
        directionToWaypoint.y = 0; // Ignore height difference
        float distanceToWaypoint = directionToWaypoint.magnitude;
        
        // Check if we've reached the waypoint
        if (distanceToWaypoint <= waypointReachedDistance)
        {
            // Move to the next waypoint
            currentWaypointIndex++;
            
            // If we've reached the end of the path
            if (currentWaypointIndex >= currentPath.Count)
            {
                // Stop following the path
                StopFollowing();
                Debug.Log("Path complete");
                return;
            }
        }
        
        // Calculate the direction to move in
        directionToWaypoint.Normalize();
        
        // Calculate the angle between our current forward direction and the direction to the waypoint
        float angle = Vector3.SignedAngle(transform.forward, directionToWaypoint, Vector3.up);
        
        // Rotate towards the waypoint
        float rotationAmount = Mathf.Clamp(angle, -rotationSpeed * Time.fixedDeltaTime, rotationSpeed * Time.fixedDeltaTime);
        transform.Rotate(Vector3.up, rotationAmount);
        
        // Move forward
        Vector3 moveDirection = transform.forward * moveSpeed;
        
        // Apply the movement
        robotRigidbody.velocity = moveDirection;
    }
    
    /// <summary>
    /// Check if the path follower is currently following a path
    /// </summary>
    public bool IsFollowingPath()
    {
        return isFollowingPath;
    }
    
    /// <summary>
    /// Get the remaining distance to the target
    /// </summary>
    public float GetRemainingDistance()
    {
        if (!isFollowingPath || currentPath == null || currentWaypointIndex >= currentPath.Count)
            return 0;
            
        // Calculate the distance to the current waypoint
        float distance = Vector3.Distance(transform.position, currentPath[currentWaypointIndex]);
        
        // Add the distances between remaining waypoints
        for (int i = currentWaypointIndex; i < currentPath.Count - 1; i++)
        {
            distance += Vector3.Distance(currentPath[i], currentPath[i + 1]);
        }
        
        return distance;
    }
}