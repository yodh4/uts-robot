using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PathFollower : MonoBehaviour
{
    public PathPlanner planner;
    public SimpleCarController carController;
    public float lookAheadDistance = 0.5f;
    public float maxDriveSpeed = 0.4f; // Increased a bit for better movement
    public bool debugMode = false;
    
    // Stuck detection
    private float lastMovementTime = 0f;
    private Vector3 lastPosition;
    private float stuckThreshold = 3.0f;
    private bool isStuck = false;
    private int stuckRecoveryPhase = 0;
    private float stuckRecoveryTimer = 0f;
    
    // Obstacle avoidance
    public float obstacleDetectionDistance = 1.5f; // How far to check for obstacles
    public float obstacleAvoidanceStrength = 1.5f; // How strongly to steer away
    public LayerMask obstacleLayer = -1; // Default to all layers
    private bool obstacleDetected = false;
    private Vector3 obstacleAvoidanceDirection = Vector3.zero;
    
    // Recovery states
    private enum RecoveryState { None, BackingUp, Turning }
    private RecoveryState currentRecovery = RecoveryState.None;
    private float recoveryStateTimer = 0f;
    private float backupTime = 1.5f; // How long to back up when stuck
    private float turnTime = 2.0f;   // How long to turn when stuck
    
    void Start()
    {
        lastPosition = transform.position;
        lastMovementTime = Time.time;
    }
    
    void FixedUpdate()
    {
        if (carController == null)
        {
            Debug.LogError("PathFollower: Missing carController!");
            return;
        }

        // Check for obstacles using raycasts (our "lidar")
        CheckForObstacles();
        
        // Check if we're in recovery mode
        if (currentRecovery != RecoveryState.None)
        {
            HandleRecoveryMovement();
            return;
        }

        // Check if we're stuck - only check every 10 frames
        if (Time.frameCount % 10 == 0)
        {
            CheckIfStuck();
        }

        // If stuck, start recovery sequence
        if (isStuck && currentRecovery == RecoveryState.None)
        {
            StartCoroutine(RecoverySequence());
            return;
        }

        // If there's an obstacle, handle obstacle avoidance
        if (obstacleDetected)
        {
            HandleObstacleAvoidance();
            return;
        }

        // Regular path following when no obstacles or not stuck
        FollowPath();
    }
    
    void CheckIfStuck()
    {
        float distanceMoved = Vector3.Distance(transform.position, lastPosition);
        if (distanceMoved > 0.01f)
        {
            // We've moved, reset stuck timer
            lastPosition = transform.position;
            lastMovementTime = Time.time;
            isStuck = false;
        }
        else if (Time.time - lastMovementTime > stuckThreshold)
        {
            isStuck = true;
            if (Time.frameCount % 60 == 0)
            {
                Debug.LogWarning("PathFollower: Robot appears to be stuck! Starting recovery sequence.");
            }
        }
    }
    
    void CheckForObstacles()
    {
        obstacleDetected = false;
        obstacleAvoidanceDirection = Vector3.zero;
        
        // Cast rays in a fan pattern in front of the robot
        int rayCount = 5;
        float fanAngle = 120f; // degrees
        
        for (int i = 0; i < rayCount; i++)
        {
            // Calculate ray angle (-fanAngle/2 to +fanAngle/2)
            float angle = -fanAngle/2 + fanAngle * i / (rayCount - 1);
            Vector3 rayDir = Quaternion.Euler(0, angle, 0) * transform.forward;
            
            // Adjust ray length based on angle (longer directly ahead)
            float rayLength = obstacleDetectionDistance * (1.0f - 0.3f * Mathf.Abs(angle) / (fanAngle/2));
            
            Ray ray = new Ray(transform.position + Vector3.up * 0.2f, rayDir);
            RaycastHit hit;
            
            // Debug rays if in debug mode
            if (debugMode)
            {
                Debug.DrawRay(ray.origin, ray.direction * rayLength, Color.yellow, 0.1f);
            }
            
            if (Physics.Raycast(ray, out hit, rayLength, obstacleLayer))
            {
                // Hit something - calculate avoidance direction
                obstacleDetected = true;
                
                // Calculate avoidance vector (perpendicular to hit direction)
                Vector3 avoidDir = Vector3.Cross(Vector3.up, rayDir).normalized;
                
                // Choose the better turning direction (left or right of obstacle)
                if (Vector3.Dot(avoidDir, transform.right) < 0)
                    avoidDir = -avoidDir;
                
                // Closer obstacles have stronger influence
                float weight = 1.0f - hit.distance / rayLength;
                obstacleAvoidanceDirection += avoidDir * weight * 2;
                
                if (debugMode)
                {
                    Debug.DrawRay(hit.point, avoidDir * weight, Color.red, 0.1f);
                }
            }
        }
        
        // Normalize the avoidance direction if we have one
        if (obstacleDetected && obstacleAvoidanceDirection.magnitude > 0)
        {
            obstacleAvoidanceDirection.Normalize();
        }
    }
    
    void HandleObstacleAvoidance()
    {
        // Calculate steering to avoid obstacle
        float steerDirection = Vector3.Dot(obstacleAvoidanceDirection, transform.right);
        float steer = Mathf.Clamp(steerDirection * obstacleAvoidanceStrength, -1f, 1f);
        
        // Slow down when avoiding obstacles
        float speed = maxDriveSpeed * 0.6f;
        
        // If very close to obstacle, back up instead
        Ray frontRay = new Ray(transform.position + Vector3.up * 0.2f, transform.forward);
        if (Physics.Raycast(frontRay, obstacleDetectionDistance * 0.5f, obstacleLayer))
        {
            // Too close - back up
            speed = -maxDriveSpeed * 0.4f;
        }
        
        if (debugMode)
        {
            Debug.Log($"Avoiding obstacle: steer={steer}, speed={speed}");
        }
        
        carController.SetInputs(speed, steer);
    }
    
    IEnumerator RecoverySequence()
    {
        // First back up
        currentRecovery = RecoveryState.BackingUp;
        float backupEndTime = Time.time + backupTime;
        
        Debug.Log("Recovery: Starting to back up");
        
        while (Time.time < backupEndTime)
        {
            // Back up at an angle to try to get unstuck
            carController.SetInputs(-maxDriveSpeed * 0.5f, 0.7f);
            yield return new WaitForFixedUpdate();
        }
        
        // Then turn
        currentRecovery = RecoveryState.Turning;
        float turnEndTime = Time.time + turnTime;
        
        Debug.Log("Recovery: Starting to turn");
        
        while (Time.time < turnEndTime)
        {
            // Turn sharply with a bit of forward motion
            carController.SetInputs(maxDriveSpeed * 0.3f, 1.0f);
            yield return new WaitForFixedUpdate();
        }
        
        // Reset stuck status and recovery mode
        isStuck = false;
        currentRecovery = RecoveryState.None;
        lastPosition = transform.position;
        lastMovementTime = Time.time;
        
        Debug.Log("Recovery: Completed recovery sequence");
    }
    
    void HandleRecoveryMovement()
    {
        // This is handled by the coroutine, but we need this method
        // to return early in FixedUpdate when in recovery mode
    }
    
    void FollowPath()
    {
        // Handle path following
        if (planner == null || planner.currentPath == null || planner.currentPath.Count < 2)
        {
            // Only log warning occasionally
            if (Time.frameCount % 60 == 0) 
            {
                Debug.LogWarning("PathFollower: No valid path available");
            }
            
            // Just move forward as fallback when no path
            carController.SetInputs(maxDriveSpeed, 0.0f);
            return;
        }

        // Get current position
        Vector3 robotPos = transform.position;
        List<Vector2> path = planner.currentPath;
        
        // Find closest point on path
        int closestIdx = 0;
        float minDist = float.MaxValue;
        
        for (int i = 0; i < path.Count; i++)
        {
            float dist = Vector2.Distance(new Vector2(robotPos.x, robotPos.z), path[i]);
            if (dist < minDist)
            {
                minDist = dist;
                closestIdx = i;
            }
        }
        
        // IMPROVED: Dynamic look-ahead based on speed and curvature
        float dynamicLookAhead = lookAheadDistance * (1.0f + carController.CurrentSpeed);
        
        // Find look-ahead point along path
        Vector3 lookAheadPoint = Vector3.zero;
        float distanceAccumulated = 0f;
        int targetIdx = closestIdx;
        
        // Walk forward from closest point until we reach our look-ahead distance
        for (int i = closestIdx; i < path.Count - 1; i++)
        {
            Vector2 currentPathPoint = path[i];
            Vector2 nextPathPoint = path[i + 1];
            float segmentLength = Vector2.Distance(currentPathPoint, nextPathPoint);
            
            if (distanceAccumulated + segmentLength > dynamicLookAhead)
            {
                // Our target is on this segment - interpolate to find exact point
                float remainingDistance = dynamicLookAhead - distanceAccumulated;
                float t = remainingDistance / segmentLength;
                Vector2 targetPoint2D = Vector2.Lerp(currentPathPoint, nextPathPoint, t);
                lookAheadPoint = new Vector3(targetPoint2D.x, 0, targetPoint2D.y);
                targetIdx = i;
                break;
            }
            
            distanceAccumulated += segmentLength;
            
            // If we reached the end of path without hitting our look-ahead distance
            if (i == path.Count - 2)
            {
                lookAheadPoint = new Vector3(nextPathPoint.x, 0, nextPathPoint.y);
                targetIdx = i + 1;
            }
        }
        
        // If we're close to the end of the path, just use the last point
        if (targetIdx >= path.Count - 1 && Vector3.Distance(robotPos, new Vector3(path[path.Count - 1].x, 0, path[path.Count - 1].y)) < 0.5f)
        {
            // We've reached destination, stop moving
            carController.SetInputs(0f, 0f);
            return;
        }
        
        // IMPROVED: Calculate direction to target
        Vector3 dirToTarget = lookAheadPoint - robotPos;
        dirToTarget.y = 0; // Ensure we're only considering horizontal plane
        
        // IMPROVED: Calculate steering angle
        float angle = Vector3.SignedAngle(transform.forward, dirToTarget, Vector3.up);
        
        // IMPROVED: Convert angle to steering with a more responsive curve
        // Use a smoother response curve for more natural steering
        float normalizedAngle = Mathf.Clamp(angle / 45f, -1f, 1f); // Normalize to -1,1 range
        float steer = Mathf.Sign(normalizedAngle) * (1.0f - Mathf.Pow(1.0f - Mathf.Abs(normalizedAngle), 2.0f));
        
        // IMPROVED: Better speed control in turns
        // Calculate path curvature (approximation based on upcoming points)
        float curvature = 0f;
        if (targetIdx < path.Count - 2)
        {
            Vector2 p1 = path[targetIdx];
            Vector2 p2 = path[targetIdx + 1];
            Vector2 p3 = path[targetIdx + 2];
            
            Vector2 dir1 = (p2 - p1).normalized;
            Vector2 dir2 = (p3 - p2).normalized;
            
            // Dot product gives cosine of angle between segments
            float dot = Vector2.Dot(dir1, dir2);
            curvature = 1.0f - Mathf.Clamp01(dot); // 0 = straight line, 1 = sharp turn
        }
        
        // Speed based on turn angle and path curvature - slow down in sharp turns
        float speed = maxDriveSpeed * (1.0f - Mathf.Abs(steer) * 0.3f - curvature * 0.5f);
        speed = Mathf.Max(0.2f, speed); // Ensure minimum speed
        
        // Only log debugging info occasionally if debug mode is enabled
        if (debugMode && Time.frameCount % 30 == 0)
        {
            Debug.Log($"[PathFollower] Target: {lookAheadPoint}, Angle: {angle}, Steer: {steer}, Speed: {speed}, Curvature: {curvature}");
            Debug.DrawLine(robotPos, lookAheadPoint, Color.green, 0.1f);
        }
        
        // Apply inputs
        carController.SetInputs(speed, steer);
    }
    
    void OnDrawGizmos()
    {
        if (planner == null || planner.currentPath == null || planner.currentPath.Count < 2) return;

        // Draw the path
        Gizmos.color = Color.blue;
        for (int i = 0; i < planner.currentPath.Count - 1; i++)
        {
            Vector3 p1 = new Vector3(planner.currentPath[i].x, 0.1f, planner.currentPath[i].y);
            Vector3 p2 = new Vector3(planner.currentPath[i+1].x, 0.1f, planner.currentPath[i+1].y);
            Gizmos.DrawLine(p1, p2);
        }
        
        // Try to draw the target point if possible
        if (planner.currentPath != null && planner.currentPath.Count > 0)
        {
            Vector3 robotPos = transform.position;
            List<Vector2> path = planner.currentPath;
            
            int closestIdx = 0;
            float minDist = float.MaxValue;
            
            for (int i = 0; i < path.Count; i++)
            {
                float dist = Vector2.Distance(new Vector2(robotPos.x, robotPos.z), path[i]);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestIdx = i;
                }
            }
            
            int targetIdx = Mathf.Min(closestIdx + 1, path.Count - 1);
            Vector3 targetPoint = new Vector3(path[targetIdx].x, 0.3f, path[targetIdx].y);
            
            // Draw big yellow sphere at target point
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(targetPoint, 0.25f);
            
            // Draw line from robot to target
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(transform.position + Vector3.up * 0.3f, targetPoint);
        }
    }
}
