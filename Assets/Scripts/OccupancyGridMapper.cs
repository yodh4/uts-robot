using UnityEngine;

public class OccupancyGridMapper : MonoBehaviour
{
    // Core variables
    public LidarSimulator lidar;
    public PoseEstimator pose;
    public int gridWidth = 100;
    public int gridHeight = 100;
    public float cellSize = 0.2f;
    public int[,] occupancyGrid;
    
    // Grid origin in world coordinates
    private Vector2 gridWorldOrigin;
    public float heightOffset = 0.1f; // Height for visualization
    
    // Probability configuration
    [Range(0, 100)]
    public int occupiedThreshold = 80;  // Probability threshold to consider a cell occupied
    [Range(0, 100)]
    public int freeThreshold = 20;      // Probability threshold to consider a cell free
    [Range(1, 10)]
    public int occupiedUpdateValue = 5;  // Value to add when updating occupied cell
    [Range(1, 10)]
    public int freeUpdateValue = 1;     // Value to subtract when updating free cell
    
    // Debug options
    public bool showDebugInfo = true;
    public bool forceGridVisibility = true; // Force grid visibility always
    
    private void Start()
    {
        // Check for lidar and pose component references
        if (lidar == null)
        {
            Debug.LogError("LidarSimulator reference is missing on OccupancyGridMapper!");
            lidar = FindObjectOfType<LidarSimulator>();
            if (lidar) Debug.Log("Found and assigned LidarSimulator automatically.");
        }
        
        if (pose == null)
        {
            Debug.LogError("PoseEstimator reference is missing on OccupancyGridMapper!");
            pose = FindObjectOfType<PoseEstimator>();
            if (pose) Debug.Log("Found and assigned PoseEstimator automatically.");
        }
        
        // Initialize grid with 50% probability (unknown)
        occupancyGrid = new int[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                occupancyGrid[x, y] = 50; // 50% probability - unknown
            }
        }
        
        // Set grid origin based on initial pose position
        UpdateGridOrigin();
        
        // Debug logging
        Debug.Log($"OccupancyGridMapper initialized: Grid size: {gridWidth}x{gridHeight}, Cell size: {cellSize}, Origin: {gridWorldOrigin}");
    }

    private void Update()
    {
        if (lidar == null || pose == null)
        {
            Debug.LogWarning("LidarSimulator or PoseEstimator reference missing. Grid won't update.");
            return;
        }
        
        // Update grid origin to follow the robot
        UpdateGridOrigin();
        
        // Update grid with LIDAR data
        UpdateGrid();
    }
    
    // Calculate grid world origin (center of grid in world space)
    private void UpdateGridOrigin()
    {
        if (pose != null)
        {
            // Set grid center at current pose position
            gridWorldOrigin = pose.position;
            if (showDebugInfo && Time.frameCount % 60 == 0)
            {
                Debug.Log($"Grid origin updated to: {gridWorldOrigin}");
            }
        }
    }

    void UpdateGrid()
    {
        // Get robot position directly from pose estimator
        Vector2 robotPos = pose.position;
        float robotHeading = pose.heading;

        // Convert robot position to grid coordinates
        Vector2Int robotCell = WorldToGrid(robotPos);
        
        // Check if LIDAR data is available
        if (lidar.distances == null || lidar.distances.Length == 0)
        {
            Debug.LogWarning("No LIDAR distance data available. Grid won't update.");
            return;
        }
        
        for (int i = 0; i < lidar.rays; i++)
        {
            float distance = lidar.distances[i];
            float angle = robotHeading + (i * 360f / lidar.rays);
            
            // Calculate direction in X,Y plane (which maps to X,Z in Unity)
            Vector2 dir = new Vector2(Mathf.Cos(angle * Mathf.Deg2Rad), Mathf.Sin(angle * Mathf.Deg2Rad));
            Vector2 hitPoint = robotPos + dir * distance;
            Vector2Int hitCell = WorldToGrid(hitPoint);
            
            // Skip if hit point is outside the grid
            if (!IsValidCell(hitCell))
                continue;
                
            // Detect if this ray hit an obstacle (distance less than max range)
            bool hitObstacle = distance < lidar.maxDistance - 0.1f;
            
            if (showDebugInfo) 
            {
                // Show debug ray in scene view (red for hits, yellow for max range)
                Debug.DrawLine(
                    new Vector3(robotPos.x, heightOffset, robotPos.y), 
                    new Vector3(hitPoint.x, heightOffset, hitPoint.y),
                    hitObstacle ? Color.red : Color.yellow, 0.1f);
            }
            
            // If we hit an obstacle, directly set its cell as occupied before ray tracing
            if (hitObstacle && IsValidCell(hitCell))
            {
                // Directly mark the endpoint as an obstacle with high confidence
                occupancyGrid[hitCell.x, hitCell.y] = Mathf.Min(100, occupancyGrid[hitCell.x, hitCell.y] + occupiedUpdateValue * 4);
                
                if (showDebugInfo)
                {
                    // Draw a vertical line at the hit point for better visualization
                    Debug.DrawLine(
                        new Vector3(hitPoint.x, 0, hitPoint.y),
                        new Vector3(hitPoint.x, heightOffset * 2, hitPoint.y),
                        Color.red, 0.1f);
                }
            }
            
            // Trace the ray using Bresenham's line algorithm (still needed for free cells)
            TraceLine(robotCell, hitCell, hitObstacle);
        }
    }
    
    // Bresenham's line algorithm to trace rays in grid
    void TraceLine(Vector2Int start, Vector2Int end, bool hitObstacle)
    {
        int x0 = start.x;
        int y0 = start.y;
        int x1 = end.x;
        int y1 = end.y;
        
        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        while (true) 
        {
            // If the current cell is valid
            if (IsValidCell(new Vector2Int(x0, y0)))
            {
                // Skip the robot's own cell and the endpoint (which we already handled)
                if (x0 == start.x && y0 == start.y)
                {
                    // Do nothing for the robot's cell
                }
                else if (x0 == x1 && y0 == y1)
                {
                    // Skip the endpoint - we already handled this in UpdateGrid
                    // We avoid double-counting here
                }
                else
                {
                    // Path to hit point - decrease occupied probability (mark as free)
                    UpdateCellProbability(x0, y0, false);
                }
            }
            
            // Exit condition
            if (x0 == x1 && y0 == y1) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) 
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) 
            {
                err += dx;
                y0 += sy;
            }
        }
    }
    
    // Update cell probability using Bayesian update
    void UpdateCellProbability(int x, int y, bool isObstacle)
    {
        if (isObstacle)
        {
            // Increase probability (more likely to be occupied)
            // Using a larger increment for obstacles to help them stand out
            occupancyGrid[x, y] = Mathf.Min(100, occupancyGrid[x, y] + occupiedUpdateValue * 2);
        }
        else
        {
            // Decrease probability (more likely to be free)
            occupancyGrid[x, y] = Mathf.Max(0, occupancyGrid[x, y] - freeUpdateValue);
        }
    }

    // Convert world position to grid cell position
    public Vector2Int WorldToGrid(Vector2 worldPos)
    {
        // Calculate position relative to grid center
        float relX = worldPos.x - (gridWorldOrigin.x - (gridWidth * cellSize / 2));
        float relY = worldPos.y - (gridWorldOrigin.y - (gridHeight * cellSize / 2));
        
        // Convert to grid indices
        int x = Mathf.FloorToInt(relX / cellSize);
        int y = Mathf.FloorToInt(relY / cellSize);
        
        return new Vector2Int(x, y);
    }
    
    // Convert grid cell to world position
    public Vector2 GridToWorld(Vector2Int gridPos)
    {
        // Convert from grid indices to world position
        float worldX = (gridPos.x * cellSize) + (gridWorldOrigin.x - (gridWidth * cellSize / 2)) + (cellSize / 2);
        float worldY = (gridPos.y * cellSize) + (gridWorldOrigin.y - (gridHeight * cellSize / 2)) + (cellSize / 2);
        
        return new Vector2(worldX, worldY);
    }

    // Visualize the grid in the Scene view with Gizmos
    void OnDrawGizmos()
    {
        // Only draw if we have initialized the grid
        if (occupancyGrid == null) return;
        
        // If in play mode, use the proper grid origin
        Vector2 originToUse = Application.isPlaying ? gridWorldOrigin : 
                             (pose != null ? pose.position : Vector2.zero);

        if (!Application.isPlaying && !forceGridVisibility)
        {
            // Draw a simple outline of the grid area in edit mode
            Gizmos.color = Color.cyan;
            Vector3 outlineCenter = new Vector3(originToUse.x, 0, originToUse.y);
            Gizmos.DrawWireCube(outlineCenter, new Vector3(gridWidth * cellSize, 0.01f, gridHeight * cellSize));
            return;
        }
        
        // Calculate the bottom-left corner of the grid
        Vector2 gridBottomLeft = new Vector2(
            originToUse.x - (gridWidth * cellSize / 2),
            originToUse.y - (gridHeight * cellSize / 2)
        );

        // Draw grid outline for better visibility
        Gizmos.color = Color.cyan;
        Vector3 gridCenter = new Vector3(originToUse.x, heightOffset, originToUse.y);
        Gizmos.DrawWireCube(gridCenter, new Vector3(gridWidth * cellSize, 0.01f, gridHeight * cellSize));

        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                // Calculate cell center in world coordinates
                Vector3 cellCenter = new Vector3(
                    gridBottomLeft.x + (x * cellSize) + (cellSize / 2f),
                    0, // Y in Unity is height
                    gridBottomLeft.y + (y * cellSize) + (cellSize / 2f)
                );

                // Only draw cells with significant probability (to reduce visual clutter)
                if (!forceGridVisibility && 
                    occupancyGrid[x, y] > freeThreshold && 
                    occupancyGrid[x, y] < occupiedThreshold)
                {
                    continue; // Skip drawing uncertain cells
                }

                // Set color based on cell state using probability values
                if (occupancyGrid[x, y] >= occupiedThreshold)
                {
                    // Use a more noticeable red for obstacles
                    Gizmos.color = new Color(1f, 0f, 0f, 0.8f); // Occupied (bright red, more opaque)
                    
                    // Make obstacles slightly higher for better visibility
                    Gizmos.DrawCube(cellCenter + new Vector3(0, heightOffset, 0), 
                                    new Vector3(cellSize * 0.9f, 0.1f, cellSize * 0.9f));
                }
                else if (occupancyGrid[x, y] <= freeThreshold)
                {
                    Gizmos.color = new Color(0f, 1f, 0f, 0.3f); // Free (transparent green)
                    Gizmos.DrawCube(cellCenter + new Vector3(0, heightOffset * 0.5f, 0), 
                                    new Vector3(cellSize * 0.9f, 0.02f, cellSize * 0.9f));
                }
                else
                {
                    // Scale gray color based on probability (darker = more likely occupied)
                    float grayValue = 0.8f - (occupancyGrid[x, y] / 150f); // More contrast
                    Gizmos.color = new Color(grayValue, grayValue, grayValue, 0.3f);
                    Gizmos.DrawCube(cellCenter + new Vector3(0, heightOffset * 0.5f, 0), 
                                    new Vector3(cellSize * 0.9f, 0.02f, cellSize * 0.9f));
                }
            }
        }
        
        // Draw grid origin for debugging
        if (showDebugInfo && Application.isPlaying)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(new Vector3(originToUse.x, heightOffset, originToUse.y), 0.5f);
            
            // Draw robot position (pose)
            if (pose != null)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawSphere(new Vector3(pose.position.x, heightOffset * 1.5f, pose.position.y), 0.3f);
            }
        }
    }

    public bool IsValidCell(Vector2Int cell)
    {
        return cell.x >= 0 && cell.x < gridWidth && cell.y >= 0 && cell.y < gridHeight;
    }
}
