using UnityEngine;

public class RobotController : MonoBehaviour
{
    // Reference to the car controller for movement
    private SimpleCarController carController;

    // LIDAR parameters
    public int lidarRays = 300; // Number of rays (e.g., 10° increments)
    public float lidarRange = 10f; // Max distance for LIDAR
    public LayerMask obstacleMask; // Layer for obstacles
    public float lidarHeightOffset = 0.2f; // Height above ground for LIDAR origin
    public int lidarRaysVertical = 1; // Number of vertical layers for LIDAR
    public float lidarVerticalAngle = 5f; // Angle between vertical layers

    // Odometry
    private Vector3 lastPosition;
    private float totalDistanceTravelled = 0f;
    private float heading = 0f;

    // Occupancy grid mapping
    public int gridSize = 100; // Number of cells per side
    public float cellSize = 0.7f; // Size of each cell in world units
    private int[,] occupancyGrid; // -1: unknown, 0: free, 1: occupied
    private Vector2 gridOrigin; // World position of grid (bottom-left corner)

    void Awake()
    {
        carController = GetComponent<SimpleCarController>();
        occupancyGrid = new int[gridSize, gridSize];
        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                occupancyGrid[x, y] = -1; // Unknown
        lastPosition = transform.position;
        gridOrigin = new Vector2(30f, -51.48f);
    }

    void Update()
    {
        // gridOrigin = new Vector2(transform.position.x - gridSize * cellSize / 2f, transform.position.z - gridSize * cellSize / 2f);
        UpdateOdometry();
        SimulateLidar();
    }

    // Simulate a 360-degree LIDAR using raycasts
    void SimulateLidar()
    {
        float angleStep = 360f / lidarRays;
        Vector3 lidarOrigin = transform.position + Vector3.up * lidarHeightOffset;
        for (int v = 0; v < lidarRaysVertical; v++)
        {
            float verticalOffset = (v - (lidarRaysVertical - 1) / 2f) * lidarVerticalAngle;
            Quaternion verticalRot = Quaternion.Euler(verticalOffset, 0, 0);
            for (int i = 0; i < lidarRays; i++)
            {
                float angle = i * angleStep;
                Vector3 dir = verticalRot * (Quaternion.Euler(0, angle, 0) * transform.forward);
                Ray ray = new Ray(lidarOrigin, dir);
                if (Physics.Raycast(ray, out RaycastHit hit, lidarRange, obstacleMask))
                {
                    // Debug.DrawLine(lidarOrigin, hit.point, Color.red);
                    UpdateGridWithRay(lidarOrigin, hit.point, true);
                }
                else
                {
                    // Debug.DrawRay(lidarOrigin, dir * lidarRange, Color.green);
                    UpdateGridWithRay(lidarOrigin, lidarOrigin + dir * lidarRange, false);
                }
            }
        }
    }

    // Bresenham's line algorithm to update grid cells along a ray
    void UpdateGridWithRay(Vector3 start, Vector3 end, bool hitObstacle)
    {
        Vector2 start2D = new Vector2(start.x, start.z);
        Vector2 end2D = new Vector2(end.x, end.z);
        int x0, y0, x1, y1;
        WorldToGrid(start2D, out x0, out y0);
        WorldToGrid(end2D, out x1, out y1);
        int dx = Mathf.Abs(x1 - x0), dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (true)
        {
            if (x >= 0 && x < gridSize && y >= 0 && y < gridSize)
                occupancyGrid[x, y] = 0; // Free
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
        // Mark obstacle cell if hit
        // Mark obstacle cell if hit
    if (hitObstacle && x1 >= 0 && x1 < gridSize && y1 >= 0 && y1 < gridSize) {
        occupancyGrid[x1, y1] = 1;
    }
        
    }

    // Convert world position to grid indices
    void WorldToGrid(Vector2 pos, out int x, out int y)
    {
        x = Mathf.FloorToInt((pos.x - gridOrigin.x) / cellSize);
        y = Mathf.FloorToInt((pos.y - gridOrigin.y) / cellSize);
    }

    // Visualize occupancy grid in Scene view
    void OnDrawGizmos()
    {
        if (occupancyGrid == null) return;
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                Vector3 cellCenter = new Vector3(gridOrigin.x + (x + 0.5f) * cellSize, 0.05f, gridOrigin.y + (y + 0.5f) * cellSize);
                if (occupancyGrid[x, y] == -1)
                    Gizmos.color = new Color(0.2f, 0.2f, 0.2f, 1f); // Unknown
                else if (occupancyGrid[x, y] == 0)
                    Gizmos.color = Color.green;
                else if (occupancyGrid[x, y] == 1)
                    Gizmos.color = Color.red;
                Gizmos.DrawCube(cellCenter, new Vector3(cellSize, 0.01f, cellSize));
            }
        }
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(
            new Vector3(gridOrigin.x + gridSize * cellSize / 2f, 0.1f, gridOrigin.y + gridSize * cellSize / 2f),
            new Vector3(gridSize * cellSize, 0.1f, gridSize * cellSize)
        );
    }

    // Update odometry based on movement
    void UpdateOdometry()
    {
        float distance = Vector3.Distance(transform.position, lastPosition);
        totalDistanceTravelled += distance;
        heading = transform.eulerAngles.y;
        lastPosition = transform.position;
    }

    // Get odometry data
    public float GetTotalDistanceTravelled() => totalDistanceTravelled;
    public float GetHeading() => heading;

    public int[,] GetOccupancyGrid() => occupancyGrid;
    public Vector2 GetGridOrigin() => gridOrigin;
}