using UnityEngine;

public class OccupancyGridMap : MonoBehaviour
{
    public int gridSizeX = 400;
    public int gridSizeY = 400;
    public float cellSize = 0.5f;
    public Vector2 origin = new Vector2(-100f, -200f);

    // 0 = unknown, 1 = free, 2 = occupied
    public byte[,] grid;

    public SimulatedLidar lidar;

    void Awake()
    {
        grid = new byte[gridSizeX, gridSizeY];
        if (lidar == null)
            lidar = GetComponent<SimulatedLidar>();
    }

    void Update()
    {
        if (lidar == null || lidar.distances == null) return;
        Debug.Log($"[OccupancyGridMap] Update called at position: {transform.position}");
        UpdateMapWithLidar();
    }

    void UpdateMapWithLidar()
    {
        Vector3 robotPos = transform.position;
        float angleStep = 180f / (lidar.numRays - 1);
        float startAngle = transform.eulerAngles.y - 90f;
        for (int i = 0; i < lidar.numRays; i++)
        {
            float angle = startAngle + i * angleStep;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            float dist = lidar.distances[i];
            Vector3 hitPoint = robotPos + dir * dist;
            MarkCellsAlongRay(robotPos, hitPoint, dist < lidar.maxDistance);
        }
    }

    void MarkCellsAlongRay(Vector3 start, Vector3 end, bool hitObstacle)
    {
        Vector2 start2D = new Vector2(start.x, start.z);
        Vector2 end2D = new Vector2(end.x, end.z);
        int steps = Mathf.CeilToInt(Vector2.Distance(start2D, end2D) / cellSize);
        for (int i = 0; i <= steps; i++)
        {
            Vector2 pos = Vector2.Lerp(start2D, end2D, i / (float)steps);
            int gx = Mathf.RoundToInt((pos.x - origin.x) / cellSize);
            int gy = Mathf.RoundToInt((pos.y - origin.y) / cellSize);
            if (gx >= 0 && gx < gridSizeX && gy >= 0 && gy < gridSizeY)
            {
                if (i == steps && hitObstacle)
                {
                    // Inflate obstacle: mark a 2-cell buffer as occupied
                    for (int dx = -2; dx <= 2; dx++)
                    {
                        for (int dy = -2; dy <= 2; dy++)
                        {
                            int nx = gx + dx;
                            int ny = gy + dy;
                            if (nx >= 0 && nx < gridSizeX && ny >= 0 && ny < gridSizeY)
                                grid[nx, ny] = 2; // occupied
                        }
                    }
                }
                else if (i < steps || !hitObstacle)
                {
                    grid[gx, gy] = 1; // free
                }
            }
            if (i == steps && hitObstacle) break; // Stop after the first obstacle
        }
    }

    public Vector2Int WorldToGrid(Vector2 worldPos)
    {
        int gx = Mathf.RoundToInt((worldPos.x - origin.x) / cellSize);
        int gy = Mathf.RoundToInt((worldPos.y - origin.y) / cellSize);
        return new Vector2Int(gx, gy);
    }

    public Vector2 GridToWorld(Vector2Int gridPos)
    {
        float wx = origin.x + gridPos.x * cellSize;
        float wz = origin.y + gridPos.y * cellSize;
        return new Vector2(wx, wz);
    }

    // Optional: Visualize the grid in the editor
    void OnDrawGizmos()
    {
        if (grid == null) {
            Debug.Log("[OccupancyGridMap] Gizmos: grid is null");
            return;
        }
        Debug.Log("[OccupancyGridMap] OnDrawGizmos called");
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 cellCenter = new Vector3(origin.x + x * cellSize, 0.1f, origin.y + y * cellSize);
                if (grid[x, y] == 1)
                    Gizmos.color = Color.white;
                else if (grid[x, y] == 2)
                    Gizmos.color = Color.black;
                else
                    Gizmos.color = Color.gray;
                Gizmos.DrawCube(cellCenter, new Vector3(cellSize, 0.01f, cellSize));
            }
        }
    }
}
