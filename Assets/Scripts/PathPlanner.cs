using System.Collections.Generic;
using UnityEngine;

public class PathPlanner : MonoBehaviour
{
    public OccupancyGridMap map;
    public FrontierExplorer explorer;
    public ZoneManager zoneManager;
    public enum RobotState { GoToZone, ExploreZone }
    public RobotState state = RobotState.GoToZone;

    public List<Vector2> currentPath = new List<Vector2>();

    public List<Vector2> FindPath(Vector2Int start, Vector2Int goal, bool allowUnknown = false)
    {
        int gx = map.gridSizeX;
        int gy = map.gridSizeY;
        if (start.x < 0 || start.x >= gx || start.y < 0 || start.y >= gy ||
            goal.x < 0 || goal.x >= gx || goal.y < 0 || goal.y >= gy)
        {
            Debug.LogWarning($"[PathPlanner] Start or goal out of grid bounds: start=({start.x},{start.y}), goal=({goal.x},{goal.y}), grid=({gx},{gy})");
            return new List<Vector2>();
        }
        bool[,] closed = new bool[gx, gy];
        float[,] gScore = new float[gx, gy];
        for (int x = 0; x < gx; x++)
            for (int y = 0; y < gy; y++)
                gScore[x, y] = float.MaxValue;
        gScore[start.x, start.y] = 0;
        var open = new PriorityQueue<Vector2Int>();
        open.Enqueue(start, 0);
        Dictionary<Vector2Int, Vector2Int> cameFrom = new Dictionary<Vector2Int, Vector2Int>();
        int[] dx = { -1, 1, 0, 0 };
        int[] dy = { 0, 0, -1, 1 };
        while (open.Count > 0)
        {
            Vector2Int current = open.Dequeue();
            if (current == goal)
                return ReconstructPath(cameFrom, current);
            closed[current.x, current.y] = true;
            for (int dir = 0; dir < 4; dir++)
            {
                int nx = current.x + dx[dir];
                int ny = current.y + dy[dir];
                if (nx < 0 || nx >= gx || ny < 0 || ny >= gy) continue;
                if (closed[nx, ny]) continue;
                if (map.grid[nx, ny] == 2) continue; // avoid occupied
                if (!allowUnknown && map.grid[nx, ny] == 0) continue; // avoid unknown unless allowed
                float tentativeG = gScore[current.x, current.y] + 1;
                if (tentativeG < gScore[nx, ny])
                {
                    gScore[nx, ny] = tentativeG;
                    float f = tentativeG + Vector2Int.Distance(new Vector2Int(nx, ny), goal);
                    open.Enqueue(new Vector2Int(nx, ny), f);
                    cameFrom[new Vector2Int(nx, ny)] = current;
                }
            }
        }
        return new List<Vector2>(); // No path found
    }

    List<Vector2> ReconstructPath(Dictionary<Vector2Int, Vector2Int> cameFrom, Vector2Int current)
    {
        List<Vector2> path = new List<Vector2>();
        path.Add(map.GridToWorld(current));
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Add(map.GridToWorld(current));
        }
        path.Reverse();
        return path;
    }

    void OnDrawGizmos()
    {
        if (currentPath == null || currentPath.Count < 2) return;
        Gizmos.color = Color.cyan;
        for (int i = 0; i < currentPath.Count - 1; i++)
        {
            Vector3 a = new Vector3(currentPath[i].x, 0.2f, currentPath[i].y);
            Vector3 b = new Vector3(currentPath[i + 1].x, 0.2f, currentPath[i + 1].y);
            Gizmos.DrawLine(a, b);
            Gizmos.DrawSphere(a, 0.07f);
        }
        // Draw goal
        Vector3 goal = new Vector3(currentPath[currentPath.Count - 1].x, 0.2f, currentPath[currentPath.Count - 1].y);
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(goal, 0.12f);
    }

    void Update()
    {
        if (zoneManager == null) 
        {
            Debug.LogError("[PathPlanner] ZoneManager is null!");
            return;
        }
        
        if (map == null)
        {
            Debug.LogError("[PathPlanner] OccupancyGridMap is null!");
            return;
        }
        
        Vector2Int start = map.WorldToGrid(new Vector2(transform.position.x, transform.position.z));
        
        // FALLBACK 1: First try to find a path to a cell in the current zone
        List<Vector2Int> zoneFreeCells = new List<Vector2Int>();
        List<Vector2Int> allZoneCells = new List<Vector2Int>();
        
        // Scan a smaller region around the robot to save performance
        int scanRadius = 30; // Reduced from 50 to 30 for better performance
        int minX = Mathf.Max(0, start.x - scanRadius);
        int maxX = Mathf.Min(map.gridSizeX - 1, start.x + scanRadius);
        int minY = Mathf.Max(0, start.y - scanRadius);
        int maxY = Mathf.Min(map.gridSizeY - 1, start.y + scanRadius);
        
        for (int x = minX; x <= maxX; x++)
        {
            for (int y = minY; y <= maxY; y++)
            {
                Vector2 worldPos = map.GridToWorld(new Vector2Int(x, y));
                if (zoneManager.InZone(worldPos))
                {
                    allZoneCells.Add(new Vector2Int(x, y));
                    if (map.grid[x, y] == 1) // If it's free
                    {
                        zoneFreeCells.Add(new Vector2Int(x, y));
                    }
                }
            }
        }
        
        // Only log this info every 30 frames to reduce overhead
        if (Time.frameCount % 30 == 0) 
        {
            Debug.Log($"[PathPlanner] Found {zoneFreeCells.Count} free cells and {allZoneCells.Count} total cells in zone");
        }
        
        // Try to find a path to a free cell in the zone
        List<Vector2> path = new List<Vector2>();
        
        // STRATEGY 1: Path to free cell in zone
        if (zoneFreeCells.Count > 0)
        {
            Vector2Int goal = FindClosestPoint(start, zoneFreeCells);
            path = FindPath(start, goal, false); // First try without allowing unknown cells
            
            // If that fails, allow unknown cells
            if (path.Count == 0)
            {
                path = FindPath(start, goal, true);
            }
        }
        
        // STRATEGY 2: If no path to free cells, try any cell in zone
        if (path.Count == 0 && allZoneCells.Count > 0)
        {
            Vector2Int goal = FindClosestPoint(start, allZoneCells);
            path = FindPath(start, goal, true); // Allow unknown cells
        }
        
        // STRATEGY 3: If explorer has a frontier, try that
        if (path.Count == 0 && explorer != null && explorer.currentFrontier != null)
        {
            Vector2Int goal = Vector2Int.RoundToInt(explorer.currentFrontier.Value.center);
            path = FindPath(start, goal, true);
        }
        
        // STRATEGY 4: Emergency fallback - just move forward
        if (path.Count == 0)
        {
            // Only log this warning every 30 frames
            if (Time.frameCount % 30 == 0)
            {
                Debug.LogWarning("[PathPlanner] No path found with any strategy, using emergency path");
            }
            
            // Create a simple path that just moves forward a bit
            Vector3 forward = transform.forward * 2.0f;
            Vector2 targetPos = new Vector2(transform.position.x + forward.x, transform.position.z + forward.z);
            path.Add(new Vector2(transform.position.x, transform.position.z));
            path.Add(targetPos);
        }
        
        // Only log if path changes significantly
        if (path.Count > 0 && (currentPath == null || currentPath.Count == 0 || 
            Vector2.Distance(path[path.Count-1], currentPath[currentPath.Count-1]) > 0.5f))
        {
            Debug.Log($"[PathPlanner] Path updated with {path.Count} points. State: {state}");
        }
        
        currentPath = path;
        
        // Update robot state based on zone, but log less frequently
        bool inZoneNow = zoneManager.InCurrentZone;
        if (state == RobotState.GoToZone && inZoneNow)
        {
            state = RobotState.ExploreZone;
            Debug.Log("[PathPlanner] Changing state to ExploreZone"); // Only log on state change
        }
        else if (state == RobotState.ExploreZone && !inZoneNow)
        {
            state = RobotState.GoToZone;
            Debug.Log("[PathPlanner] Changing state to GoToZone"); // Only log on state change
        }
    }
    
    // Helper function to find the closest point from a list
    private Vector2Int FindClosestPoint(Vector2Int start, List<Vector2Int> points)
    {
        Vector2Int closest = points[0];
        float minDist = Vector2Int.Distance(start, points[0]);
        
        foreach (var point in points)
        {
            float dist = Vector2Int.Distance(start, point);
            if (dist < minDist)
            {
                minDist = dist;
                closest = point;
            }
        }
        
        return closest;
    }
}

// Simple priority queue for A*
public class PriorityQueue<T>
{
    private List<KeyValuePair<T, float>> elements = new List<KeyValuePair<T, float>>();
    public int Count => elements.Count;
    public void Enqueue(T item, float priority)
    {
        elements.Add(new KeyValuePair<T, float>(item, priority));
    }
    public T Dequeue()
    {
        int bestIndex = 0;
        for (int i = 1; i < elements.Count; i++)
            if (elements[i].Value < elements[bestIndex].Value)
                bestIndex = i;
        T bestItem = elements[bestIndex].Key;
        elements.RemoveAt(bestIndex);
        return bestItem;
    }
}
