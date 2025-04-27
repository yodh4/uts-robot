using System.Collections.Generic;
using UnityEngine;

public class FrontierExplorer : MonoBehaviour
{
    public OccupancyGridMap map;
    public float minFrontierSize = 2f; // Minimum length in world units
    public ZoneManager zoneManager;

    [System.Serializable]
    public struct Frontier
    {
        public List<Vector2Int> cells;
        public Vector2 center;
    }

    public List<Frontier> frontiers = new List<Frontier>();
    public Frontier? currentFrontier = null;

    void Update()
    {
        if (map == null || map.grid == null) return;
        FindFrontiers();
        SelectCurrentFrontier();
    }

    void FindFrontiers()
    {
        frontiers.Clear();
        int gx = map.gridSizeX;
        int gy = map.gridSizeY;
        bool[,] visited = new bool[gx, gy];
        for (int x = 1; x < gx - 1; x++)
        {
            for (int y = 1; y < gy - 1; y++)
            {
                if (map.grid[x, y] == 1 && IsFrontierCell(x, y) && !visited[x, y])
                {
                    List<Vector2Int> frontierCells = new List<Vector2Int>();
                    Queue<Vector2Int> queue = new Queue<Vector2Int>();
                    queue.Enqueue(new Vector2Int(x, y));
                    visited[x, y] = true;
                    while (queue.Count > 0)
                    {
                        Vector2Int cell = queue.Dequeue();
                        frontierCells.Add(cell);
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            for (int dy = -1; dy <= 1; dy++)
                            {
                                int nx = cell.x + dx;
                                int ny = cell.y + dy;
                                if (nx > 0 && nx < gx - 1 && ny > 0 && ny < gy - 1 && !visited[nx, ny])
                                {
                                    if (map.grid[nx, ny] == 1 && IsFrontierCell(nx, ny))
                                    {
                                        queue.Enqueue(new Vector2Int(nx, ny));
                                        visited[nx, ny] = true;
                                    }
                                }
                            }
                        }
                    }
                    if (frontierCells.Count * map.cellSize >= minFrontierSize)
                    {
                        Vector2 sum = Vector2.zero;
                        foreach (var c in frontierCells)
                            sum += new Vector2(c.x, c.y);
                        Vector2 center = sum / frontierCells.Count;
                        // Only add frontiers inside the current zone
                        if (zoneManager == null || zoneManager.InZone(map.GridToWorld(Vector2Int.RoundToInt(center))))
                        {
                            frontiers.Add(new Frontier { cells = frontierCells, center = center });
                        }
                    }
                }
            }
        }
    }

    bool IsFrontierCell(int x, int y)
    {
        // A frontier cell is free and has at least one unknown neighbor
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0) continue;
                int nx = x + dx;
                int ny = y + dy;
                if (map.grid[nx, ny] == 0)
                    return true;
            }
        }
        return false;
    }

    void SelectCurrentFrontier()
    {
        if (frontiers.Count == 0)
        {
            currentFrontier = null;
            return;
        }
        // Pick the closest frontier center to the robot
        Vector3 robotPos = transform.position;
        float minDist = float.MaxValue;
        Frontier? best = null;
        foreach (var f in frontiers)
        {
            Vector3 worldCenter = map.origin + f.center * map.cellSize;
            float dist = Vector2.Distance(new Vector2(robotPos.x, robotPos.z), new Vector2(worldCenter.x, worldCenter.y));
            if (dist < minDist)
            {
                minDist = dist;
                best = f;
            }
        }
        currentFrontier = best;
    }

    // Optional: Visualize frontiers
    void OnDrawGizmos()
    {
        if (frontiers == null) return;
        Gizmos.color = Color.cyan;
        foreach (var f in frontiers)
        {
            foreach (var cell in f.cells)
            {
                Vector3 pos = new Vector3(map.origin.x + cell.x * map.cellSize, 0.2f, map.origin.y + cell.y * map.cellSize);
                Gizmos.DrawCube(pos, new Vector3(map.cellSize * 0.5f, 0.01f, map.cellSize * 0.5f));
            }
        }
        if (currentFrontier != null)
        {
            Gizmos.color = Color.yellow;
            Vector3 pos = new Vector3(map.origin.x + currentFrontier.Value.center.x * map.cellSize, 0.3f, map.origin.y + currentFrontier.Value.center.y * map.cellSize);
            Gizmos.DrawSphere(pos, map.cellSize * 0.5f);
        }
    }
}
