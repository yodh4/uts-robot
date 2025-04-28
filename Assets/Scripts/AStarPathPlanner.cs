using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Helper class used to perform A* search on a grid cell map.
/// This implementation is inspired by the provided reference code.
/// </summary>
public class AStarPathPlanner : MonoBehaviour
{
    // Reference to the occupancy grid mapper
    public OccupancyGridMapper gridMapper;
    
    // Debug visualization options
    public bool showDebugInfo = true;
    public Color pathColor = Color.yellow;
    public Color openSetColor = new Color(0, 1, 0, 0.3f); // Green
    public Color closedSetColor = new Color(1, 0, 0, 0.3f); // Red
    
    // Pathfinding parameters
    public int costMapWeight = 1000;
    public int obstacleThreshold = 50; // Cells with values >= this threshold are considered obstacles
    
    // Cache for visualization
    private List<Vector2Int> lastPath;
    private HashSet<Vector2Int> lastOpenSet;
    private HashSet<Vector2Int> lastClosedSet;
    
    // Cache for obstacle avoidance
    private int[,] costMap;
    public bool costMapGenerated = false;
    
    // 4-connected directions (N, E, S, W)
    private static readonly Vector2Int[] Directions4 = new Vector2Int[]
    {
        new Vector2Int(-1, 0), // West
        new Vector2Int(1, 0),  // East
        new Vector2Int(0, -1), // South
        new Vector2Int(0, 1)   // North
    };
    
    // 8-connected directions (N, NE, E, SE, S, SW, W, NW)
    private static readonly Vector2Int[] Directions8 = new Vector2Int[]
    {
        new Vector2Int(-1, 0),  // West
        new Vector2Int(1, 0),   // East
        new Vector2Int(0, -1),  // South
        new Vector2Int(0, 1),   // North
        new Vector2Int(-1, -1), // Southwest
        new Vector2Int(-1, 1),  // Northwest
        new Vector2Int(1, -1),  // Southeast
        new Vector2Int(1, 1)    // Northeast
    };
    
    private void Start()
    {
        if (gridMapper == null)
            gridMapper = GetComponent<OccupancyGridMapper>();
            
        lastOpenSet = new HashSet<Vector2Int>();
        lastClosedSet = new HashSet<Vector2Int>();
    }
    
    /// <summary>
    /// Generate or update the cost map for path planning
    /// </summary>
    public void GenerateCostMap()
    {
        if (gridMapper == null || gridMapper.occupancyGrid == null)
        {
            Debug.LogError("Grid mapper or occupancy grid is null!");
            return;
        }
        
        int width = gridMapper.gridWidth;
        int height = gridMapper.gridHeight;
        
        // Initialize cost map
        costMap = new int[width, height];
        
        // First, mark obstacles with very high cost
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                int cellValue = gridMapper.occupancyGrid[x, y];
                if (cellValue >= obstacleThreshold)
                {
                    costMap[x, y] = int.MaxValue; // Obstacle
                }
                else
                {
                    costMap[x, y] = 0; // Free space
                }
            }
        }
        
        // Next, create a buffer zone around obstacles with decreasing cost
        int bufferSize = 3;
        for (int buffer = bufferSize; buffer > 0; buffer--)
        {
            // Create a temporary copy of the cost map to avoid modifying it while iterating
            int[,] tempCostMap = new int[width, height];
            System.Array.Copy(costMap, tempCostMap, width * height);
            
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    // Skip if it's already an obstacle or buffer
                    if (tempCostMap[x, y] > 0)
                        continue;
                    
                    // Check neighbors for obstacles or higher-value buffers
                    bool nearObstacle = false;
                    foreach (Vector2Int dir in Directions8)
                    {
                        int nx = x + dir.x;
                        int ny = y + dir.y;
                        
                        // Skip if out of bounds
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                            continue;
                        
                        // If neighbor is obstacle or has higher buffer value than current buffer
                        if (tempCostMap[nx, ny] == int.MaxValue || 
                            (tempCostMap[nx, ny] > 0 && buffer < bufferSize - tempCostMap[nx, ny] + 1))
                        {
                            nearObstacle = true;
                            break;
                        }
                    }
                    
                    // If near an obstacle or higher buffer, set cost
                    if (nearObstacle)
                    {
                        // Use an exponential cost for the buffer zones (higher cost closer to obstacles)
                        costMap[x, y] = (bufferSize - buffer + 1) * (bufferSize - buffer + 1) * 10;
                    }
                }
            }
        }
        
        costMapGenerated = true;
        Debug.Log("Cost map generated");
    }
    
    /// <summary>
    /// Calculates the Euclidean distance between two grid cells
    /// </summary>
    private float EuclideanDistance(Vector2Int p1, Vector2Int p2)
    {
        return Mathf.Sqrt(Mathf.Pow(p2.x - p1.x, 2) + Mathf.Pow(p2.y - p1.y, 2));
    }
    
    /// <summary>
    /// Check if a cell is in bounds of the grid
    /// </summary>
    public bool IsCellInBounds(Vector2Int cell)
    {
        return gridMapper.IsValidCell(cell);
    }
    
    /// <summary>
    /// Check if a cell is walkable (in bounds and not an obstacle)
    /// </summary>
    public bool IsCellWalkable(Vector2Int cell)
    {
        if (!IsCellInBounds(cell))
            return false;
        
        // A cell is walkable if its value is less than the obstacle threshold
        return gridMapper.occupancyGrid[cell.x, cell.y] < obstacleThreshold;
    }
    
    /// <summary>
    /// Get 4-connected neighbors of a cell (N, E, S, W)
    /// </summary>
    public List<Vector2Int> GetNeighbors4(Vector2Int cell, bool mustBeWalkable = true)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();
        
        foreach (Vector2Int dir in Directions4)
        {
            Vector2Int neighbor = new Vector2Int(cell.x + dir.x, cell.y + dir.y);
            
            if (!mustBeWalkable || IsCellWalkable(neighbor))
            {
                neighbors.Add(neighbor);
            }
        }
        
        return neighbors;
    }
    
    /// <summary>
    /// Get 8-connected neighbors of a cell (N, NE, E, SE, S, SW, W, NW)
    /// </summary>
    public List<Vector2Int> GetNeighbors8(Vector2Int cell, bool mustBeWalkable = true)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();
        
        foreach (Vector2Int dir in Directions8)
        {
            Vector2Int neighbor = new Vector2Int(cell.x + dir.x, cell.y + dir.y);
            
            if (!mustBeWalkable || IsCellWalkable(neighbor))
            {
                neighbors.Add(neighbor);
            }
        }
        
        return neighbors;
    }
    
    /// <summary>
    /// Find a path from start to goal using A* algorithm
    /// </summary>
    public List<Vector2Int> FindPath(Vector2Int start, Vector2Int goal)
    {
        // Ensure the grid mapper exists
        if (gridMapper == null || gridMapper.occupancyGrid == null)
        {
            Debug.LogError("Grid mapper or occupancy grid is null!");
            return null;
        }
        
        // Ensure start and goal are walkable
        if (!IsCellWalkable(start) || !IsCellWalkable(goal))
        {
            Debug.LogWarning("Start or goal is not walkable. Start: " + start + ", Goal: " + goal);
            return null;
        }
        
        // Generate cost map if not already done
        if (!costMapGenerated)
        {
            GenerateCostMap();
        }
        
        // Clear previous visualization data
        lastPath = null;
        lastOpenSet.Clear();
        lastClosedSet.Clear();
        
        // A* algorithm starts here
        Dictionary<Vector2Int, Vector2Int> cameFrom = new Dictionary<Vector2Int, Vector2Int>();
        Dictionary<Vector2Int, float> gScore = new Dictionary<Vector2Int, float>();
        Dictionary<Vector2Int, float> fScore = new Dictionary<Vector2Int, float>();
        
        // Priority queue for open set
        SimplePriorityQueue<Vector2Int> openSet = new SimplePriorityQueue<Vector2Int>();
        
        // Initialize scores
        gScore[start] = 0;
        fScore[start] = EuclideanDistance(start, goal);
        
        // Add start to open set
        openSet.Enqueue(start, fScore[start]);
        
        // For visualization
        lastOpenSet.Add(start);
        
        // A* search loop
        while (openSet.Count > 0)
        {
            // Get the node with lowest f score
            Vector2Int current = openSet.Dequeue();
            
            // For visualization
            lastOpenSet.Remove(current);
            lastClosedSet.Add(current);
            
            // If we've reached the goal
            if (current == goal)
            {
                // Reconstruct the path
                List<Vector2Int> path = ReconstructPath(cameFrom, current);
                
                // Cache for visualization
                lastPath = path;
                
                Debug.Log("Path found with " + path.Count + " steps.");
                return path;
            }
            
            // Check all neighbors
            foreach (Vector2Int neighbor in GetNeighbors8(current))
            {
                // Calculate tentative g score
                float moveCost = 1.0f;
                
                // Diagonal movement costs more
                if (neighbor.x != current.x && neighbor.y != current.y)
                {
                    moveCost = 1.414f; // sqrt(2)
                }
                
                // Add cost from cost map if available
                if (costMapGenerated)
                {
                    int additionalCost = costMap[neighbor.x, neighbor.y];
                    if (additionalCost > 0 && additionalCost < int.MaxValue)
                    {
                        moveCost += additionalCost / costMapWeight;
                    }
                }
                
                float tentativeGScore = gScore.ContainsKey(current) ? gScore[current] + moveCost : float.MaxValue;
                
                // If this path is better than any previous one
                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    // Update path and scores
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = tentativeGScore + EuclideanDistance(neighbor, goal);
                    
                    // Add to open set if not already in it
                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Enqueue(neighbor, fScore[neighbor]);
                        
                        // For visualization
                        lastOpenSet.Add(neighbor);
                    }
                    else
                    {
                        // Update priority if already in open set
                        openSet.UpdatePriority(neighbor, fScore[neighbor]);
                    }
                }
            }
        }
        
        // No path found
        Debug.LogWarning("No path found from " + start + " to " + goal);
        return null;
    }
    
    /// <summary>
    /// Reconstruct the path from the came-from map
    /// </summary>
    private List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> cameFrom, Vector2Int current)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        path.Add(current);
        
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Insert(0, current);
        }
        
        return path;
    }
    
    /// <summary>
    /// Convert a grid path to world positions
    /// </summary>
    public List<Vector3> PathToWorld(List<Vector2Int> gridPath)
    {
        if (gridPath == null || gridPath.Count == 0)
            return null;
            
        List<Vector3> worldPath = new List<Vector3>();
        
        foreach (Vector2Int gridCell in gridPath)
        {
            Vector2 world2D = gridMapper.GridToWorld(gridCell);
            float height = Terrain.activeTerrain != null ? 
                Terrain.activeTerrain.SampleHeight(new Vector3(world2D.x, 0, world2D.y)) : 0;
                
            worldPath.Add(new Vector3(world2D.x, height, world2D.y));
        }
        
        return worldPath;
    }
    
    /// <summary>
    /// Visualize the path planning results
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!showDebugInfo || gridMapper == null)
            return;
            
        // Draw the closed set (explored cells)
        Gizmos.color = closedSetColor;
        if (lastClosedSet != null)
        {
            foreach (Vector2Int cell in lastClosedSet)
            {
                Vector2 worldPos = gridMapper.GridToWorld(cell);
                Gizmos.DrawCube(new Vector3(worldPos.x, 0.1f, worldPos.y), 
                    new Vector3(gridMapper.cellSize * 0.8f, 0.1f, gridMapper.cellSize * 0.8f));
            }
        }
        
        // Draw the open set (frontier cells)
        Gizmos.color = openSetColor;
        if (lastOpenSet != null)
        {
            foreach (Vector2Int cell in lastOpenSet)
            {
                Vector2 worldPos = gridMapper.GridToWorld(cell);
                Gizmos.DrawCube(new Vector3(worldPos.x, 0.1f, worldPos.y), 
                    new Vector3(gridMapper.cellSize * 0.8f, 0.1f, gridMapper.cellSize * 0.8f));
            }
        }
        
        // Draw the path
        Gizmos.color = pathColor;
        if (lastPath != null && lastPath.Count > 1)
        {
            for (int i = 0; i < lastPath.Count - 1; i++)
            {
                Vector2 worldPos1 = gridMapper.GridToWorld(lastPath[i]);
                Vector2 worldPos2 = gridMapper.GridToWorld(lastPath[i + 1]);
                
                Gizmos.DrawLine(
                    new Vector3(worldPos1.x, 0.2f, worldPos1.y),
                    new Vector3(worldPos2.x, 0.2f, worldPos2.y));
                    
                Gizmos.DrawSphere(new Vector3(worldPos1.x, 0.2f, worldPos1.y), gridMapper.cellSize * 0.2f);
            }
            
            // Draw the final point in the path
            Vector2 lastWorldPos = gridMapper.GridToWorld(lastPath[lastPath.Count - 1]);
            Gizmos.DrawSphere(new Vector3(lastWorldPos.x, 0.2f, lastWorldPos.y), gridMapper.cellSize * 0.2f);
        }
    }
    
    /// <summary>
    /// A simple priority queue implementation for A* search
    /// </summary>
    private class SimplePriorityQueue<T>
    {
        private List<KeyValuePair<T, float>> elements = new List<KeyValuePair<T, float>>();
        
        public int Count => elements.Count;
        
        public void Enqueue(T item, float priority)
        {
            elements.Add(new KeyValuePair<T, float>(item, priority));
            elements.Sort((x, y) => x.Value.CompareTo(y.Value));
        }
        
        public T Dequeue()
        {
            T item = elements[0].Key;
            elements.RemoveAt(0);
            return item;
        }
        
        public bool Contains(T item)
        {
            return elements.Exists(x => EqualityComparer<T>.Default.Equals(x.Key, item));
        }
        
        public void UpdatePriority(T item, float priority)
        {
            int index = elements.FindIndex(x => EqualityComparer<T>.Default.Equals(x.Key, item));
            if (index >= 0)
            {
                elements[index] = new KeyValuePair<T, float>(item, priority);
                elements.Sort((x, y) => x.Value.CompareTo(y.Value));
            }
        }
    }
}