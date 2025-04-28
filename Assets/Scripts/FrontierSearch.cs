using System.Collections.Generic;
using UnityEngine;

public class FrontierSearch : MonoBehaviour
{
    // Embedded frontier class to replace the separate ExplorationFrontier class
    [System.Serializable]
    public class Frontier
    {
        public int size;                   // Number of cells in the frontier
        public Vector3 centroid;           // World position of the frontier's center
        public List<Vector2Int> cells;     // List of cells in the frontier

        public Frontier()
        {
            size = 0;
            centroid = Vector3.zero;
            cells = new List<Vector2Int>();
        }
    }

    // Container for a list of frontiers
    [System.Serializable]
    public class FrontierCollection
    {
        public List<Frontier> frontiers;

        public FrontierCollection()
        {
            frontiers = new List<Frontier>();
        }
    }

    // Reference to the occupancy grid mapper
    public OccupancyGridMapper gridMapper;
    
    // Minimum size for a frontier to be considered valid
    public int minFrontierSize = 3;  // Smaller size for easier detection
    
    // Debug visualization options
    public bool showDebugInfo = true;  
    public Color frontierColor = new Color(0, 0, 1, 0.6f); // Blue
    public Color visitedColor = new Color(0.5f, 0.5f, 0.5f, 0.3f); // Gray

    // Cache for the latest detected frontiers
    private FrontierCollection lastDetectedFrontiers;
    private Dictionary<Vector2Int, bool> lastVisitedCells; // For debugging
    
    private void Start()
    {
        if (gridMapper == null)
            gridMapper = GetComponent<OccupancyGridMapper>();
            
        lastVisitedCells = new Dictionary<Vector2Int, bool>();
    }

    // Add debug method to show grid cell state
    public void DebugLogGridStatus()
    {
        if (gridMapper == null || gridMapper.occupancyGrid == null)
        {
            Debug.LogError("Grid mapper or occupancy grid is null!");
            return;
        }
        
        int unknownCount = 0;
        int freeCount = 0;
        int occupiedCount = 0;
        
        // Count cell types
        for (int x = 0; x < gridMapper.gridWidth; x++)
        {
            for (int y = 0; y < gridMapper.gridHeight; y++)
            {
                int value = gridMapper.occupancyGrid[x, y];
                
                if (value <= gridMapper.freeThreshold)
                    freeCount++;
                else if (value >= gridMapper.occupiedThreshold)
                    occupiedCount++;
                else
                    unknownCount++;
            }
        }
        
        Debug.Log($"Grid status: Total: {gridMapper.gridWidth * gridMapper.gridHeight}, " +
                 $"Unknown: {unknownCount}, Free: {freeCount}, Occupied: {occupiedCount}");
    }

    /// <summary>
    /// Searches for frontiers in the occupancy grid starting from the given position
    /// </summary>
    /// <param name="start">Starting position in grid coordinates</param>
    /// <returns>A list of detected frontiers</returns>
    public FrontierCollection SearchFrontiers(Vector2Int start)
    {
        // Check if the gridMapper and its grid exist
        if (gridMapper == null || gridMapper.occupancyGrid == null)
        {
            Debug.LogError("Grid mapper or occupancy grid is null!");
            return null;
        }
        
        // Log grid status for debugging
        DebugLogGridStatus();
        
        // Create queue for breadth-first search
        Queue<Vector2Int> queue = new Queue<Vector2Int>();
        queue.Enqueue(start);
        
        // Initialize dictionaries for keeping track of visited and frontier cells
        Dictionary<Vector2Int, bool> visited = new Dictionary<Vector2Int, bool>();
        Dictionary<Vector2Int, bool> isFrontier = new Dictionary<Vector2Int, bool>();
        visited[start] = true;
        
        // Initialize list of frontiers
        FrontierCollection frontierList = new FrontierCollection();
        
        // BFS through the grid to find frontiers
        while (queue.Count > 0)
        {
            Vector2Int current = queue.Dequeue();
            
            foreach (Vector2Int neighbor in GetNeighbors4(current))
            {
                // Skip invalid cells
                if (!gridMapper.IsValidCell(neighbor))
                    continue;
                
                int value = gridMapper.occupancyGrid[neighbor.x, neighbor.y];
                bool isFree = value <= gridMapper.freeThreshold;
                
                // If the neighbor is free and hasn't been visited, add it to the queue
                if (isFree && !visited.ContainsKey(neighbor))
                {
                    visited[neighbor] = true;
                    queue.Enqueue(neighbor);
                }
                // If the neighbor is a potential frontier cell and hasn't been marked as a frontier
                else if (IsNewFrontierCell(neighbor, isFrontier))
                {
                    // Build a new frontier starting from this cell
                    Frontier newFrontier = BuildNewFrontier(neighbor, isFrontier);
                    
                    // Only consider frontiers of sufficient size
                    if (newFrontier.size >= minFrontierSize)
                    {
                        frontierList.frontiers.Add(newFrontier);
                        Debug.Log($"Found frontier with {newFrontier.size} cells at {newFrontier.centroid}");
                    }
                }
            }
        }
        
        // Cache the frontiers and visited cells for visualization
        lastDetectedFrontiers = frontierList;
        lastVisitedCells = visited;
        
        Debug.Log($"Found {frontierList.frontiers.Count} frontiers from start position {start}");
        
        return frontierList;
    }
    
    /// <summary>
    /// Builds a new frontier starting from an initial cell
    /// </summary>
    /// <param name="initialCell">Starting cell for the frontier</param>
    /// <param name="isFrontier">Dictionary to track which cells have been added to frontiers</param>
    /// <returns>A new frontier object</returns>
    private Frontier BuildNewFrontier(Vector2Int initialCell, Dictionary<Vector2Int, bool> isFrontier)
    {
        Frontier frontier = new Frontier();
        
        // Initialize centroid calculation
        float centroidX = initialCell.x;
        float centroidY = initialCell.y;
        
        // Create queue for breadth-first search
        Queue<Vector2Int> queue = new Queue<Vector2Int>();
        queue.Enqueue(initialCell);
        
        // Mark the initial cell as frontier
        isFrontier[initialCell] = true;
        frontier.cells.Add(initialCell);
        frontier.size = 1;
        
        // BFS to find connecting frontier cells
        while (queue.Count > 0)
        {
            Vector2Int current = queue.Dequeue();
            
            foreach (Vector2Int neighbor in GetNeighbors8(current))
            {
                if (IsNewFrontierCell(neighbor, isFrontier))
                {
                    // Mark as frontier
                    isFrontier[neighbor] = true;
                    
                    // Update centroid calculation
                    centroidX += neighbor.x;
                    centroidY += neighbor.y;
                    
                    // Update frontier data
                    frontier.size += 1;
                    frontier.cells.Add(neighbor);
                    
                    // Continue BFS
                    queue.Enqueue(neighbor);
                }
            }
        }
        
        // Calculate centroid by taking the average
        centroidX /= frontier.size;
        centroidY /= frontier.size;
        
        // Convert grid coordinates to world position using the grid mapper's method
        Vector2Int centroidCell = new Vector2Int(Mathf.RoundToInt(centroidX), Mathf.RoundToInt(centroidY));
        frontier.centroid = GridToWorld(centroidCell);
        
        return frontier;
    }
    
    /// <summary>
    /// Checks if a cell is a frontier cell that hasn't been processed yet
    /// </summary>
    /// <param name="cell">Cell to check</param>
    /// <param name="isFrontier">Dictionary of cells already identified as frontiers</param>
    /// <returns>True if the cell is a new frontier cell</returns>
    private bool IsNewFrontierCell(Vector2Int cell, Dictionary<Vector2Int, bool> isFrontier)
    {
        // Check if cell is valid
        if (!gridMapper.IsValidCell(cell))
            return false;
        
        // Cell must be unknown and not already a frontier
        int value = gridMapper.occupancyGrid[cell.x, cell.y];
        
        // Check for values in the middle range (unknown)
        bool isUnknown = value > gridMapper.freeThreshold && value < gridMapper.occupiedThreshold;
        
        if (!isUnknown || isFrontier.ContainsKey(cell))
            return false;
        
        // Cell should have at least one connected cell that is free
        foreach (Vector2Int neighbor in GetNeighbors4(cell))
        {
            if (!gridMapper.IsValidCell(neighbor))
                continue;
                
            if (gridMapper.occupancyGrid[neighbor.x, neighbor.y] <= gridMapper.freeThreshold)
                return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Get the 4-connected neighbors of a cell (N, E, S, W)
    /// </summary>
    private List<Vector2Int> GetNeighbors4(Vector2Int cell)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>
        {
            new Vector2Int(cell.x + 1, cell.y),
            new Vector2Int(cell.x - 1, cell.y),
            new Vector2Int(cell.x, cell.y + 1),
            new Vector2Int(cell.x, cell.y - 1)
        };
        return neighbors;
    }
    
    /// <summary>
    /// Get the 8-connected neighbors of a cell (N, NE, E, SE, S, SW, W, NW)
    /// </summary>
    private List<Vector2Int> GetNeighbors8(Vector2Int cell)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>
        {
            new Vector2Int(cell.x + 1, cell.y),
            new Vector2Int(cell.x - 1, cell.y),
            new Vector2Int(cell.x, cell.y + 1),
            new Vector2Int(cell.x, cell.y - 1),
            new Vector2Int(cell.x + 1, cell.y + 1),
            new Vector2Int(cell.x - 1, cell.y + 1),
            new Vector2Int(cell.x + 1, cell.y - 1),
            new Vector2Int(cell.x - 1, cell.y - 1)
        };
        return neighbors;
    }
    
    /// <summary>
    /// Converts grid coordinates to world position
    /// </summary>
    private Vector3 GridToWorld(Vector2Int gridPos)
    {
        // Use the grid mapper's conversion method
        Vector2 worldPos2D = gridMapper.GridToWorld(gridPos);
        return new Vector3(worldPos2D.x, 0.1f, worldPos2D.y);
    }
    
    /// <summary>
    /// Visualize the detected frontiers
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!showDebugInfo || gridMapper == null)
            return;
            
        // Draw visited cells (for debugging)
        if (lastVisitedCells != null)
        {
            Gizmos.color = visitedColor;
            foreach (var cell in lastVisitedCells.Keys)
            {
                Vector3 worldPos = GridToWorld(cell);
                Gizmos.DrawCube(worldPos + new Vector3(0, 0.05f, 0), 
                               new Vector3(gridMapper.cellSize * 0.6f, 0.01f, gridMapper.cellSize * 0.6f));
            }
        }
        
        // Draw frontiers
        if (lastDetectedFrontiers != null)
        {
            Gizmos.color = frontierColor;
            
            foreach (var frontier in lastDetectedFrontiers.frontiers)
            {
                // Draw a sphere at the frontier centroid
                Gizmos.DrawSphere(frontier.centroid, 0.3f);
                
                // Draw cubes at each frontier cell
                foreach (Vector2Int cell in frontier.cells)
                {
                    Vector3 worldPos = GridToWorld(cell);
                    Gizmos.DrawCube(worldPos + new Vector3(0, 0.1f, 0), 
                                   new Vector3(gridMapper.cellSize * 0.8f, 0.05f, gridMapper.cellSize * 0.8f));
                }
            }
        }
    }
}