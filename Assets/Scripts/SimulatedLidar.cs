using UnityEngine;
using System.Collections.Generic;

public class SimulatedLidar : MonoBehaviour
{
    [Header("Lidar Settings")]
    public int numRays = 360;
    public float maxDistance = 10f;
    public float scanFrequency = 5f; // scans per second
    public float angleRange = 360f; // Full 360 degrees scan
    public float startAngleOffset = -180f; // Starting from -180 degrees (behind)
    public LayerMask obstacleMask;
    
    [Header("Realism Settings")]
    public bool addNoise = false;
    public float noiseVariance = 0.05f;
    
    [Header("Debug Visualization")]
    public bool visualizeInGame = false;
    public bool useDistanceColoring = true;
    public Color minDistanceColor = Color.red;
    public Color maxDistanceColor = Color.green;

    // Array to store distance readings
    public float[] distances;
    
    // Cached point cloud for visualization/processing
    [HideInInspector]
    public Vector3[] scanPoints;
    
    // Event that other components can subscribe to
    public delegate void OnLidarScanComplete(float[] distances);
    public event OnLidarScanComplete OnScanComplete;

    private float scanTimer = 0f;

    void Start()
    {
        distances = new float[numRays];
        scanPoints = new Vector3[numRays];
    }

    void Update()
    {
        scanTimer += Time.deltaTime;
        if (scanTimer >= 1f / scanFrequency)
        {
            PerformScan();
            scanTimer = 0f;
            
            // Notify subscribers that a new scan is available
            if (OnScanComplete != null)
            {
                OnScanComplete(distances);
            }
        }
        
        // In-game visualization if enabled
        if (visualizeInGame)
        {
            VisualizeRays();
        }
    }

    void PerformScan()
    {
        float angleStep = angleRange / numRays;
        float currentAngle = transform.eulerAngles.y + startAngleOffset;
        
        for (int i = 0; i < numRays; i++)
        {
            float angle = currentAngle + i * angleStep;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            Ray ray = new Ray(transform.position, dir);
            
            if (Physics.Raycast(ray, out RaycastHit hit, maxDistance, obstacleMask))
            {
                float distance = hit.distance;
                
                // Add noise if enabled
                if (addNoise)
                {
                    distance += Random.Range(-noiseVariance, noiseVariance) * distance;
                }
                
                distances[i] = distance;
                scanPoints[i] = hit.point;
            }
            else
            {
                distances[i] = maxDistance;
                scanPoints[i] = transform.position + dir * maxDistance;
            }
        }
    }
    
    // Get minimum distance in a specified angle range
    public float GetMinDistanceInRange(float startAngle, float endAngle)
    {
        int startIndex = AngleToIndex(startAngle);
        int endIndex = AngleToIndex(endAngle);
        
        float minDist = maxDistance;
        
        // Handle wraparound if needed
        if (endIndex < startIndex)
        {
            // Check first segment (startIndex to end of array)
            for (int i = startIndex; i < distances.Length; i++)
            {
                minDist = Mathf.Min(minDist, distances[i]);
            }
            
            // Check second segment (start of array to endIndex)
            for (int i = 0; i <= endIndex; i++)
            {
                minDist = Mathf.Min(minDist, distances[i]);
            }
        }
        else
        {
            // Simple case - just check from start to end
            for (int i = startIndex; i <= endIndex; i++)
            {
                minDist = Mathf.Min(minDist, distances[i]);
            }
        }
        
        return minDist;
    }
    
    // Convert a world angle to an index in our distances array
    private int AngleToIndex(float worldAngle)
    {
        // Normalize the angle to lidar's local reference frame
        float localAngle = worldAngle - transform.eulerAngles.y - startAngleOffset;
        
        // Ensure angle is in the 0-360 range
        while (localAngle < 0) localAngle += 360f;
        while (localAngle >= 360f) localAngle -= 360f;
        
        // Convert to index
        int index = Mathf.RoundToInt(localAngle * numRays / angleRange) % numRays;
        return index;
    }
    
    // Check if there's an obstacle in a direction within threshold distance
    public bool IsObstacleInDirection(Vector3 direction, float threshold)
    {
        // Convert direction to world angle
        float angle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
        if (angle < 0) angle += 360f;
        
        // Get the closest reading in that direction
        int index = AngleToIndex(angle);
        return distances[index] < threshold;
    }
    
    // Get a simplified point cloud with fewer points for efficiency
    public List<Vector3> GetSimplifiedPointCloud(int stride)
    {
        List<Vector3> simplified = new List<Vector3>();
        for (int i = 0; i < scanPoints.Length; i += stride)
        {
            if (distances[i] < maxDistance) // Only include actual hits
            {
                simplified.Add(scanPoints[i]);
            }
        }
        return simplified;
    }
    
    // In-game visualization function
    private void VisualizeRays()
    {
        if (distances == null) return;
        
        float angleStep = angleRange / numRays;
        float currentAngle = transform.eulerAngles.y + startAngleOffset;
        
        for (int i = 0; i < numRays; i += 4) // Skip some rays for performance
        {
            float angle = currentAngle + i * angleStep;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            
            Color rayColor;
            if (useDistanceColoring)
            {
                // Interpolate color based on distance
                float t = distances[i] / maxDistance;
                rayColor = Color.Lerp(minDistanceColor, maxDistanceColor, t);
            }
            else
            {
                rayColor = Color.green;
            }
            
            Debug.DrawRay(transform.position, dir * distances[i], rayColor);
        }
    }

    // Editor visualization
    void OnDrawGizmosSelected()
    {
        if (distances == null || distances.Length != numRays) 
            return;
        
        float angleStep = angleRange / numRays;
        float currentAngle = transform.eulerAngles.y + startAngleOffset;
        
        for (int i = 0; i < numRays; i += 4) // Skip some rays for better performance
        {
            float angle = currentAngle + i * angleStep;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            
            if (useDistanceColoring)
            {
                // Color based on distance - red for close, green for far
                float t = distances[i] / maxDistance;
                Gizmos.color = Color.Lerp(minDistanceColor, maxDistanceColor, t);
            }
            else
            {
                Gizmos.color = Color.green;
            }
            
            Gizmos.DrawRay(transform.position, dir * distances[i]);
        }
    }
}
