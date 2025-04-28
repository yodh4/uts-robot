using UnityEngine;

public class LidarSimulator : MonoBehaviour
{
    public int rays = 360;
    public float maxDistance = 10f;
    public LayerMask obstacleLayer;

    public float[] distances;
    
    // Add color options for visualization
    public Color hitColor = Color.red;
    public Color missColor = Color.green;
    public bool showRays = true;

    private void Start()
    {
        distances = new float[rays]; // initialize distances array
    }
    
    private void Update()
    {
        if (distances == null || distances.Length != rays)
        {
            distances = new float[rays];
        }

        for (int i = 0; i < rays; i++)
        {
            float angle = i * (360f / rays);
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            Ray ray = new Ray(transform.position, dir);
            if (Physics.Raycast(ray, out RaycastHit hit, maxDistance, obstacleLayer))
            {
                distances[i] = hit.distance;
                
                // Draw the ray that hit something in red
                if (showRays) 
                    Debug.DrawLine(transform.position, hit.point, hitColor);
            }
            else
            {
                distances[i] = maxDistance;
                
                // Draw the ray that didn't hit anything in green
                if (showRays) 
                    Debug.DrawRay(transform.position, dir * maxDistance, missColor);
            }
        }
    }
    
    // Optionally add a gizmo to show the LIDAR's position and range
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(0, 0.5f, 1f, 0.3f); // Light blue, semi-transparent
        Gizmos.DrawSphere(transform.position, 0.2f);
        
        // Draw a wire sphere to show max range
        Gizmos.color = new Color(0, 0.5f, 1f, 0.1f);
        Gizmos.DrawWireSphere(transform.position, maxDistance);
    }
}
