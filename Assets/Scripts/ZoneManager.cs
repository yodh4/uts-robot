using UnityEngine;
using System.Collections.Generic;

public class ZoneManager : MonoBehaviour
{
    [System.Serializable]
    public struct Zone
    {
        public Vector3 p1, p2, p3, p4; // Four corners
        public Vector3 Center => (p1 + p2 + p3 + p4) / 4f;
    }

    public List<Zone> zones = new List<Zone>();
    public int currentZoneIndex = 0;
    public Transform robot;
    public float zoneEnterThreshold = 2.0f;

    public bool InCurrentZone
    {
        get
        {
            if (zones.Count == 0 || robot == null) return false;
            Vector2 pos2D = new Vector2(robot.position.x, robot.position.z);
            bool inside = InZone(pos2D);
            Debug.Log($"[ZoneManager] Robot at {pos2D} in zone {currentZoneIndex}: {inside}");
            return inside;
        }
    }

    // Ray-casting algorithm for point-in-polygon
    private bool PointInPolygon(Vector2 p, Vector2[] poly)
    {
        int n = poly.Length;
        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
                (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y + 0.00001f) + poly[i].x))
                inside = !inside;
        }
        return inside;
    }

    public Vector3 GetCurrentZoneCenter()
    {
        if (zones.Count == 0) return Vector3.zero;
        return zones[currentZoneIndex].Center;
    }

    public void GoToNextZone()
    {
        if (currentZoneIndex < zones.Count - 1)
            currentZoneIndex++;
    }

    // Add this helper to ZoneManager for use by FrontierExplorer
    public bool InZone(Vector2 worldPos)
    {
        if (zones.Count == 0) return false;
        Zone z = zones[currentZoneIndex];
        Vector2[] poly = new Vector2[] {
            new Vector2(z.p1.x, z.p1.z),
            new Vector2(z.p2.x, z.p2.z),
            new Vector2(z.p3.x, z.p3.z),
            new Vector2(z.p4.x, z.p4.z)
        };
        bool inside = PointInPolygon(worldPos, poly);
        Debug.Log($"[ZoneManager] InZone check for {worldPos} in zone {currentZoneIndex}: {inside}");
        return inside;
    }

    void Awake()
    {
        // Add your updated zones directly here (ignore y, use x and z only)
        zones = new List<Zone>
        {
            new Zone {
                p1 = new Vector3(4.9000001f, 0f, -36.0400009f),
                p2 = new Vector3(4.86000013f, 0f, -50.9399986f),
                p3 = new Vector3(59.0400009f, 0f, -51.3300018f),
                p4 = new Vector3(59.3400002f, 0f, -36.2599983f)
            },
            new Zone {
                p1 = new Vector3(61.1300011f, 0f, -51.1399994f),
                p2 = new Vector3(90.9100037f, 0f, -51.4799995f),
                p3 = new Vector3(60.1599998f, 0f, -6.28999996f),
                p4 = new Vector3(96.6600037f, 0f, -6.05999994f)
            },
            new Zone {
                p1 = new Vector3(77.9100037f, 0f, 7f),
                p2 = new Vector3(77.6699982f, 0f, 24.1800003f),
                p3 = new Vector3(42.4399986f, 0f, 24.0400009f),
                p4 = new Vector3(42.2700005f, 0f, 7.09000015f)
            }
        };
    }
}
