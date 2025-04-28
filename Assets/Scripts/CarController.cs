using UnityEngine;

public class SimpleCarController : MonoBehaviour
{
    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel;
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;

    public Transform frontLeftTransform;
    public Transform frontRightTransform;
    public Transform rearLeftTransform;
    public Transform rearRightTransform;

    public float motorForce = 1500f;
    public float brakeForce = 3000f;
    public float maxSteerAngle = 30f;

    private float currentSteerAngle;
    private float currentBrakeForce;
    private bool isBraking;

    private Vector3 targetPosition; // Target position for the car to move towards
    public float targetReachThreshold = 1.5f; // Berapa dekat ke target dianggap sampai

    public float lookAheadDistance = 5f;

    [Header("Steering Settings")]
    public float steeringSmoothness = 5.0f; // Higher = smoother but slower response
    private float targetSteerAngle; // The angle we're moving toward


    [Header("Obstacle Avoidance")]
    public float lidarRange = 5f;
    public int lidarRays = 36; // 360/10 = 36 rays (every 10 degrees)
    public LayerMask obstacleLayer;
    public float avoidanceStrength = 1.5f;

    [Header("Roaming Area")]
    public Vector3 areaMin = new Vector3(60.15f, 0f, -51.48f); // Lower left (p1 or p3, min x/z)
    public Vector3 areaMax = new Vector3(96.66f, 0f, -6.06f);  // Upper right (p2 or p4, max x/z)


    private Vector3 gateA = new Vector3(59.95f, 0f, -47.02f);
    private Vector3 gateB = new Vector3(59.95f, 0f, -42.42f);
    private bool hasEnteredArea = false;

    private void Start()
{
    // Set first target as gate midpoint
    targetPosition = (gateA + gateB) / 2f;
}

    private void SetRandomTarget()
{
    float x = Random.Range(areaMin.x, areaMax.x);
    float z = Random.Range(areaMin.z, areaMax.z);
    targetPosition = new Vector3(x, transform.position.y, z);
}

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();
    }

    private void HandleMotor()
{
    float distanceToTarget = Vector3.Distance(targetPosition, transform.position);

    if (distanceToTarget <= targetReachThreshold)
    {
        if (!hasEnteredArea)
        {
            hasEnteredArea = true;
            SetRandomTarget();
        }
        else
        {
            SetRandomTarget();
        }

        // Stop motors and apply brakes
        frontLeftWheel.motorTorque = 0f;
        frontRightWheel.motorTorque = 0f;
        rearLeftWheel.motorTorque = 0f;
        rearRightWheel.motorTorque = 0f;

        currentBrakeForce = brakeForce;
    }
    else
    {
        frontLeftWheel.motorTorque = motorForce;
        frontRightWheel.motorTorque = motorForce;
        rearLeftWheel.motorTorque = motorForce;
        rearRightWheel.motorTorque = motorForce;

        currentBrakeForce = 0f;
    }

    ApplyBraking();
}

    private Vector3 GetAvoidanceDirection()
    {
        Vector3 avoidance = Vector3.zero;
        float angleStep = 360f / lidarRays;

        for (int i = 0; i < lidarRays; i++)
        {
            float angle = i * angleStep;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;
            Ray ray = new Ray(transform.position + Vector3.up * 0.5f, dir);

            // Draw the ray in the Scene view (green if no hit, red if hit)
            if (Physics.Raycast(ray, out RaycastHit hit, lidarRange, obstacleLayer))
            {
                avoidance -= dir / (hit.distance + 0.1f);
                Debug.DrawRay(ray.origin, dir * hit.distance, Color.red); // Red for hit
            }
            else
            {
                Debug.DrawRay(ray.origin, dir * lidarRange, Color.green); // Green for no hit
            }
        }

        return avoidance.normalized;
    }


    private void ApplyBraking()
    {
        frontLeftWheel.brakeTorque = currentBrakeForce;
        frontRightWheel.brakeTorque = currentBrakeForce;
        rearLeftWheel.brakeTorque = currentBrakeForce;
        rearRightWheel.brakeTorque = currentBrakeForce;
    }

    private void HandleSteering()
    {
        Vector3 avoidanceDir = GetAvoidanceDirection();
        Vector3 targetDir = (targetPosition - transform.position).normalized;

        Vector3 finalDir = (targetDir + avoidanceDir * avoidanceStrength).normalized;

        if (Vector3.Distance(targetPosition, transform.position) < targetReachThreshold)
        {
            currentSteerAngle = 0f;
        }
        else
        {
            Vector3 localDir = transform.InverseTransformDirection(finalDir);
            float steerInput = Mathf.Atan2(localDir.x, localDir.z);
            targetSteerAngle = Mathf.Rad2Deg * steerInput;
            targetSteerAngle = Mathf.Clamp(targetSteerAngle, -maxSteerAngle, maxSteerAngle);
        }

        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetSteerAngle, Time.fixedDeltaTime * steeringSmoothness);

        frontLeftWheel.steerAngle = currentSteerAngle;
        frontRightWheel.steerAngle = currentSteerAngle;
    }
    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheel, frontLeftTransform);
        UpdateSingleWheel(frontRightWheel, frontRightTransform);
        UpdateSingleWheel(rearLeftWheel, rearLeftTransform);
        UpdateSingleWheel(rearRightWheel, rearRightTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.position = pos;
        wheelTransform.rotation = rot;
    }
}
