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

    public Transform target; // Target yang mau dituju robot
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

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();
    }

    private void HandleMotor()
    {
        if (target == null)
            return;

        float distanceToTarget = Vector3.Distance(target.position, transform.position);
        
        // Check if we've reached the target
        if (distanceToTarget <= targetReachThreshold)
        {
            // Stop motors and apply brakes
            frontLeftWheel.motorTorque = 0f;
            frontRightWheel.motorTorque = 0f;
            rearLeftWheel.motorTorque = 0f;
            rearRightWheel.motorTorque = 0f;
            
            currentBrakeForce = brakeForce;
        }
        else
        {
            // Moving toward target
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
        if (target == null)
            return;

        Vector3 avoidanceDir = GetAvoidanceDirection();
        Vector3 targetDir = (target.position - transform.position).normalized;

        Vector3 finalDir = (targetDir + avoidanceDir * avoidanceStrength).normalized;

        if (Vector3.Distance(target.position, transform.position) < targetReachThreshold)
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
