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
    public float speedMultiplier = 0.5f; // Increased to make movement smoother

    private float currentSteerAngle;
    private float currentBrakeForce;
    private bool isBraking;

    private float aiVerticalInput = 0f;
    private float aiHorizontalInput = 0f;
    private bool useAIInput = false;
    private bool isReversing = false;
    private float reverseTimer = 0f;

    public void SetInputs(float vertical, float horizontal)
    {
        aiVerticalInput = vertical;
        aiHorizontalInput = horizontal;
        useAIInput = true;
        
        // Track if we're reversing for better wheel handling
        if (vertical < -0.1f)
        {
            isReversing = true;
            reverseTimer = 0f;
        }
        else if (isReversing && vertical > 0.1f)
        {
            // Add a small delay when changing from reverse to forward
            reverseTimer += Time.fixedDeltaTime;
            if (reverseTimer > 0.2f)
            {
                isReversing = false;
            }
        }
        
        // Only log input changes when they change significantly and not too often
        if ((Mathf.Abs(vertical) > 0.1f || Mathf.Abs(horizontal) > 0.1f) && Time.frameCount % 120 == 0)
        {
            Debug.Log($"Car inputs: vertical={vertical}, horizontal={horizontal}, reversing={isReversing}");
        }
    }

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();
    }

    private void HandleMotor()
    {
        float verticalInput = useAIInput ? aiVerticalInput : Input.GetAxis("Vertical");
        isBraking = useAIInput ? false : Input.GetKey(KeyCode.Space);

        // Apply speed reduction but with better handling for reversing
        float effectiveMotorForce = motorForce * speedMultiplier;
        
        // Add a bit more force when starting from standstill
        if (GetComponent<Rigidbody>().velocity.magnitude < 0.5f)
        {
            effectiveMotorForce *= 1.5f;
        }

        // Apply torque to all wheels for better traction
        frontLeftWheel.motorTorque = verticalInput * effectiveMotorForce;
        frontRightWheel.motorTorque = verticalInput * effectiveMotorForce;
        rearLeftWheel.motorTorque = verticalInput * effectiveMotorForce;
        rearRightWheel.motorTorque = verticalInput * effectiveMotorForce;
        
        // Apply light braking when changing direction to prevent sliding
        if ((verticalInput > 0.1f && GetComponent<Rigidbody>().velocity.magnitude > 0.1f && 
             Vector3.Dot(transform.forward, GetComponent<Rigidbody>().velocity) < -0.1f) ||
            (verticalInput < -0.1f && GetComponent<Rigidbody>().velocity.magnitude > 0.1f && 
             Vector3.Dot(transform.forward, GetComponent<Rigidbody>().velocity) > 0.1f))
        {
            // Apply gentle braking when changing direction
            currentBrakeForce = brakeForce * 0.6f;
        }
        else
        {
            currentBrakeForce = isBraking ? brakeForce : 0f;
        }
        
        ApplyBraking();
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
        float horizontalInput = useAIInput ? aiHorizontalInput : Input.GetAxis("Horizontal");
        
        // Apply more aggressive steering at low speeds for better maneuverability
        float speedFactor = Mathf.Clamp01(GetComponent<Rigidbody>().velocity.magnitude / 5.0f);
        float effectiveSteerAngle = maxSteerAngle * (1.0f + (1.0f - speedFactor) * 0.3f);
        
        currentSteerAngle = effectiveSteerAngle * horizontalInput;
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
