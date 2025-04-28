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

    // Autonomous control inputs
    [HideInInspector] public float autoThrottle = 0f;
    [HideInInspector] public float autoSteer = 0f;
    public bool useAutonomousControl = false;

    private void FixedUpdate()
    {
        if (useAutonomousControl)
        {
            HandleMotor(autoThrottle);
            HandleSteering(autoSteer);
        }
        else
        {
            HandleMotor(Input.GetAxis("Vertical"));
            HandleSteering(Input.GetAxis("Horizontal"));
        }
        UpdateWheels();
    }

    private void HandleMotor(float verticalInput)
    {
        isBraking = Input.GetKey(KeyCode.Space) && !useAutonomousControl;
        frontLeftWheel.motorTorque = verticalInput * motorForce;
        frontRightWheel.motorTorque = verticalInput * motorForce;
        rearLeftWheel.motorTorque = verticalInput * motorForce;
        rearRightWheel.motorTorque = verticalInput * motorForce;
        currentBrakeForce = isBraking ? brakeForce : 0f;
        ApplyBraking();
    }

    private void ApplyBraking()
    {
        frontLeftWheel.brakeTorque = currentBrakeForce;
        frontRightWheel.brakeTorque = currentBrakeForce;
        rearLeftWheel.brakeTorque = currentBrakeForce;
        rearRightWheel.brakeTorque = currentBrakeForce;
    }

    private void HandleSteering(float horizontalInput)
    {
        currentSteerAngle = maxSteerAngle * horizontalInput;
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
