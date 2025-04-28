using UnityEngine;

public class SimpleCarController : MonoBehaviour
{
    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel;
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;

    public float maxMotorTorque = 400f; // Increased from 150f for better movement
    public float brakeTorque = 300f;
    
    // Debug variables
    public bool debugMode = true;
    private float leftTorque, rightTorque;
    
    // Reference to rigidbody for physics checks
    private Rigidbody carRigidbody;

    private void Start()
    {
        // Get rigidbody reference
        carRigidbody = GetComponent<Rigidbody>();
        if (carRigidbody == null)
        {
            Debug.LogError("No Rigidbody found on car! Adding one...");
            carRigidbody = gameObject.AddComponent<Rigidbody>();
            carRigidbody.mass = 1000f; // Set appropriate mass for a car
        }
        
        // Set default wheel friction
        SetupWheelFriction(frontLeftWheel);
        SetupWheelFriction(frontRightWheel);
        SetupWheelFriction(rearLeftWheel);
        SetupWheelFriction(rearRightWheel);
        
        // Ensure wheels are properly set up
        if (frontLeftWheel == null || frontRightWheel == null || rearLeftWheel == null || rearRightWheel == null)
        {
            Debug.LogError("One or more wheel colliders are missing! Car won't move properly.");
        }
        
        // Check if wheels are touching the ground
        CheckWheelContacts();
        
        if (debugMode)
        {
            Debug.Log("SimpleCarController initialized with max torque: " + maxMotorTorque);
        }
    }
    
    private void Update()
    {
        // Display current torque values in inspector when in debug mode
        if (debugMode && (leftTorque != 0 || rightTorque != 0) && Time.frameCount % 60 == 0)
        {
            Debug.Log($"Wheel torques - Left: {leftTorque}, Right: {rightTorque}, " +
                     $"RPMs: L:{frontLeftWheel.rpm:F1}, R:{frontRightWheel.rpm:F1}, " +
                     $"Velocity: {carRigidbody.velocity.magnitude:F2}");
            
            // Check if wheels are actually touching the ground
            CheckWheelContacts();
        }
        
        // Apply wheel rotations for visual feedback
        ApplyWheelTransforms();
    }

    private void SetupWheelFriction(WheelCollider wheel)
    {
        if (wheel == null) return;
        
        // Configure wheel friction for better movement
        WheelFrictionCurve forwardFriction = wheel.forwardFriction;
        forwardFriction.stiffness = 2.0f;  // Increased from 1.5f for better grip
        wheel.forwardFriction = forwardFriction;
        
        WheelFrictionCurve sidewaysFriction = wheel.sidewaysFriction;
        sidewaysFriction.stiffness = 2.0f;  // Increased from 1.5f for better grip
        wheel.sidewaysFriction = sidewaysFriction;
        
        // Adjust wheel suspension for better ground contact
        JointSpring suspension = wheel.suspensionSpring;
        suspension.spring = 35000f;
        suspension.damper = 4500f;
        wheel.suspensionSpring = suspension;
        
        // Ensure proper suspension distance
        wheel.suspensionDistance = 0.3f;
    }
    
    // Check if wheels are touching the ground
    private void CheckWheelContacts()
    {
        if (debugMode)
        {
            Debug.Log($"Wheel ground contacts - FL: {IsWheelGrounded(frontLeftWheel)}, " +
                      $"FR: {IsWheelGrounded(frontRightWheel)}, " +
                      $"RL: {IsWheelGrounded(rearLeftWheel)}, " +
                      $"RR: {IsWheelGrounded(rearRightWheel)}");
        }
    }
    
    // Check if a wheel is in contact with the ground
    public bool IsWheelGrounded(WheelCollider wheel)
    {
        if (wheel == null) return false;
        
        WheelHit hit;
        return wheel.GetGroundHit(out hit);
    }
    
    // Apply wheel transforms to visual wheel meshes (if you have them)
    private void ApplyWheelTransforms()
    {
        // This is where you would update visual wheel meshes if you have them
    }

    private void SetWheelTorque(WheelCollider wheel, float torque)
    {
        if (wheel == null) return;
        
        // Reset brake torque if we're moving
        if (Mathf.Abs(torque) > 0.01f)
        {
            wheel.brakeTorque = 0;
        }
        
        // Apply motor torque
        wheel.motorTorque = torque;
        
        // Apply brake if we're trying to stop
        if (Mathf.Abs(torque) < 0.01f && wheel.rpm != 0)
        {
            wheel.brakeTorque = brakeTorque;
        }
    }

    public void SetWheelSpeeds(float leftSpeed, float rightSpeed)
    {
        // Calculate torques from speeds
        leftTorque = leftSpeed * maxMotorTorque;
        rightTorque = rightSpeed * maxMotorTorque;

        // Apply to wheels
        // Left side
        SetWheelTorque(frontLeftWheel, leftTorque);
        SetWheelTorque(rearLeftWheel, leftTorque);

        // Right side
        SetWheelTorque(frontRightWheel, rightTorque);
        SetWheelTorque(rearRightWheel, rightTorque);
        
        if (debugMode && (Mathf.Abs(leftSpeed) > 0.01f || Mathf.Abs(rightSpeed) > 0.01f))
        {
            Debug.Log($"Setting wheel speeds: Left={leftSpeed:F2} (torque: {leftTorque:F0}), " +
                      $"Right={rightSpeed:F2} (torque: {rightTorque:F0})");
        }
    }

    public float GetLeftSpeed()
    {
        // Ensure wheel is valid before accessing its properties
        if (frontLeftWheel == null) return 0;
        return frontLeftWheel.rpm * frontLeftWheel.radius * Mathf.PI / 30f; // Convert RPM to m/s
    }
    
    public float GetRightSpeed()
    {
        // Ensure wheel is valid before accessing its properties
        if (frontRightWheel == null) return 0;
        return frontRightWheel.rpm * frontRightWheel.radius * Mathf.PI / 30f; // Convert RPM to m/s
    }
    
    // Test movement with direct keyboard input
    public void ManualControl(float leftInput, float rightInput)
    {
        SetWheelSpeeds(leftInput, rightInput);
    }
}
