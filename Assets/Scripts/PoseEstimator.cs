using UnityEngine;

public class PoseEstimator : MonoBehaviour
{
    public Vector2 position = Vector2.zero; // (x, y) position on flat ground
    public float heading = 0f; // In degrees, 0 = facing forward

    public float wheelBase = 2f; // Distance between wheels
    public SimpleCarController drive;

    private void Update()
    {
        // You have to integrate manually if you don't have odometry
        float leftSpeed = drive.GetLeftSpeed();   // Custom method you must create
        float rightSpeed = drive.GetRightSpeed(); // Custom method you must create

        float linearVelocity = (leftSpeed + rightSpeed) / 2f;
        float angularVelocity = (rightSpeed - leftSpeed) / wheelBase;

        float dt = Time.deltaTime;

        heading += angularVelocity * Mathf.Rad2Deg * dt;
        heading = heading % 360f;

        Vector2 direction = new Vector2(Mathf.Cos(heading * Mathf.Deg2Rad), Mathf.Sin(heading * Mathf.Deg2Rad));
        position += direction * linearVelocity * dt;
    }
}
