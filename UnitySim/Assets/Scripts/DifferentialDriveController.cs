using UnityEngine;

public class SimpleDiffDriveController : MonoBehaviour
{
    public float linearSpeed = 2.0f;      // m/s
    public float angularSpeedRad = 1.5f;  // rad/s

    void Update()
    {
        float dt = Time.deltaTime;
        if (dt <= 0f) return;

        float v = 0f;
        float w = 0f;

        // Explicit key mapping to avoid Input Manager weirdness
        if (Input.GetKey(KeyCode.UpArrow))
            v += linearSpeed;
        if (Input.GetKey(KeyCode.DownArrow))
            v -= linearSpeed;
        if (Input.GetKey(KeyCode.LeftArrow))
            w += angularSpeedRad;   // Left = CCW (turn left)
        if (Input.GetKey(KeyCode.RightArrow))
            w -= angularSpeedRad;   // Right = CW (turn right)

        // Integrate yaw first
        float yawRad = transform.eulerAngles.y * Mathf.Deg2Rad;
        yawRad += w * dt;
        float yawDeg = yawRad * Mathf.Rad2Deg;

        // Move forward in local +X (transform.right)
        Vector3 deltaPos = transform.right * (v * dt);

        transform.position += deltaPos;
        transform.rotation = Quaternion.Euler(0f, yawDeg, 0f);
    }
}
