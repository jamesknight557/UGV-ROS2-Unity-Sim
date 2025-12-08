using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class UGVBaseController : MonoBehaviour
{
    public float linearSpeed = 2.0f;   // m/s
    public float angularSpeed = 60f;   // deg/s

    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezePositionY |
                         RigidbodyConstraints.FreezeRotationX |
                         RigidbodyConstraints.FreezeRotationZ;
    }

    void FixedUpdate()
    {
        float v = Input.GetAxis("Vertical");   // Up/Down arrows
        float w = Input.GetAxis("Horizontal"); // Left/Right arrows

        // Forward is +X
        Vector3 vel = transform.right * (v * linearSpeed); // right == +X
        float yawRate = -w * angularSpeed; // Right arrow -> negative yaw (clockwise)

        // Integrate manually for clean odom
        Vector3 newPos = rb.position + vel * Time.fixedDeltaTime;
        Quaternion yawRot = Quaternion.Euler(0f, yawRate * Time.fixedDeltaTime, 0f);
        Quaternion newRot = rb.rotation * yawRot;

        rb.MovePosition(newPos);
        rb.MoveRotation(newRot);
    }
}
