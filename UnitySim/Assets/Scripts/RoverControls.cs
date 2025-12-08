
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimpleRoverController : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float turnSpeed = 100f;
    public string cmdVelTopic = "cmd_vel";
    
    private Rigidbody rb;
    private ROSConnection ros;
    private float rosLinear = 0f;
    private float rosAngular = 0f;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        
        // Setup ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveCmdVel);
    }
    
    void ReceiveCmdVel(TwistMsg msg)
    {
        // Store the ROS command velocities
        rosLinear = (float)msg.linear.x;
        rosAngular = (float)msg.angular.z;
    }
    
    void FixedUpdate()
    {
        // Get Unity keyboard input
        float unityMove = Input.GetAxis("Vertical");   // W/S or Up/Down arrows
        float unityTurn = Input.GetAxis("Horizontal"); // A/D or Left/Right arrows
        
        // Combine Unity input with ROS input (either can control)
        float move = unityMove + rosLinear;
        float turn = unityTurn + rosAngular;
        
        // Clamp to prevent excessive speed when both inputs are active
        move = Mathf.Clamp(move, -1f, 1f);
        turn = Mathf.Clamp(turn, -1f, 1f);
        
        // Move forward/backward
        Vector3 moveDirection = transform.forward * move * moveSpeed * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + moveDirection);
        
        // Rotate left/right
        Quaternion turnRotation = Quaternion.Euler(0f, turn * turnSpeed * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }
}