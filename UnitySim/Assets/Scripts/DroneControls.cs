using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimpleDroneController : MonoBehaviour
{
    public float moveSpeed = 10f;
    public float turnSpeed = 100f;
    public float verticalSpeed = 5f;
    public float tiltAmount = 15f;
    public float tiltSpeed = 5f;
    
    public string cmdVelTopic = "cmd_vel";
    
    private Rigidbody rb;
    private ROSConnection ros;
    private float rosLinear = 0f;
    private float rosAngular = 0f;
    
    private Transform droneModel;
    private float currentTilt = 0f;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        
        // Find the drone model child
        droneModel = transform.GetChild(0);
        
        // Apply base rotation to model
        if (droneModel != null)
        {
            droneModel.localRotation = Quaternion.Euler(270f, 90f, 0f);
        }
        
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveCmdVel);
    }
    
    void ReceiveCmdVel(TwistMsg msg)
    {
        rosLinear = (float)msg.linear.x;
        rosAngular = (float)msg.angular.z;
    }
    
    void FixedUpdate()
    {
        // Get input - Up/Down arrows for forward/back
        float move = 0f;
        if (Input.GetKey(KeyCode.UpArrow)) move = 1f;
        if (Input.GetKey(KeyCode.DownArrow)) move = -1f;
        
        // Left/Right arrows for yaw
        float turn = 0f;
        if (Input.GetKey(KeyCode.LeftArrow)) turn = -1f;
        if (Input.GetKey(KeyCode.RightArrow)) turn = 1f;
        
        // Vertical - Space up, Shift down
        float vertical = 0f;
        if (Input.GetKey(KeyCode.Space)) vertical = 1f;
        if (Input.GetKey(KeyCode.LeftShift)) vertical = -1f;
        
        // Combine with ROS
        move = Mathf.Clamp(move + rosLinear, -1f, 1f);
        turn = Mathf.Clamp(turn + rosAngular, -1f, 1f);
        
        // Move forward/backward
        Vector3 moveDirection = transform.forward * move * moveSpeed * Time.fixedDeltaTime;
        moveDirection.y = vertical * verticalSpeed * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + moveDirection);
        
        // Yaw rotation
        Quaternion turnRotation = Quaternion.Euler(0f, turn * turnSpeed * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);

        // Apply tilt to the model
        if (droneModel != null)
        {
            // Calculate target tilt based on forward/back movement
            float targetTilt = -move * tiltAmount;
            currentTilt = Mathf.Lerp(currentTilt, targetTilt, tiltSpeed * Time.fixedDeltaTime);
            
            // Apply tilt while keeping base rotation - tilt on X axis
            droneModel.localRotation = Quaternion.Euler(currentTilt - 90f,  90f , 0f );
        }
    }
}