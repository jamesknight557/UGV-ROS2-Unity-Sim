using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RoverPosePublisher : MonoBehaviour
{
    public string poseTopic = "rover/pose";
    public int publishRate = 10; // Hz
    
    private ROSConnection ros;
    private float timer = 0f;
    
    void Start()
    {
        // Setup ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        
        // Register position publisher
        ros.RegisterPublisher<PoseStampedMsg>(poseTopic);
    }
    
    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1.0f / publishRate)
        {
            PublishPose();
            timer = 0f;
        }
    }
    
    void PublishPose()
    {
        // Get current position and rotation
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;
        
        // Create pose message
        PoseStampedMsg poseMsg = new PoseStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "world"
            },
            pose = new PoseMsg
            {
                position = new PointMsg
                {
                    x = position.x,
                    y = position.y,
                    z = position.z
                },
                orientation = new QuaternionMsg
                {
                    x = rotation.x,
                    y = rotation.y,
                    z = rotation.z,
                    w = rotation.w
                }
            }
        };
        
        ros.Publish(poseTopic, poseMsg);
    }
}