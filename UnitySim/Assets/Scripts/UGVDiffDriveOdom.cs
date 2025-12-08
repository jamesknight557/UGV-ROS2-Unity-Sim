using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using System;
using RosMessageTypes.Geometry; // For TwistMsg
using System;


public class UGVDiffDriveOdom : MonoBehaviour
{
    [Header("Motion")]
    public float linearSpeed = 2.0f;      // m/s
    public float angularSpeed = 1.5f;     // rad/s

    [Header("Odom")]
    public string odomTopic = "/odom";
    public float odomRateHz = 30f;

    private ROSConnection ros;

    // Robot pose in a 2D odom frame (ROS convention)
    private float x;       // ROS x (forward)
    private float y;       // ROS y (left)
    private float theta;   // yaw in radians, CCW positive

    // Commanded velocities (base_link frame)
    private float vCmd;    // linear x
    private float wCmd;    // angular z

    private float nextOdomTime = 0f;
    
    private bool useCmdVel = false;
    private float cmdVelV = 0f;   // linear x from ROS
    private float cmdVelW = 0f;   // angular z from ROS


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odomTopic);
        
        ros.Subscribe<TwistMsg>("/cmd_vel", CmdVelCallback);


        // Initialise pose from current Unity transform
        Vector3 p = transform.position;
        x = p.x;     // Unity X → ROS x
        y = p.z;     // Unity Z → ROS y

        // Start with heading from current rotation (but we’ll own it from now on)
        theta = 0f;  // assume facing +X initially; we’ll overwrite transform below

        // Force Unity transform to match our internal state
        transform.position = new Vector3(x, 0f, y);
        transform.rotation = Quaternion.Euler(0f, 0f, 0f);

        nextOdomTime = 0f;
    }
    
    void CmdVelCallback(TwistMsg msg)
    {
	// ROS uses: linear.x (forward), angular.z (yaw CCW)
	cmdVelV = (float)msg.linear.x;
	cmdVelW = (float)msg.angular.z;
	useCmdVel = true; // start using ROS commands
    }


    void Update()
    {
        float dt = Time.deltaTime;
        if (dt <= 0f) return;

        if (useCmdVel)
	{
	    // use ROS commands
	    vCmd = cmdVelV;
	    wCmd = cmdVelW;
	}
	else
	{
	    // fallback to keyboard (nice for testing)
	    vCmd = 0f;
	    wCmd = 0f;

	    if (Input.GetKey(KeyCode.UpArrow))
		vCmd += linearSpeed;
	    if (Input.GetKey(KeyCode.DownArrow))
		vCmd -= linearSpeed;
	    if (Input.GetKey(KeyCode.LeftArrow))
		wCmd += angularSpeed;  // CCW
	    if (Input.GetKey(KeyCode.RightArrow))
		wCmd -= angularSpeed;  // CW
	}


        // --- Integrate unicycle model in odom frame ---
        theta += wCmd * dt;  // CCW+ in math

        float cosTh = Mathf.Cos(theta);
        float sinTh = Mathf.Sin(theta);

        // x forward, y left
        x += vCmd * cosTh * dt;
        y += vCmd * sinTh * dt;

        // --- Apply pose to Unity transform ---
        // Map ROS (x,y) → Unity (X,Z)
        transform.position = new Vector3(x, 0f, y);

        // IMPORTANT: positive theta (CCW) should look like a left turn in Unity
        // Unity's positive Y rotation appears as right turn for your model,
        // so we use -theta here to flip it visually.
        float yawDegUnity = -theta * Mathf.Rad2Deg;
        transform.rotation = Quaternion.Euler(0f, yawDegUnity, 0f);

        // --- Publish odom at desired rate ---
        PublishOdomIfNeeded();
    }

    void PublishOdomIfNeeded()
    {
        float nowUnity = Time.time;   // still fine to use for rate limiting
        if (nowUnity < nextOdomTime) return;
        nextOdomTime = nowUnity + 1.0f / odomRateHz;

        // --- Timestamp using real wall-clock time ---
        DateTimeOffset now = DateTimeOffset.UtcNow;
        long secsLong = now.ToUnixTimeSeconds();
        long nsecsLong = (now.ToUnixTimeMilliseconds() % 1000) * 1000000L;

        int secs = (int)secsLong;
        uint nsecs = (uint)nsecsLong;
        TimeMsg stamp = new TimeMsg(secs, nsecs);

        // Pose in ROS
        PointMsg pos = new PointMsg(x, y, 0.0);

        // Quaternion for yaw about +Z using *theta* (not -theta)
        double half = theta * 0.5;
        double cz = Math.Cos(half);
        double sz = Math.Sin(half);
        QuaternionMsg q = new QuaternionMsg(0.0, 0.0, sz, cz);

        OdometryMsg odom = new OdometryMsg();
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position = pos;
        odom.pose.pose.orientation = q;

        // Twist in base_link
        odom.twist.twist.linear = new Vector3Msg(vCmd, 0.0, 0.0);
        odom.twist.twist.angular = new Vector3Msg(0.0, 0.0, wCmd);

        ros.Publish(odomTopic, odom);
    }
}
