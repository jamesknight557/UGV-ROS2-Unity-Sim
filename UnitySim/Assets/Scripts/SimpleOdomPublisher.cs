using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using System;

public class SimpleOdomPublisher : MonoBehaviour
{
    public string odomTopic = "/odom";
    public float publishRateHz = 30f;

    ROSConnection ros;

    Vector3 lastPos;
    float lastYaw;     // radians
    float lastTime;    // seconds
    float nextPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odomTopic);

        lastPos = transform.position;
        lastYaw = GetUnityYaw();
        lastTime = Time.time;
        nextPublishTime = 0f;
    }

    void FixedUpdate()
    {
        float now = Time.time;
        if (now < nextPublishTime)
            return;

        float dt = now - lastTime;
        if (dt <= 0f)
        {
            lastTime = now;
            return;
        }

        nextPublishTime = now + 1.0f / publishRateHz;

        Vector3 pos = transform.position;
        float yaw = GetUnityYaw(); // radians

        // Unity: X (forward), Z (left), Y (up)
        // ROS:   x (forward), y (left), z (up)
        double rosX = pos.x;
        double rosY = pos.z;
        double rosZ = 0.0;

        // World-space velocity
        Vector3 dp = (pos - lastPos) / dt;

        // Linear velocity in base_link x
        float v = Vector3.Dot(dp, transform.right);

        // Yaw rate (rad/s)
        float dyaw = Mathf.DeltaAngle(lastYaw * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg) * Mathf.Deg2Rad;
        float w = dyaw / dt;

        QuaternionMsg q = YawToQuaternionMsg(yaw);

        // Timestamp
        int secs = (int)Math.Floor(now);
        uint nsecs = (uint)((now - secs) * 1e9);
        TimeMsg stamp = new TimeMsg(secs, nsecs);

        OdometryMsg odom = new OdometryMsg();
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position = new PointMsg(rosX, rosY, rosZ);
        odom.pose.pose.orientation = q;

        odom.twist.twist.linear = new Vector3Msg(v, 0.0, 0.0);
        odom.twist.twist.angular = new Vector3Msg(0.0, 0.0, w);

        ros.Publish(odomTopic, odom);

        lastPos = pos;
        lastYaw = yaw;
        lastTime = now;
    }

    float GetUnityYaw()
    {
        // yaw about +Y, increases when we press LeftArrow
        return transform.eulerAngles.y * Mathf.Deg2Rad;
    }

    QuaternionMsg YawToQuaternionMsg(float yaw)
    {
        double half = yaw * 0.5;
        double cz = Math.Cos(half);
        double sz = Math.Sin(half);
        // rotation about ROS +Z
        return new QuaternionMsg(0.0, 0.0, sz, cz);
    }
}
