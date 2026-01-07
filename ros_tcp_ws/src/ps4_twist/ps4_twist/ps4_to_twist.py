#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def deadzone(x: float, dz: float) -> float:
    if abs(x) < dz:
        return 0.0
    # rescale so it doesn't "jump" at the edge of the deadzone
    return (x - dz) / (1.0 - dz) if x > 0 else (x + dz) / (1.0 - dz)


class PS4ToTwist(Node):
    """
    Subscribes to /joy (sensor_msgs/Joy) and publishes /cmd_vel (geometry_msgs/Twist).

    Default mappings assume a common SDL/joy layout:
      - Left stick vertical = axes[1] : forward/back
      - Right stick horizontal = axes[3] : yaw
      - R2 trigger = axes[5] (often 1..-1 when pressed) for "turbo" scaling (optional)
    If your axes differ, change parameters below without changing code.
    """

    def __init__(self):
        super().__init__("ps4_to_twist")

        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Axis indices (these are the most common defaults; adjust if needed)
        self.declare_parameter("axis_linear", 1)   # left stick up/down
        self.declare_parameter("axis_angular", 3)  # right stick left/right

        # Scaling
        self.declare_parameter("max_linear", 1.0)    # m/s
        self.declare_parameter("max_angular", 1.5)   # rad/s
        self.declare_parameter("deadzone", 0.08)

        # Optional turbo using R2 trigger (set to -1 to disable)
        self.declare_parameter("axis_turbo", 5)      # R2
        self.declare_parameter("turbo_min", 0.35)    # when not pressed
        self.declare_parameter("turbo_max", 1.00)    # when fully pressed
        self.declare_parameter("invert_linear", False)
        self.declare_parameter("invert_angular", False)

        joy_topic = self.get_parameter("joy_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.sub = self.create_subscription(Joy, joy_topic, self.on_joy, 10)

        self.get_logger().info(f"Listening: {joy_topic}  → Publishing: {cmd_vel_topic}")

    def on_joy(self, msg: Joy):
        axis_lin = int(self.get_parameter("axis_linear").value)
        axis_ang = int(self.get_parameter("axis_angular").value)
        dz = float(self.get_parameter("deadzone").value)

        max_lin = float(self.get_parameter("max_linear").value)
        max_ang = float(self.get_parameter("max_angular").value)

        inv_lin = bool(self.get_parameter("invert_linear").value)
        inv_ang = bool(self.get_parameter("invert_angular").value)

        # Guard for unexpected controller layouts
        if axis_lin >= len(msg.axes) or axis_ang >= len(msg.axes):
            self.get_logger().warn(
                f"Axis index out of range. axes_len={len(msg.axes)} "
                f"axis_linear={axis_lin} axis_angular={axis_ang}"
            )
            return

        lin = msg.axes[axis_lin]
        ang = msg.axes[axis_ang]

        if inv_lin:
            lin = -lin
        if inv_ang:
            ang = -ang

        lin = deadzone(lin, dz)
        ang = deadzone(ang, dz)

        # Turbo scaling from R2 trigger (optional)
        turbo_axis = int(self.get_parameter("axis_turbo").value)
        turbo = 1.0
        if turbo_axis >= 0 and turbo_axis < len(msg.axes):
            # Common PS4 trigger axis: 1.0 (released) → -1.0 (fully pressed)
            r2 = msg.axes[turbo_axis]
            # Map to 0..1 where 0 = released, 1 = pressed
            pressed = (1.0 - r2) * 0.5
            tmin = float(self.get_parameter("turbo_min").value)
            tmax = float(self.get_parameter("turbo_max").value)
            turbo = tmin + (tmax - tmin) * max(0.0, min(1.0, pressed))

        out = Twist()
        out.linear.x = lin * max_lin * turbo
        out.angular.z = ang * max_ang * turbo
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PS4ToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

