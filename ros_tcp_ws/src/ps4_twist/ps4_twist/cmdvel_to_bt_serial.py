#!/usr/bin/env python3

import math
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_bt_serial')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('port', '/dev/rfcomm0')
        self.declare_parameter('baud', 9600)

        self.declare_parameter('track_width', 0.32)   # meters (center-to-center)
        self.declare_parameter('max_lin', 0.25)       # m/s -> full PWM
        self.declare_parameter('max_ang', 2.0)        # rad/s clamp
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('min_pwm', 110)        # overcome static friction
        self.declare_parameter('deadband', 0.01)      # cmd_vel deadband

        # ---------------- LOAD PARAMETERS ----------------
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.track_width = float(self.get_parameter('track_width').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.deadband = float(self.get_parameter('deadband').value)

        # ---------------- SERIAL ----------------
        self.get_logger().info(f"Opening serial {self.port} @ {self.baud}")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)

        # ---------------- ROS ----------------
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.2, self.watchdog)

        self.get_logger().info("cmdvel_to_bt_serial READY")

    # --------------------------------------------------
    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # --------------------------------------------------
    def apply_min_pwm(self, pwm):
        if abs(pwm) < 5:
            return 0
        if abs(pwm) < self.min_pwm:
            return int(math.copysign(self.min_pwm, pwm))
        return pwm

    # --------------------------------------------------
    def vel_to_pwm(self, v_left, v_right):
        scale = max(self.max_lin, 1e-3)

        l = int((v_left / scale) * self.max_pwm)
        r = int((v_right / scale) * self.max_pwm)

        l = self.clamp(l, -self.max_pwm, self.max_pwm)
        r = self.clamp(r, -self.max_pwm, self.max_pwm)

        l = self.apply_min_pwm(l)
        r = self.apply_min_pwm(r)

        return l, r

    # --------------------------------------------------
    def cmd_vel_cb(self, msg: Twist):
        v = float(msg.linear.x) * -1
        w = float(msg.angular.z) * -1

        # Deadband
        if abs(v) < self.deadband:
            v = 0.0
        if abs(w) < self.deadband:
            w = 0.0


        v = self.clamp(v, -self.max_lin, self.max_lin)
        w = self.clamp(w, -self.max_ang, self.max_ang)

        # Differential / tracked kinematics
        v_left  = v - (w * self.track_width / 2.0)
        v_right = v + (w * self.track_width / 2.0)

        l_pwm, r_pwm = self.vel_to_pwm(v_left, v_right)

        line = f"D,{l_pwm},{r_pwm}\n"

        try:
            self.ser.write(line.encode('ascii'))
            self.last_cmd_time = time.time()
            self.get_logger().info(
                f"cmd_vel v={v:.2f} w={w:.2f} -> L={l_pwm} R={r_pwm}"
            )
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # --------------------------------------------------
    def watchdog(self):
        # Stop if cmd_vel stops arriving
        if time.time() - self.last_cmd_time > 0.5:
            try:
                self.ser.write(b"D,0,0\n")
            except Exception:
                pass


def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

