#!/usr/bin/python3
"""45° X 型全向轮逆运动学: /cmd_vel (vx, vy, wz) -> 4 个轮速.

仿真里底盘由 gz VelocityControl 直接给 body 速度 (理想全向运动), 本节点负责:
  - 模拟电机动态: /cmd_vel 做加速度限幅 + 超时停车 -> /sim/cmd_vel_actual (桥接到 gz)
    (速度阶跃会让 IMU 出现几百 m/s^2 的尖峰, 真车不会这样)
  - 把轮速积分成关节角, 发 /joint_states 让 RViz 里的轮子转起来
  - 发 /wheel_speeds (rad/s, 顺序 FL FR RL RR), 与真机下发给电机的量一致, 方便对照

轮子在底盘四角 r_i=(x_i, y_i), 轮轴沿径向(指向车体中心), 滚动方向为切向
  d_i = (-y_i, x_i) / |r_i|
  轮缘线速度 v_i = d_i · (v + w × r_i) = d_i · v + w |r_i|
  轮角速度   ω_i = v_i / R
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class OmniWheelKinematics(Node):
    def __init__(self):
        super().__init__('omni_wheel_kinematics')
        self.R = self.declare_parameter('wheel_radius', 0.075).value
        a = self.declare_parameter('wheel_offset', 0.1785).value  # 轮心到车体中心在 x/y 上的距离
        # (关节名, x, y) —— 与 chassis.urdf.xacro 一致
        self.wheels = [
            ('wheel_front_left', a, a),
            ('wheel_front_right', a, -a),
            ('wheel_rear_left', -a, a),
            ('wheel_rear_right', -a, -a),
        ]
        self.acc_lim = self.declare_parameter('accel_limits', [2.0, 2.0, 3.0]).value  # vx vy wz
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.angles = [0.0] * 4
        self.target = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0]
        self.last = self.get_clock().now()
        self.last_cmd = self.last
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, '/sim/cmd_vel_actual', 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_ws = self.create_publisher(Float64MultiArray, 'wheel_speeds', 10)
        self.create_timer(0.01, self.tick)

    def on_cmd(self, msg):
        self.target = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_cmd = self.get_clock().now()

    def wheel_speeds(self, vx, vy, wz):
        out = []
        for _, x, y in self.wheels:
            r = math.hypot(x, y)
            dx, dy = -y / r, x / r
            out.append((dx * vx + dy * vy + wz * r) / self.R)
        return out

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.0
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.target = [0.0, 0.0, 0.0]
        for k in range(3):
            step = self.acc_lim[k] * dt
            self.actual[k] += max(-step, min(step, self.target[k] - self.actual[k]))
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = self.actual
        self.pub_cmd.publish(out)

        w = self.wheel_speeds(*self.actual)
        for i in range(4):
            self.angles[i] = math.remainder(self.angles[i] + w[i] * dt, 2 * math.pi)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [n for n, _, _ in self.wheels]
        js.position = list(self.angles)
        js.velocity = w
        self.pub_js.publish(js)
        self.pub_ws.publish(Float64MultiArray(data=w))


def main():
    rclpy.init()
    rclpy.spin(OmniWheelKinematics())


if __name__ == '__main__':
    main()
