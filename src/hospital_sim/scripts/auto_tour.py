#!/usr/bin/python3
"""建图用自动巡游: 用 gz 真值闭环沿走廊航点走一圈 (只用于采集建图数据, 不参与定位/导航).

  ros2 run hospital_sim auto_tour.py [--ros-args -p speed:=0.6]

航点在 world 系, 绕开了走廊里的轮椅/推床/候诊椅; 回程走 y=-14.5 横廊中线 (y=-8.5 横廊有拖把车 (1.08,-8.59), 右走廊 y>-7 有推床).
航点要离墙 >0.6m: 贴墙时墙上的点大多落进 blind/自遮挡框, LIO 会跳. 行进时车头朝向运动方向 (有转弯, 更考验 LIO).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

WAYPOINTS = [
    (0.0, 10.0), (-3.8, 8.5), (-4.3, 3.5), (-4.3, -8.7), (-4.6, -14.5), (-4.6, -24.5),
    (4.6, -24.5), (4.6, -14.5), (-4.4, -14.5), (-4.3, -8.7), (-4.3, 3.5), (-3.8, 8.5), (0.0, 10.0),
]


class AutoTour(Node):
    def __init__(self):
        super().__init__('auto_tour')
        self.speed = self.declare_parameter('speed', 0.6).value
        self.wz_max = self.declare_parameter('wz_max', 0.8).value
        self.i = 1
        self.pose = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)

    def tick(self):
        if self.pose is None:
            return
        cmd = Twist()
        if self.i >= len(WAYPOINTS):
            self.pub.publish(cmd)
            return
        x, y, yaw = self.pose
        gx, gy = WAYPOINTS[self.i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < 0.25:
            self.get_logger().info(f'reached waypoint {self.i}/{len(WAYPOINTS) - 1} ({gx}, {gy})')
            self.i += 1
            if self.i >= len(WAYPOINTS):
                self.get_logger().info('tour finished')
            return
        v = min(self.speed, 0.8 * dist)
        vx_w, vy_w = v * dx / dist, v * dy / dist
        c, s = math.cos(yaw), math.sin(yaw)
        cmd.linear.x = c * vx_w + s * vy_w          # world -> body
        cmd.linear.y = -s * vx_w + c * vy_w
        eyaw = math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)
        cmd.angular.z = max(-self.wz_max, min(self.wz_max, 1.5 * eyaw))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(AutoTour())


if __name__ == '__main__':
    main()
