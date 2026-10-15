#!/usr/bin/python3
"""定位真值评估: gz 真值 (world->base_link) vs 估计 (TF map->base_link).

map 系 = 建图起点的 base_link. world->map 的确定方式:
  align:=first (默认)  第一次拿到 map->base_link 时, 用当时的真值对齐 (建图模式下就是精确定义)
  align:=spawn         用 spawn_x/y/yaw (建图时的出生位姿, 与 sim.launch.py 的 x/y/yaw 一致),
                       评估重定位时用这个 (重定位结果与对齐无关)
发布:
  /ground_truth/path      真值轨迹 (map 系, RViz 可直接叠加)
  /localization_error     [dx, dy, dyaw_deg, |dxy|] (Float64MultiArray)
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GtEvaluator(Node):
    def __init__(self):
        super().__init__('gt_evaluator')
        self.sx = self.declare_parameter('spawn_x', 0.0).value
        self.sy = self.declare_parameter('spawn_y', 10.0).value
        self.syaw = self.declare_parameter('spawn_yaw', -1.5708).value
        self.align = self.declare_parameter('align', 'first').value
        self.aligned = self.align == 'spawn'
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.pub_path = self.create_publisher(Path, '/ground_truth/path', 1)
        self.pub_err = self.create_publisher(Float64MultiArray, '/localization_error', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 10)
        self.n = 0

    def world_to_map(self, x, y, yaw):
        dx, dy = x - self.sx, y - self.sy
        c, s = math.cos(-self.syaw), math.sin(-self.syaw)
        return c * dx - s * dy, s * dx + c * dy, math.remainder(yaw - self.syaw, 2 * math.pi)

    def align_first(self, gx, gy, gyaw, t):
        # T_world_map = T_world_base(gt) * T_map_base(est)^-1
        ex, ey, eyaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        self.syaw = math.remainder(gyaw - eyaw, 2 * math.pi)
        c, s = math.cos(self.syaw), math.sin(self.syaw)
        self.sx = gx - (c * ex - s * ey)
        self.sy = gy - (s * ex + c * ey)
        self.aligned = True
        self.get_logger().info(
            f'aligned world->map: x {self.sx:.3f} y {self.sy:.3f} yaw {math.degrees(self.syaw):.2f} deg')

    def on_gt(self, msg):
        p = msg.pose.pose
        if not self.aligned:
            try:
                t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            except TransformException:
                return
            self.align_first(p.position.x, p.position.y, yaw_of(p.orientation), t)
        mx, my, myaw = self.world_to_map(p.position.x, p.position.y, yaw_of(p.orientation))
        self.n += 1
        if self.n % 10 == 0:
            ps = PoseStamped()
            ps.header = msg.header
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = mx, my
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(myaw / 2), math.cos(myaw / 2)
            self.path.poses.append(ps)
            self.path.header.stamp = msg.header.stamp
            self.pub_path.publish(self.path)
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return
        ex = t.transform.translation.x - mx
        ey = t.transform.translation.y - my
        eyaw = math.degrees(math.remainder(yaw_of(t.transform.rotation) - myaw, 2 * math.pi))
        exy = math.hypot(ex, ey)
        self.pub_err.publish(Float64MultiArray(data=[ex, ey, eyaw, exy]))
        if self.n % 250 == 0:
            self.get_logger().info(f'loc error: xy {exy:.3f} m, yaw {eyaw:.2f} deg')


def main():
    rclpy.init()
    rclpy.spin(GtEvaluator())


if __name__ == '__main__':
    main()
