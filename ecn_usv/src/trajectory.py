#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from ecn_usv.srv import Target
import numpy as np

# ellipse params
a = 400.
b = 250.
x0 = -30.
y0 = -40
theta = 0.8


class Traj(Node):

    def __init__(self):
        super().__init__('trajectory')

        self.path = Path()
        self.path.header.frame_id = 'world'
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "path", qos_profile=latching_qos)

        ct = np.cos(theta)
        st = np.sin(theta)

        self.M = np.dot(np.array([[ct, -st, x0],[st,ct,y0],[0,0,1]]), np.diag([a,b,1]))
        self.Minv = np.linalg.inv(self.M)

        for t in np.linspace(-np.pi, np.pi, 100):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = self.angle2point(t)
            self.path.poses.append(pose)

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

        self.target_pub = self.create_publisher(PoseStamped, 'wp', 1)
        self.target = PoseStamped()
        self.target.header.frame_id = 'world'

        self.srv = self.create_service(Target, 'current_wp', self.target_cb)

    def angle2point(self, t):
        return np.dot(self.M, [np.cos(t), np.sin(t), 1]).flatten()[:2]

    def target_cb(self, req: Target.Request, res: Target.Response):

        # to ellipse frame
        x, y = np.dot(self.Minv, [req.x, req.y, 1]).flatten()[:2]
        t = np.arctan2(y, x) + 0.3

        res.x, res.y = self.angle2point(t)

        # also write current target
        self.target.pose.position.x = res.x
        self.target.pose.position.y = res.y

        x, y = self.angle2point(t+0.01)
        t = np.arctan2(y-res.y, x-res.x)
        self.target.pose.orientation.z = np.sin(t/2)
        self.target.pose.orientation.w = np.cos(t/2)

        self.target.header.stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(self.target)

        return res


rclpy.init()
node = Traj()
rclpy.spin(node)
node.destroy_node()
