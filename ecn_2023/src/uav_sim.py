#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
import numpy as np

dt = 0.05


def quaternion_msg(rpy):

    def cossin(angle):
        return np.cos(angle), np.sin(angle)

    cr,sr = cossin(rpy[0]/2)
    cp,sp = cossin(rpy[1]/2)
    cy,sy = cossin(rpy[2]/2)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class UAV(Node):

    def __init__(self):
        super().__init__('uav_sim')

        ns = self.get_namespace()[1:]

        self.br = TransformBroadcaster(self)
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'world'
        self.tf.child_frame_id = ns + '/base_link'

        self.tf.transform.translation.x = self.x0 = 10*float(ns[-1])
        self.tf.transform.translation.z = self.z0 = 20.+0.5*float(ns[-1])

        self.rpy = [0,0,0]

        self.cmd = None
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 1)

        self.timer = self.create_timer(dt, self.update)
        self.last = self.now()

    def now(self):
        return self.get_clock().now().seconds_nanoseconds()[0]

    def cmd_cb(self, msg):
        self.cmd = msg
        self.last = self.now()

    def update(self):

        c, s = np.cos(self.rpy[2]), np.sin(self.rpy[2])

        if self.cmd is not None and self.now() - self.last < 2.:
            vx = np.clip(self.cmd.linear.x, -200, 200)
            vy = np.clip(self.cmd.linear.y, -200, 200)
            w = self.cmd.angular.z

            self.rpy[0], self.rpy[1] = -vy * 0.005, vx * 0.005

            vx, vy = c*vx-s*vy, s*vx+c*vy
            vz = self.cmd.linear.z
        else:
            vx = 0.5*(self.x0 - self.tf.transform.translation.x)
            vy = -0.5*self.tf.transform.translation.y
            vz = 0.5*(self.z0 - self.tf.transform.translation.z)
            self.rpy[0], self.rpy[1] = -vy * 0.005, vx * 0.005
            w = -0.1*self.rpy[2]

        self.rpy[2] += w * dt
        self.tf.transform.translation.x += vx*dt
        self.tf.transform.translation.y += vy*dt
        self.tf.transform.translation.z += vz*dt

        self.tf.transform.rotation = quaternion_msg(self.rpy)
        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.tf)


rclpy.init()
node = UAV()
rclpy.spin(node)
node.destroy_node()
