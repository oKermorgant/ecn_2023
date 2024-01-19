#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ecn_usv.srv import Target
import numpy as np
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

dt = 0.02


class USV(Node):

    def __init__(self):
        super().__init__('usv_sim')

        self.js = JointState()
        self.js.name = ['support', 'blades']
        self.js.position = [0.,0.]
        self.js_pub = self.create_publisher(JointState, 'joint_states', 1)

        self.br = TransformBroadcaster(self)
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'world'
        self.tf.child_frame_id = 'usv/base_link'

        self.target = self.create_client(Target, 'current_wp')
        self.target.wait_for_service()
        self.get_logger().info('Got trajectory service')

        self.req = Target.Request()
        self.res = None

        # control gains
        def declare_gain(name, default):
            gain = ParameterDescriptor(
                name = name,
                floating_point_range = [FloatingPointRange(
                    from_value = 0.,
                    to_value = 10.)])
            param = self.declare_parameter(gain.name, default, gain)
            setattr(self, name, param.value)
        declare_gain('Kv', 5.)
        declare_gain('Kw', .6)

        self.add_on_set_parameters_callback(self.cb_params)

        # dynamics
        self.p = np.matrix([300,0.,2.]).T
        self.v = np.matrix([0.,0.,0.]).T
        self.Minv = np.linalg.inv(np.matrix(np.diag([10, 10, 5])))
        self.d = np.matrix(np.diag([2, 50, 15]))
        self.R = np.eye(3)

        self.timer = self.create_timer(dt, self.update)

    def cb_params(self, params):
        accept = False
        for param in params:
            if param.name == 'Kv':
                if param.value > 0:
                    self.Kv = param.value
                    accept = True
            elif param.name == 'Kw':
                if param.value > 0:
                    self.Kw = param.value
                    accept = True

        return SetParametersResult(successful=accept)

    def cmd_callback(self, msg):
        self.cmd = msg

    def x(self):
        return self.p[0,0]

    def y(self):
        return self.p[1,0]

    def theta(self):
        return self.p[2,0]

    def update(self):

        if self.res is None:

            # get current target
            self.req.x = self.x()
            self.req.y = self.y()
            self.res = self.target.call_async(self.req)
            return
        elif not self.res.done():
            return

        c, s = np.cos(self.theta()), np.sin(self.theta())
        self.R[0,0] = self.R[1,1] = c
        self.R[0,1] = -s
        self.R[1,0] = s

        # compute control
        dx = self.res.result().x - self.x()
        dy = self.res.result().y - self.y()
        dx, dy, _ = np.dot(self.R.T, [dx, dy, 0]).flatten()
        dtheta = np.arctan2(dy, dx)

        # cmd to joints
        a = self.js.position[0]

        self.js.position[0] = np.clip(-self.Kw * dtheta, max(-1.5, a-0.1), min(1.5, a+0.1))
        self.js.position[1] += -10.*dt

        # joints to force
        f = np.matrix([0,0,0]).T
        c, s = np.cos(self.js.position[0]), np.sin(self.js.position[0])
        f[0] = self.Kv*dx*c
        f[1] = -self.Kv*dx*s
        f[2] = -dx*self.js.position[0]/2

        # force to vel
        self.v += self.Minv * (-self.d * self.v + f) * dt
        # vel to pose
        self.p += np.dot(self.R, self.v) * dt

        self.tf.transform.translation.x = self.x()
        self.tf.transform.translation.y = self.y()
        self.tf.transform.rotation.z = np.sin(self.theta()/2)
        self.tf.transform.rotation.w = np.cos(self.theta()/2)
        self.tf.header.stamp = self.js.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.tf)

        self.js_pub.publish(self.js)

        self.res = None


rclpy.init()
node = USV()
rclpy.spin(node)
node.destroy_node()
