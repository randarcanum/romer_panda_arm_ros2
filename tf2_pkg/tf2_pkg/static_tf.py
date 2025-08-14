#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R

from romer_interfaces.srv import JointToCartesian
from sensor_msgs.msg import JointState
import numpy

class JointToCartClientAsync(Node):

    def __init__(self):
        super().__init__('joint_to_cart_client')
        self.cli = self.create_client(JointToCartesian, 'joint_to_cartesian')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = JointToCartesian.Request()


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.msg = msg


def initialize():
    global __client__
    global __subscriber__
    __client__ = JointToCartClientAsync()
    __subscriber__ = JointStateSubscriber()


def finalize():
    __client__.destroy_node()
    __subscriber__.subscription.destroy()


def get_cart():
    rclpy.spin_once(__subscriber__)
    __client__.req.joint_state = __subscriber__.msg
    future = __client__.cli.call_async(__client__.req)
    rclpy.spin_until_future_complete(__client__, future)
    response = future.result()
    if response.success:
        translation = numpy.array([
                response.pose.pose.position.x,
                response.pose.pose.position.y,
                response.pose.pose.position.z])
        rotation = R.from_quat([
                response.pose.pose.orientation.x,
                response.pose.pose.orientation.y,
                response.pose.pose.orientation.z,
                response.pose.pose.orientation.w])
    else:
        translation = numpy.zeros(3, dtype=numpy.double)
        rotation = R.identity()
    return response.success, translation, rotation, numpy.max(numpy.abs(__subscriber__.msg.velocity)) < 1e-3

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        self.broadcaster = StaticTransformBroadcaster(self)

        # Initialize transform message once and reuse
        self.static_transform_stamped = TransformStamped()
        self.static_transform_stamped.header.frame_id = 'panda_link7'
        self.static_transform_stamped.child_frame_id = 'camera_link'

        # Translation (x, y, z)
        self.static_transform_stamped.transform.translation.x = -0.05
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.0

        # Rotation (roll=0, pitch=0, yaw=3.1415) converted to quaternion using scipy
        quat = R.from_euler('xyz', [0, -1.57, 0]).as_quat()  # [x, y, z, w]
        self.static_transform_stamped.transform.rotation.x = quat[0]
        self.static_transform_stamped.transform.rotation.y = quat[1]
        self.static_transform_stamped.transform.rotation.z = quat[2]
        self.static_transform_stamped.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(self.static_transform_stamped)
        self.get_logger().info('Published static transform from camera_link to camera_link_optical')

        # Start a timer to periodically call update
        # self.timer = self.create_timer(0.1, self.update)  # 10 Hz

    def update(self):
        pose = get_cart()

        if not pose[0]:
            self.get_logger().warn("Pose not available")
            return

        # Update header timestamp
        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()

        # Translation
        self.static_transform_stamped.transform.translation.x = pose[1][0]
        self.static_transform_stamped.transform.translation.y = pose[1][1]
        self.static_transform_stamped.transform.translation.z = pose[1][2]

        # R (quaternion)
        self.static_transform_stamped.transform.rotation.x = pose[2][0]
        self.static_transform_stamped.transform.rotation.y = pose[2][1]
        self.static_transform_stamped.transform.rotation.z = pose[2][2]
        self.static_transform_stamped.transform.rotation.w = pose[2][3]

        # Publish
        self.broadcaster.sendTransform(self.static_transform_stamped)
        self.get_logger().info('Published static transform from panda_link8 to camera_link')


def main():
    rclpy.init()
    # initialize()

    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down static_tf_publisher")
    finally:
        # finalize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

