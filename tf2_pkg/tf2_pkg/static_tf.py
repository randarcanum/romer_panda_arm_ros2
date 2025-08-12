#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R  #  tf_transformations yerine desteklemedi nedense

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        self.broadcaster = StaticTransformBroadcaster(self)

        static_transform_stamped = TransformStamped()

        # Header
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'panda_link8'
        static_transform_stamped.child_frame_id = 'camera_link'

        # Translation (x, y, z)
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.02

        # Rotation (roll=0, pitch=0, yaw=3.1415) converted to quaternion using scipy
        quat = R.from_euler('xyz', [0, 0, 0]).as_quat()  # [x, y, z, w]
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info('Published static transform from camera_link to camera_link_optical')

def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

