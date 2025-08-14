from romer_interfaces.srv import JointToCartesian
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
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
    rclpy.init()
    global __client__
    global __subscriber__
    __client__ = JointToCartClientAsync()
    __subscriber__ = JointStateSubscriber()


def finalize():
    __client__.destroy_node()
    __subscriber__.subscription.destroy()
    rclpy.shutdown()


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
        rotation = Rotation.from_quat([
                response.pose.pose.orientation.x,
                response.pose.pose.orientation.y,
                response.pose.pose.orientation.z,
                response.pose.pose.orientation.w])
    else:
        translation = numpy.zeros(3, dtype=numpy.double)
        rotation = Rotation.identity()
    return response.success, translation, rotation, numpy.max(numpy.abs(__subscriber__.msg.velocity)) < 1e-3
