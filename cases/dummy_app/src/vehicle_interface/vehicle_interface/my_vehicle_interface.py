import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class MyVehicleInterface(Node):

    def __init__(self):
        super().__init__('my_vehicle_interface')
        self.set_my_parameters()

        qos_profile = QoSProfile(depth=10)
        self.control_subscriber = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.subscribe_control_message, 
            qos_profile)

        self.get_logger().info('Subscribe state (start)')


    def set_my_parameters(self):
        self.declare_parameter('vehicle_interface_sub_time_mean')
        self.declare_parameter('vehicle_interface_sub_time_std')

        self.vehicle_interface_sub_time_mean = self.get_parameter('vehicle_interface_sub_time_mean').value
        self.vehicle_interface_sub_time_std = self.get_parameter('vehicle_interface_sub_time_std').value

        self.get_logger().info('[PARAM] vehicle_interface_sub_time_mean: {0}'.format(self.vehicle_interface_sub_time_mean))
        self.get_logger().info('[PARAM] vehicle_interface_sub_time_std: {0}'.format(self.vehicle_interface_sub_time_std))


    def subscribe_control_message(self, msg):
        self.get_logger().info('Subscribe state (start-end)')
        # self.get_logger().info('Received control message: [linear.x:{0}, angular.z:{1}]'.format(msg.linear.x, msg.angular.z))
        




def main(args=None):
    rclpy.init(args=args)
    node = MyVehicleInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()