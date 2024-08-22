import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MySensor(Node):

    def __init__(self):
        super().__init__('my_sensor')
        self.set_my_parameters()

        qos_profile = QoSProfile(depth=10)

        self.ladar_publisher = self.create_publisher(LaserScan, '/scan', qos_profile)
        self.timer = self.create_timer(self.sensor_sense_time_mean, self.publish_lidar_msg)

        self.get_logger().info('Sense state (start)')
    
    def set_my_parameters(self):
        self.declare_parameter('sensor_sense_time_mean')

        self.sensor_sense_time_mean = self.get_parameter('sensor_sense_time_mean').value

        self.get_logger().info('[PARAM] sensor_sense_time_mean: {0}'.format(self.sensor_sense_time_mean))


    def publish_lidar_msg(self):
        msg = LaserScan()
        self.ladar_publisher.publish(msg)
        # self.get_logger().info('Published lidar scan header: {0}'.format(msg.header))
        self.get_logger().info('Sense state (end)')
        self.get_logger().info('Sense state (start)')


def main(args=None):
    rclpy.init(args=args)
    node = MySensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()