import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time


class MyLidarLocalization(Node):

    def __init__(self):
        super().__init__('my_lidar_localization')
        self.set_my_parameters()

        qos_profile = QoSProfile(depth=10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.subscribe_lidar_message, 
            qos_profile)

        self.localization_output_publisher = self.create_publisher(String, '/my_lidar_localization_output', qos_profile)
        # self.timer = self.create_timer(1, self.publish_lidar_localization_output_msg)

        self.get_logger().info('Subscribe state (start)')


    def set_my_parameters(self):
        self.declare_parameter('localization_split')
        self.declare_parameter('localization_proc_time_mean')
        self.declare_parameter('localization_proc_time_std')
        self.declare_parameter('localization_pre_time_mean')
        self.declare_parameter('localization_pre_time_std')
        self.declare_parameter('localization_wait_time_mean')
        self.declare_parameter('localization_wait_time_std')
        self.declare_parameter('localization_post_time_mean')   
        self.declare_parameter('localization_post_time_std')

        self.localization_split = self.get_parameter('localization_split').value
        self.localization_proc_time_mean = self.get_parameter('localization_proc_time_mean').value
        self.localization_proc_time_std = self.get_parameter('localization_proc_time_std').value
        self.localization_pre_time_mean = self.get_parameter('localization_pre_time_mean').value
        self.localization_pre_time_std = self.get_parameter('localization_pre_time_std').value
        self.localization_wait_time_mean = self.get_parameter('localization_wait_time_mean').value
        self.localization_wait_time_std = self.get_parameter('localization_wait_time_std').value
        self.localization_post_time_mean = self.get_parameter('localization_post_time_mean').value
        self.localization_post_time_std = self.get_parameter('localization_post_time_std').value

        self.get_logger().info('[PARAM] localization_split: {0}'.format(self.localization_split))
        self.get_logger().info('[PARAM] localization_proc_time_mean: {0}'.format(self.localization_proc_time_mean))
        self.get_logger().info('[PARAM] localization_proc_time_std: {0}'.format(self.localization_proc_time_std))
        self.get_logger().info('[PARAM] localization_pre_time_mean: {0}'.format(self.localization_pre_time_mean))
        self.get_logger().info('[PARAM] localization_pre_time_std: {0}'.format(self.localization_pre_time_std))
        self.get_logger().info('[PARAM] localization_wait_time_mean: {0}'.format(self.localization_wait_time_mean))
        self.get_logger().info('[PARAM] localization_wait_time_std: {0}'.format(self.localization_wait_time_std))
        self.get_logger().info('[PARAM] localization_post_time_mean: {0}'.format(self.localization_post_time_mean))
        self.get_logger().info('[PARAM] localization_post_time_std: {0}'.format(self.localization_post_time_std))


    def subscribe_lidar_message(self, msg):
        # self.get_logger().info('Received lidar scan header: {0}'.format(msg.header))
        self.publish_lidar_localization_output_msg()


    def publish_lidar_localization_output_msg(self):
        self.get_logger().info('Subscribe state (end)')

        if self.localization_split == 0:
            self.get_logger().info('Processing state (start)')
            time.sleep(3)
            msg = String()
            msg.data = 'Lidar localization output ({0})'.format(Clock().now())
            self.get_logger().info('Processing state (end)')
        else:
            self.get_logger().info('PreProcessing state (start)')
            time.sleep(3)
            self.get_logger().info('PreProcessing state (end)')

            self.get_logger().info('Wait state (start)')
            time.sleep(3)
            self.get_logger().info('Wait state (end)')

            self.get_logger().info('PostProcessing state (start)')
            time.sleep(3)
            msg = String()
            msg.data = 'Lidar localization output ({0})'.format(Clock().now())
            self.get_logger().info('PostProcessing state (end)')
        
        self.get_logger().info('Publish state (start)')
        self.localization_output_publisher.publish(msg)
        self.get_logger().info('Publish state (end)')

        self.get_logger().info('Subscribe state (start)')


def main(args=None):
    rclpy.init(args=args)
    node = MyLidarLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()