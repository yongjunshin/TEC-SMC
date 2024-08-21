import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time

class MyLidarPerception(Node):

    def __init__(self):
        super().__init__('my_lidar_perception')
        self.set_my_parameters()

        qos_profile = QoSProfile(depth=10)

        self.sensor_data_available = False
        self.localization_data_availalbe = False
        
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan',  
            self.subscribe_lidar_message, 
            qos_profile)
        
        self.lidar_localization_subscriber = self.create_subscription(
            String, 
            '/my_lidar_localization_output',  
            self.subscribe_lidar_localization_message, 
            qos_profile)
        
        self.peception_output_publisher = self.create_publisher(String, '/my_lidar_perception_output', qos_profile)
        # self.timer = self.create_timer(1, self.publish_lidar_perception_output_msg)

        self.get_logger().info('Subscribe state (start)')


    def set_my_parameters(self):
        self.declare_parameter('perception_split')
        self.declare_parameter('perception_proc_time_mean')
        self.declare_parameter('perception_proc_time_std')
        self.declare_parameter('perception_pre_time_mean')
        self.declare_parameter('perception_pre_time_std')
        self.declare_parameter('perception_wait_time_mean')
        self.declare_parameter('perception_wait_time_std')
        self.declare_parameter('perception_post_time_mean')   
        self.declare_parameter('perception_post_time_std')

        self.perception_split = self.get_parameter('perception_split').value
        self.perception_proc_time_mean = self.get_parameter('perception_proc_time_mean').value
        self.perception_proc_time_std = self.get_parameter('perception_proc_time_std').value
        self.perception_pre_time_mean = self.get_parameter('perception_pre_time_mean').value
        self.perception_pre_time_std = self.get_parameter('perception_pre_time_std').value
        self.perception_wait_time_mean = self.get_parameter('perception_wait_time_mean').value
        self.perception_wait_time_std = self.get_parameter('perception_wait_time_std').value
        self.perception_post_time_mean = self.get_parameter('perception_post_time_mean').value
        self.perception_post_time_std = self.get_parameter('perception_post_time_std').value

        self.get_logger().info('[PARAM] perception_split: {0}'.format(self.perception_split))
        self.get_logger().info('[PARAM] perception_proc_time_mean: {0}'.format(self.perception_proc_time_mean))
        self.get_logger().info('[PARAM] perception_proc_time_std: {0}'.format(self.perception_proc_time_std))
        self.get_logger().info('[PARAM] perception_pre_time_mean: {0}'.format(self.perception_pre_time_mean))
        self.get_logger().info('[PARAM] perception_pre_time_std: {0}'.format(self.perception_pre_time_std))
        self.get_logger().info('[PARAM] perception_wait_time_mean: {0}'.format(self.perception_wait_time_mean))
        self.get_logger().info('[PARAM] perception_wait_time_std: {0}'.format(self.perception_wait_time_std))
        self.get_logger().info('[PARAM] perception_post_time_mean: {0}'.format(self.perception_post_time_mean))
        self.get_logger().info('[PARAM] perception_post_time_std: {0}'.format(self.perception_post_time_std))

    
    def subscribe_lidar_message(self, msg):
        self.sensor_data_available = True
        # self.get_logger().info('Received lidar scan header: {0}'.format(msg.header))

        if self.sensor_data_available and self.localization_data_availalbe:
            self.publish_lidar_perception_output_msg()       


    def subscribe_lidar_localization_message(self, msg):
        self.localization_data_availalbe = True
        # self.get_logger().info('Received lidar localization msg: {0}'.format(msg.data))
        
        if self.sensor_data_available and self.localization_data_availalbe:
            self.publish_lidar_perception_output_msg()

            
    def publish_lidar_perception_output_msg(self):
        self.get_logger().info('Subscribe state (end)')

        if self.perception_split == 0:
            self.get_logger().info('Processing state (start)')
            time.sleep(3)
            msg = String()
            msg.data = 'Lidar perception output ({0})'.format(Clock().now())
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
            msg.data = 'Lidar perception output ({0})'.format(Clock().now())
            self.get_logger().info('PostProcessing state (end)')

        self.get_logger().info('Publish state (start)')
        self.peception_output_publisher.publish(msg)
        self.get_logger().info('Publish state (end)')

        self.get_logger().info('Subscribe state (start)')

def main(args=None):
    rclpy.init(args=args)
    node = MyLidarPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()