import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_node_interface.srv import EmergencyControl
import time


class MyControl(Node):

    def __init__(self):
        super().__init__('my_control')
        self.set_my_parameters()

        qos_profile = QoSProfile(depth=10)
        self.callback_group = ReentrantCallbackGroup()      # Callback group definition for parallization

        self.localization_data_available = False
        self.planning_data_available = False

        self.emergency = False

        self.localization_subscriber = self.create_subscription(
            String, 
            '/my_lidar_localization_output', 
            self.subscribe_localization_message, 
            qos_profile)
        
        self.planning_subscriber = self.create_subscription(
            String, 
            '/my_trajectory', 
            self.subscribe_planning_message, 
            qos_profile)

        # self.emergency_service_server = self.create_service(       # Service server definition
        #     EmergencyControl,                       # Service type
        #     'emergency_control',                    # Service name
        #     self.get_emergency_control_request,     # Service server callback function
        #     callback_group=self.callback_group)     # Callback function parallization group


        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        # self.timer = self.create_timer(1, self.publish_control_msg)

        self.get_logger().info('Subscribe state (start)')


    def set_my_parameters(self):
        self.declare_parameter('control_split')
        self.declare_parameter('control_proc_time_mean')
        self.declare_parameter('control_proc_time_std')
        self.declare_parameter('control_pre_time_mean')
        self.declare_parameter('control_pre_time_std')
        self.declare_parameter('control_wait_time_mean')
        self.declare_parameter('control_wait_time_std')
        self.declare_parameter('control_post_time_mean')   
        self.declare_parameter('control_post_time_std')

        self.control_split = self.get_parameter('control_split').value
        self.control_proc_time_mean = self.get_parameter('control_proc_time_mean').value
        self.control_proc_time_std = self.get_parameter('control_proc_time_std').value
        self.control_pre_time_mean = self.get_parameter('control_pre_time_mean').value
        self.control_pre_time_std = self.get_parameter('control_pre_time_std').value
        self.control_wait_time_mean = self.get_parameter('control_wait_time_mean').value
        self.control_wait_time_std = self.get_parameter('control_wait_time_std').value
        self.control_post_time_mean = self.get_parameter('control_post_time_mean').value
        self.control_post_time_std = self.get_parameter('control_post_time_std').value

        self.get_logger().info('[PARAM] control_split: {0}'.format(self.control_split))
        self.get_logger().info('[PARAM] control_proc_time_mean: {0}'.format(self.control_proc_time_mean))
        self.get_logger().info('[PARAM] control_proc_time_std: {0}'.format(self.control_proc_time_std))
        self.get_logger().info('[PARAM] control_pre_time_mean: {0}'.format(self.control_pre_time_mean))
        self.get_logger().info('[PARAM] control_pre_time_std: {0}'.format(self.control_pre_time_std))
        self.get_logger().info('[PARAM] control_wait_time_mean: {0}'.format(self.control_wait_time_mean))
        self.get_logger().info('[PARAM] control_wait_time_std: {0}'.format(self.control_wait_time_std))
        self.get_logger().info('[PARAM] control_post_time_mean: {0}'.format(self.control_post_time_mean))
        self.get_logger().info('[PARAM] control_post_time_std: {0}'.format(self.control_post_time_std))


    def subscribe_localization_message(self, msg):
        self.localization_data_available = True
        # self.get_logger().info('Received lidar localization output header {0}'.format(msg.data))

        if self.localization_data_available and self.planning_data_available:
            self.publish_control_msg()
    
    def subscribe_planning_message(self, msg):
        self.planning_data_available = True
        # self.get_logger().info('Received trajectory: {0}'.format(msg.data))
        if self.localization_data_available and self.planning_data_available:
            self.publish_control_msg()

    
    # def get_emergency_control_request(self, request, response):
    #     self.emergency = request.emergency_control_request    # Interprete service request
    #     if self.emergency:      # Service respond instantiation
    #         response.control_holder = 'driver'
    #         self.get_logger().info('Emergency! The driver takes control!')
    #     else:
    #         response.control_holder = 'vehicle'
    #         self.get_logger().info('Emergency released! The vehicle takes control!')
        
    #     return response     # return service respond
    

    def publish_control_msg(self):
        self.get_logger().info('Subscribe state (end)')

        if self.control_split == 0:
            self.get_logger().info('Processing state (start)')
            time.sleep(3)
            msg = Twist()
            if self.emergency:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 1.0
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
            msg = Twist()
            if self.emergency:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 1.0
            self.get_logger().info('PostProcessing state (end)')
        
        self.get_logger().info('Publish state (start)')
        self.twist_publisher.publish(msg)
        self.get_logger().info('Publish state (end)')

        self.get_logger().info('Subscribe state (start)')


def main(args=None):
    rclpy.init(args=args)
    node = MyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()