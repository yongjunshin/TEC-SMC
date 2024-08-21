import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_node_interface.action import Destination
import random
import time


class MyPlanning(Node):

    def __init__(self):
        super().__init__('my_planning')
        self.set_my_parameters()

        qos_profile = QoSProfile(depth=10)
        self.callback_group = ReentrantCallbackGroup()      # Callback group definition for parallization

        self.localization_data_available = False
        self.perception_data_available = False

        self.localization_subscriber = self.create_subscription(
            String, 
            '/my_lidar_localization_output', 
            self.subscribe_localization_message, 
            qos_profile)
        
        # self.camera_perception_subscriber = self.create_subscription(
        #     String, 
        #     '/my_camera_perception_output', 
        #     self.subscribe_camera_perception_message, 
        #     qos_profile)
        
        # self.destination_action_server = ActionServer(       # Action server definition
        #     self,
        #     Destination,      # Action type
        #     'destination',   # Action name
        #     self.destination_checker,   # Action callback function
        #     callback_group=self.callback_group)     # Callback function parallization group
        
        self.lidar_perception_subscriber = self.create_subscription(
            String, 
            '/my_lidar_perception_output', 
            self.subscribe_lidar_perception_message, 
            qos_profile)

        self.trajectory_publisher = self.create_publisher(String, '/my_trajectory', qos_profile)
        # self.timer = self.create_timer(1, self.publish_trajectory_msg)

        self.get_logger().info('Subscribe state (start)')


    def set_my_parameters(self):
        self.declare_parameter('planning_split')
        self.declare_parameter('planning_proc_time_mean')
        self.declare_parameter('planning_proc_time_std')
        self.declare_parameter('planning_pre_time_mean')
        self.declare_parameter('planning_pre_time_std')
        self.declare_parameter('planning_wait_time_mean')
        self.declare_parameter('planning_wait_time_std')
        self.declare_parameter('planning_post_time_mean')   
        self.declare_parameter('planning_post_time_std')

        self.planning_split = self.get_parameter('planning_split').value
        self.planning_proc_time_mean = self.get_parameter('planning_proc_time_mean').value
        self.planning_proc_time_std = self.get_parameter('planning_proc_time_std').value
        self.planning_pre_time_mean = self.get_parameter('planning_pre_time_mean').value
        self.planning_pre_time_std = self.get_parameter('planning_pre_time_std').value
        self.planning_wait_time_mean = self.get_parameter('planning_wait_time_mean').value
        self.planning_wait_time_std = self.get_parameter('planning_wait_time_std').value
        self.planning_post_time_mean = self.get_parameter('planning_post_time_mean').value
        self.planning_post_time_std = self.get_parameter('planning_post_time_std').value

        self.get_logger().info('[PARAM] planning_split: {0}'.format(self.planning_split))
        self.get_logger().info('[PARAM] planning_proc_time_mean: {0}'.format(self.planning_proc_time_mean))
        self.get_logger().info('[PARAM] planning_proc_time_std: {0}'.format(self.planning_proc_time_std))
        self.get_logger().info('[PARAM] planning_pre_time_mean: {0}'.format(self.planning_pre_time_mean))
        self.get_logger().info('[PARAM] planning_pre_time_std: {0}'.format(self.planning_pre_time_std))
        self.get_logger().info('[PARAM] planning_wait_time_mean: {0}'.format(self.planning_wait_time_mean))
        self.get_logger().info('[PARAM] planning_wait_time_std: {0}'.format(self.planning_wait_time_std))
        self.get_logger().info('[PARAM] planning_post_time_mean: {0}'.format(self.planning_post_time_mean))
        self.get_logger().info('[PARAM] planning_post_time_std: {0}'.format(self.planning_post_time_std))



    def subscribe_localization_message(self, msg):
        self.localization_data_available = True
        # self.get_logger().info('Received lidar localization output header {0}'.format(msg.data))

        if self.localization_data_available and self.perception_data_available:
            self.publish_trajectory_msg()

    # def subscribe_camera_perception_message(self, msg):
    #     self.get_logger().info('Received camera perception output header {0}'.format(msg.data))

    def subscribe_lidar_perception_message(self, msg):
        self.perception_data_available = True
        # self.get_logger().info('Received lidar perception output header {0}'.format(msg.data))

        if self.localization_data_available and self.perception_data_available:
            self.publish_trajectory_msg()

    # def destination_checker(self, goal_handle):
    #     self.get_logger().info('Execute destination_checker action!')
    #     feedback_msg = Destination.Feedback()     # Define action feedback holder
        
    #     cur_location = 0.0
    #     destination = goal_handle.request.destination     # Action goal interpretation
    #     distance_to_destination = random.randint(5, 10)

    #     diff_to_destination = distance_to_destination - cur_location
    #     while diff_to_destination > 1.0:     # Action process until goal achievement
    #         feedback_msg.diff_to_destination = diff_to_destination
    #         self.get_logger().info('Feedback: {0}'.format(feedback_msg.diff_to_destination))
    #         goal_handle.publish_feedback(feedback_msg)      # Action feedback publish
    #         time.sleep(1)
    #         cur_location = cur_location + 1.0
    #         diff_to_destination = distance_to_destination - cur_location

    #     goal_handle.succeed()       # Inform goal achievement status
    #     result = Destination.Result()     # Action result generation
    #     result.diff_to_destination = diff_to_destination
        
    #     return result       # Return action result

    def publish_trajectory_msg(self):
        self.get_logger().info('Subscribe state (end)')

        if self.planning_split == 0:
            self.get_logger().info('Processing state (start)')
            time.sleep(3)
            msg = String()
            msg.data = 'Planned trajectory ({0})'.format(Clock().now())
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
            msg.data = 'Planned trajectory ({0})'.format(Clock().now())
            self.get_logger().info('PostProcessing state (end)')
        
        self.get_logger().info('Publish state (start)')
        self.trajectory_publisher.publish(msg)
        self.get_logger().info('Publish state (end)')

        self.get_logger().info('Subscribe state (start)')


def main(args=None):
    rclpy.init(args=args)
    node = MyPlanning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()