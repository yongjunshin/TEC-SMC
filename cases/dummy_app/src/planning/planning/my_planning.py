import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import numpy as np
from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter


class MyPlanning(Node):

    def __init__(self):
        super().__init__('my_planning')
        self.set_my_parameters()

        self.devices = DeviceFactory.create_devices()
        self.meter = EnergyMeter(self.devices)

        qos_profile = QoSProfile(depth=10)
        self.callback_group = ReentrantCallbackGroup()      # Callback group definition for parallization

        self.localization_data_available = False
        self.perception_data_available = False

        self.localization_subscriber = self.create_subscription(
            String, 
            '/my_lidar_localization_output', 
            self.subscribe_localization_message, 
            qos_profile)
        
        self.lidar_perception_subscriber = self.create_subscription(
            String, 
            '/my_lidar_perception_output', 
            self.subscribe_lidar_perception_message, 
            qos_profile)

        self.trajectory_publisher = self.create_publisher(String, '/my_trajectory', qos_profile)
        # self.timer = self.create_timer(1, self.publish_trajectory_msg)

        self.get_logger().info('Subscribe state (start)')
        self.meter.start(tag='Subscribe')


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


    def publish_trajectory_msg(self):
        self.meter.stop()
        energy_tag, power = self.get_power()
        self.get_logger().info('Subscribe state (end) ({0} power:{1})'.format(energy_tag, power))

        if self.planning_split == 0:
            self.get_logger().info('Processing state (start)')
            self.meter.start(tag='Processing')
            proc_latency = self.normal_latency(self.planning_proc_time_mean, self.planning_proc_time_std)
            time.sleep(proc_latency)
            msg = String()
            msg.data = 'Planned trajectory ({0})'.format(Clock().now())
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('Processing state (end) ({0} power:{1})'.format(energy_tag, power))
        else:
            self.get_logger().info('PreProcessing state (start)')
            self.meter.start(tag='PreProcessing')
            pre_latency = self.normal_latency(self.planning_pre_time_mean, self.planning_pre_time_std)
            time.sleep(pre_latency)
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('PreProcessing state (end) ({0} power:{1})'.format(energy_tag, power))

            self.get_logger().info('Wait state (start)')
            self.meter.start(tag='Wait')
            wait_latency = self.normal_latency(self.planning_wait_time_mean, self.planning_wait_time_std)
            time.sleep(wait_latency)
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('Wait state (end) ({0} power:{1})'.format(energy_tag, power))

            self.get_logger().info('PostProcessing state (start)')
            self.meter.start(tag='PostProcessing')
            post_latency = self.normal_latency(self.planning_post_time_mean, self.planning_post_time_std)
            time.sleep(post_latency)
            msg = String()
            msg.data = 'Planned trajectory ({0})'.format(Clock().now())
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('PostProcessing state (end) ({0} power:{1})'.format(energy_tag, power))
        
        # self.get_logger().info('Publish state (start)')
        self.trajectory_publisher.publish(msg)
        # self.get_logger().info('Publish state (end)')

        self.get_logger().info('Subscribe state (start)')
        self.meter.start(tag='Subscribe')


    def normal_latency(self, mean, stddev):
        latency = np.random.normal(mean, stddev, 1)[0]
        if latency < 0:
            latency = 0
        return latency
    
    def get_power(self):
        sample = self.meter.get_trace()[0]
        power = sum(sample.energy.values())/sample.duration
        return sample.tag, power

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