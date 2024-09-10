import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import numpy as np

from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter

import sys
absolute_path = "/home/yjshin/Desktop/dev/TEC-SMC/cases/dummy_app/src/my_util"
sys.path.append(absolute_path)
import my_time


class MyControl(Node):

    def __init__(self):
        super().__init__('my_control')
        self.set_my_parameters()

        self.devices = DeviceFactory.create_devices()
        self.meter = EnergyMeter(self.devices)

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

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        # self.timer = self.create_timer(1, self.publish_control_msg)

        self.get_logger().info('Subscribe state (start)')
        self.meter.start(tag='Subscribe')


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


    def publish_control_msg(self):
        self.meter.stop()
        energy_tag, duration, power, energy = self.get_power()
        self.get_logger().info('Subscribe state (end) ({0} duration:{1}) ({0} power:{2}) ({0} energy:{3})'.format(energy_tag, duration, power, energy))

        if self.control_split == 0:
            self.get_logger().info('Processing state (start)')
            self.meter.start(tag='Processing')
            proc_latency = self.normal_latency(self.control_proc_time_mean, self.control_proc_time_std)
            my_time.wait(proc_latency)
            msg = Twist()
            if self.emergency:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 1.0
            self.meter.stop()
            energy_tag, duration, power, energy = self.get_power()    
            self.get_logger().info('Processing state (end) ({0} duration:{1}) ({0} power:{2}) ({0} energy:{3})'.format(energy_tag, duration, power, energy))
        else:
            self.get_logger().info('PreProcessing state (start)')
            self.meter.start(tag='PreProcessing')
            pre_latency = self.normal_latency(self.control_pre_time_mean, self.control_pre_time_std)
            my_time.wait(pre_latency)
            self.meter.stop()
            energy_tag, duration, power, energy = self.get_power()
            self.get_logger().info('PreProcessing state (end) ({0} duration:{1}) ({0} power:{2}) ({0} energy:{3})'.format(energy_tag, duration, power, energy))

            self.get_logger().info('Wait state (start)')
            self.meter.start(tag='Wait')
            wait_latency = self.normal_latency(self.control_wait_time_mean, self.control_wait_time_std)
            my_time.wait(wait_latency)
            self.meter.stop()
            energy_tag, duration, power, energy = self.get_power()
            self.get_logger().info('Wait state (end) ({0} duration:{1}) ({0} power:{2}) ({0} energy:{3})'.format(energy_tag, duration, power, energy))

            self.get_logger().info('PostProcessing state (start)')
            self.meter.start(tag='PostProcessing')
            post_latency = self.normal_latency(self.control_post_time_mean, self.control_post_time_std)
            my_time.wait(post_latency)
            msg = Twist()
            if self.emergency:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 1.0
            self.meter.stop()
            energy_tag, duration, power, energy = self.get_power()
            self.get_logger().info('PostProcessing state (end) ({0} duration:{1}) ({0} power:{2}) ({0} energy:{3})'.format(energy_tag, duration, power, energy))
        
        # self.get_logger().info('Publish state (start)')
        self.twist_publisher.publish(msg)
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
        energy = sum(sample.energy.values()) 
        power = energy/sample.duration
        return sample.tag, sample.duration, power, energy

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