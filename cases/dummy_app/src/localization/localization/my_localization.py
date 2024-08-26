import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time
import numpy as np

from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter

class MyLocalization(Node):

    def __init__(self):
        super().__init__('my_localization')
        self.set_my_parameters()

        self.devices = DeviceFactory.create_devices()
        self.meter = EnergyMeter(self.devices)

        qos_profile = QoSProfile(depth=10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.subscribe_lidar_message, 
            qos_profile)

        self.localization_output_publisher = self.create_publisher(String, '/my_lidar_localization_output', qos_profile)
        # self.timer = self.create_timer(1, self.publish_lidar_localization_output_msg)

        self.get_logger().info('Subscribe state (start)')
        self.meter.start(tag='Subscribe')


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
        self.meter.stop()
        energy_tag, power = self.get_power()
        self.get_logger().info('Subscribe state (end) ({0} power:{1})'.format(energy_tag, power))

        if self.localization_split == 0:
            self.get_logger().info('Processing state (start)')
            self.meter.start(tag='Processing')
            proc_latency = self.normal_latency(self.localization_proc_time_mean, self.localization_proc_time_std)
            time.sleep(proc_latency)
            msg = String()
            msg.data = 'Lidar localization output ({0})'.format(Clock().now())
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('Processing state (end) ({0} power:{1})'.format(energy_tag, power))
        else:
            self.get_logger().info('PreProcessing state (start)')
            self.meter.start(tag='PreProcessing')
            pre_latency = self.normal_latency(self.localization_pre_time_mean, self.localization_pre_time_std)
            time.sleep(pre_latency)
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('PreProcessing state (end) ({0} power:{1})'.format(energy_tag, power))

            self.get_logger().info('Wait state (start)')
            self.meter.start(tag='Wait')
            wait_latency = self.normal_latency(self.localization_wait_time_mean, self.localization_wait_time_std)
            time.sleep(wait_latency)
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('Wait state (end) ({0} power:{1})'.format(energy_tag, power))

            self.get_logger().info('PostProcessing state (start)')
            self.meter.start(tag='PostProcessing')
            post_latency = self.normal_latency(self.localization_post_time_mean, self.localization_post_time_std)
            time.sleep(post_latency)
            msg = String()
            msg.data = 'Lidar localization output ({0})'.format(Clock().now())
            self.meter.stop()
            energy_tag, power = self.get_power()
            self.get_logger().info('PostProcessing state (end) ({0} power:{1})'.format(energy_tag, power))
        
        # self.get_logger().info('Publish state (start)')
        self.localization_output_publisher.publish(msg)
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
    node = MyLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()