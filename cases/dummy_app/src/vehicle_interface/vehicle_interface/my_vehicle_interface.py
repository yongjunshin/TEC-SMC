import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter

import sys
absolute_path = "/home/yjshin/Desktop/dev/TEC-SMC/cases/dummy_app/src/my_util"
sys.path.append(absolute_path)
import my_power

class MyVehicleInterface(Node):

    def __init__(self):
        super().__init__('my_vehicle_interface')
        self.set_my_parameters()

        self.devices = DeviceFactory.create_devices()
        self.meter = EnergyMeter(self.devices)

        qos_profile = QoSProfile(depth=10)
        self.control_subscriber = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.subscribe_control_message, 
            qos_profile)

        self.get_logger().info('Subscribe state (start)')
        self.meter.start(tag='Subscribe')


    def set_my_parameters(self):
        pass


    def subscribe_control_message(self, msg):
        # self.get_logger().info('Received control message: [linear.x:{0}, angular.z:{1}]'.format(msg.linear.x, msg.angular.z))
        self.meter.stop()
        energy_tag, duration, power, energy = my_power.get_power(self.meter)
        self.get_logger().info('Subscribe state (end) ({0} duration:{1:.6f}) ({0} power:{2:.6f}) ({0} energy:{3:.6f})'.format(energy_tag, duration, power, energy))
        self.get_logger().info('Subscribe state (start)')
        self.meter.start(tag='Subscribe')
        

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