import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayLayout

import HaplyHardwareAPI
import time


class HaplyInverse3Pos(Node):

    forces_ = [0, 0, 0]

    def __init__(self):
        super().__init__('haply_inverse3_pos')

        # Init haply
        self.initHaply()

        # Init ROS
        self.subscription_ = self.create_subscription(
            Float32MultiArray,
            'haply_forces',
            self.forces_callback,
            10)
        self.subscription_  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'haply_pos_vel', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.update(self.forces_)
        self.publisher_.publish(msg)

    def forces_callback(self, msg):
        self.forces_ = msg.data
    
    def initHaply(self):
        # Inverse 3
        self.connected_devices_ = HaplyHardwareAPI.detect_inverse3s()
        print(self.connected_devices_)
        if(len(self.connected_devices_)):
            self.com_stream_ = HaplyHardwareAPI.SerialStream(self.connected_devices_[0])
            # self.com_stream_ = HaplyHardwareAPI.SerialStream('/dev/serial/by-id/usb-Teensyduino_USB_Serial_16021850-if00')
            self.inverse3_ = HaplyHardwareAPI.Inverse3(self.com_stream_)
            response_to_wakeup = self.inverse3_.device_wakeup_dict()
            print("Connected to device {}".format(response_to_wakeup["device_id"]))
        else:
            print("ERROR - No Haply device connected")

    def update(self, forces):
        # Get Inverse pos, vel and send force
        position, velocity = self.inverse3_.end_effector_force(forces)
        data = position + velocity
        return data

def main(args=None):
    rclpy.init(args=args)
    haply_inverse3_pos = HaplyInverse3Pos()
    rclpy.spin(haply_inverse3_pos)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    haply_inverse3_pos.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()