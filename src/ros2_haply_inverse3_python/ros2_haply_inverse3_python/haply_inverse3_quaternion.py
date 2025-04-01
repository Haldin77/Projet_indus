import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import HaplyHardwareAPI

class HaplyInverse3Quaternion(Node):

    def __init__(self):
        super().__init__('haply_inverse3_quaternion')

        # Init haply
        self.initHaply()

        # Init ROS        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'haply_quaternion', 100)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Get versgrip info
        response = self.versegrip_.GetVersegripStatus()

        msg = Float32MultiArray()
        msg.data = response["quaternion"]
        msg.data.append(response["buttons"])

        self.publisher_.publish(msg)
    
    def initHaply(self):
        # VerseGrip
        self.connected_handles_ = HaplyHardwareAPI.detect_handles()
        print(self.connected_handles_)
        if(len(self.connected_handles_)):
            self.handle_stream_ = HaplyHardwareAPI.SerialStream(self.connected_handles_[0])
            # self.handle_stream_ = HaplyHardwareAPI.SerialStream('/dev/serial/by-id/usb-ZEPHYR_Haply_USB_Transceiver_7BD7C2F68DA7D969-if00')
            self.versegrip_ = HaplyHardwareAPI.Handle(self.handle_stream_)
            response = self.versegrip_.GetVersegripStatus()
            print(f"Versegrip status: {response}")
        else:
            print("ERROR - No Haply handle connected")

def main(args=None):
    rclpy.init(args=args)
    haply_inverse3_quaternion = HaplyInverse3Quaternion()
    rclpy.spin(haply_inverse3_quaternion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    haply_inverse3_quaternion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()