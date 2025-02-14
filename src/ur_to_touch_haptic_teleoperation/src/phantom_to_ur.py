import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState, OmniButtonEvent,OmniFeedback
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped
from rclpy.clock import Clock


class OmniStateToTwistWithButton(Node):
    def __init__(self):
        super().__init__('omni_to_twist_with_button_node')
        self.last_orientation = OmniState()
        self.last_orientation.pose.orientation.x = 0.0
        self.last_orientation.pose.orientation.y = 0.0
        self.last_orientation.pose.orientation.z = 0.0
        self.last_orientation.pose.orientation.w = 0.0
        # Initialisation des variables
        self.clock = Clock()
        self.timer = self.clock.now().nanoseconds / 1e9
        self.grey_button_pressed = False  # État du bouton gris
        self.wrench_msg = OmniFeedback()
        # Subscriber au topic OmniState
        self.subscription_state = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.omni_state_callback,
            10)

        # Subscriber au topic OmniButtonEvent
        self.subscription_button = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.omni_button_callback,
            10)
        self.subscription_wrench = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10)

        # Publisher vers TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1000)
        self.publisher_omni = self.create_publisher(OmniFeedback, '/phantom/force_feedback', 1000)
        self.get_logger().info("OmniState to TwistStamped with Button Control Node started.")

    def omni_state_callback(self, msg: OmniState):
        """
        Callback pour recevoir les données OmniState, les convertir en TwistStamped,
        et les publier sur /servo_node/delta_twist_cmds.
        """
        # Si le bouton gris est pressé, ne pas publier de message
        if self.grey_button_pressed == 0 :
            self.get_logger().info("Grey button is pressed. Stopping publishing.")
            return
        current_time = self.clock.now().nanoseconds / 1e9  # Convertir en secondes
        dt = current_time - self.timer
        self.timer = current_time

        # Créer un message TwistStamped
        twist_msg = TwistStamped()
        # Remplir le header
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"  # Cadre de référence (à ajuster si nécessaire)

        # Copier les vitesses linéaires et angulaires
        twist_msg.twist.linear.x = msg.velocity.x*0.1
        twist_msg.twist.linear.y = msg.velocity.y*0.1
        twist_msg.twist.linear.z = msg.velocity.z*0.1
        
        angular_velocity = TwistStamped
        q1 = self.last_orientation.pose.orientation
        q2 = msg.pose.orientation
        twist_msg.twist.angular.x = -(0.1 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y)
        twist_msg.twist.angular.y = -(0.1/ dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x)
        twist_msg.twist.angular.z = -(0.1/ dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w)

        self.deadband_filter_noise(twist_msg,0.001)
        # Publier le message TwistStamped
        self.publisher_.publish(twist_msg)
        self.publisher_omni.publish(self.wrench_msg)
        self.get_logger().info(f"Published TwistStamped: {twist_msg}")
        self.last_orientation.pose.orientation = msg.pose.orientation
    def omni_button_callback(self, msg: OmniButtonEvent):
        """
        Callback pour mettre à jour l'état du bouton gris.
        """
        # Mettre à jour l'état du bouton gris
        self.grey_button_pressed = msg.grey_button 
        self.wrench_msg.force.x = 0.0
        self.wrench_msg.force.y = 0.0
        self.wrench_msg.force.z = 0.0
        # Logger l'état du bouton
        if self.grey_button_pressed == 0:
            self.get_logger().info("Grey button pressed. Stopping Twist commands.")
            
        else:
            self.get_logger().info("Grey button released. Resuming Twist commands.")

    def wrench_callback(self, msg: WrenchStamped):
        """
        Callback pour mettre à jour l'état du bouton gris.
        """
        # Mettre à jour l'état du bouton gris
        self.wrench_msg.force.z = 0.1*msg.wrench.force.x
        self.wrench_msg.force.y = 0.1*msg.wrench.force.y
        self.wrench_msg.force.x = -0.1*msg.wrench.force.z
    def deadband_filter_noise(self,twist, deadband):
    
        if abs(twist.twist.linear.x) < deadband:
            twist.twist.linear.x = 0.0
        if abs(twist.twist.linear.y) < deadband:
            twist.twist.linear.y = 0.0
        if abs(twist.twist.linear.z) < deadband:
            twist.twist.linear.z = 0.0
        if abs(twist.twist.angular.x) < deadband:
            twist.twist.angular.x = 0.0
        if abs(twist.twist.angular.y) < deadband:
            twist.twist.angular.y = 0.0
        if abs(twist.twist.angular.z) < deadband:
            twist.twist.angular.z = 0.0

    
def main(args=None):
    rclpy.init(args=args)
    
    node = OmniStateToTwistWithButton()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
