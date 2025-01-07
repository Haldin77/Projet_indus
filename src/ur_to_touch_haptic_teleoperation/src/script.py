import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped  # Correction ici
from pynput import keyboard
from trajectory_msgs.msg import JointTrajectory

class ContinuousKeyboardControlNode(Node):
    def __init__(self):
        super().__init__('continuous_keyboard_control_node')
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 100)
        self.linear_speed = 10.1  # m/s
        self.angular_speed = 10.1  # rad/s
        self.current_twist = TwistStamped()
        self.pressed_key = None

        # Publie à une fréquence constante
        self.timer = self.create_timer(0.1, self.publish_twist)

    def on_press(self, key):
        try:
            if key.char == 'w':  # Avancer
                self.set_twist(linear_x=self.linear_speed)
            elif key.char == 's':  # Reculer
                self.set_twist(linear_x=-self.linear_speed)
            elif key.char == 'a':  # Tourner à gauche
                self.set_twist(angular_z=self.angular_speed)
            elif key.char == 'd':  # Tourner à droite
                self.set_twist(angular_z=-self.angular_speed)
            elif key.char == 'q':  # Monter
                self.set_twist(linear_z=self.linear_speed)
            elif key.char == 'e':  # Descendre
                self.set_twist(linear_z=-self.linear_speed)

            self.pressed_key = key.char
        except AttributeError:
            pass  # Ignore les touches spéciales

    def on_release(self, key):
        # Arrête le mouvement lorsque la touche est relâchée
        if self.pressed_key == getattr(key, 'char', None):
            self.reset_twist()
            self.pressed_key = None

        if key == keyboard.Key.esc:
            # Quitte le programme avec Échap
            return False

    def set_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        self.current_twist.twist.linear.x = linear_x
        self.current_twist.twist.linear.y = linear_y
        self.current_twist.twist.linear.z = linear_z
        self.current_twist.twist.angular.x = angular_x
        self.current_twist.twist.angular.y = angular_y
        self.current_twist.twist.angular.z = angular_z

    def reset_twist(self):
        self.set_twist()  # Reset all values to 0

    def publish_twist(self):
        if self.pressed_key is not None:  # Publie uniquement si une touche est pressée
            self.current_twist.header.stamp = self.get_clock().now().to_msg()
            self.current_twist.header.frame_id = 'base_link'
            self.publisher.publish(self.current_twist)
            self.get_logger().info(f'Published: {self.current_twist}')

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousKeyboardControlNode()

    with keyboard.Listener(on_press=node.on_press, on_release=node.on_release) as listener:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

