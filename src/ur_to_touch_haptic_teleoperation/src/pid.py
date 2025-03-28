import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class OmniStateToTwistWithButton(Node):
    def __init__(self):
        super().__init__('omni_to_twist_with_button_node')
        self.clock = Clock()
        self.K = 0.1

        # Initialisation des listes pour stocker les valeurs de q1[0] et le temps
        self.q1_0_values = deque()
        self.time_values = deque()

        # Abonnement au topic de la pose
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pl_scene_callback,
            10)
        
        # Création du publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Temps initial
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def pl_scene_callback(self, msg: PoseStamped):
        """Callback pour traiter les messages de pose."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        quat = msg.pose.orientation
        twist_msg = TwistStamped()
        q1 = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('zyx', degrees=False)

        # Stocker les valeurs de q1[0] et le temps écoulé
        self.q1_0_values.append(q1[0])
        self.time_values.append(current_time - self.start_time)
        q2 = [1, 0, 0]
        # self.get_logger().info(f"Euler ur tool: {q1} , Phantom Euler tool : {q2}")
        theta_erreur = R.from_euler('zyx', q2-q1)
        theta_axis = theta_erreur.as_rotvec()
        self.get_logger().info(f"theta : {theta_axis}")
        twist_msg.twist.linear.x = 0.1
        twist_msg.twist.linear.y = 0.1
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = theta_axis[0]*self.K
        twist_msg.twist.angular.y = theta_axis[1]*self.K
        twist_msg.twist.angular.z = theta_axis[2]*self.K
        self.publisher_.publish(twist_msg)
        # Affichage dans le terminal (optionnel)
        self.get_logger().info(f"Temps : {current_time - self.start_time:.2f} s, q1[0] : {q1[0]:.4f}")

    def plot_data(self):
        """Tracer q1[0] en fonction du temps."""
        plt.figure()
        plt.plot(self.time_values, self.q1_0_values, label='q1[0]')
        plt.xlabel('Temps (s)')
        plt.ylabel('q1[0] (rad)')
        plt.title('Évolution de q1[0] en fonction du temps')
        plt.legend()
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OmniStateToTwistWithButton()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.plot_data()  # Afficher le graphique à la fin
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
