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
        self.K = 0.5

        # Initialisation des listes pour stocker les valeurs de q1[0] et le temps
        self.q1_0_values = deque()
        self.time_values = deque()

        # Abonnement au topic de la pose
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pl_scene_callback,
            100)
        
        # Création du publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 100)

        # Temps initial
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def pl_scene_callback(self, msg: PoseStamped):
        """Callback pour traiter les messages de pose."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        quat = msg.pose.orientation
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        q1 = np.array([quat.x, quat.y, quat.z, quat.w])

        # Stocker les valeurs de q1[0] et le temps écoulé
        
        q2 = np.array([0.0, 0.0, np.sin(np.pi/4), np.cos(np.pi/4)])
        self.q1_0_values.append(q1[0])
        self.time_values.append(current_time - self.start_time)
        theta_axis = quaternion_to_axis_angle(q2,q1)*self.K
        self.get_logger().info(f"theta : {q1}")
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = theta_axis[0]
        twist_msg.twist.angular.y = theta_axis[1]
        twist_msg.twist.angular.z = theta_axis[2]
        self.publisher_.publish(twist_msg)
        # Affichage dans le terminal (optionnel)
        #self.get_logger().info(f"Temps : {current_time - self.start_time:.2f} s, q1[0] : {q1[0]:.4f}")

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

def quaternion_to_axis_angle(q_ref, q_mes):
    """
    Calcule l'erreur en rotation vectorielle (axis-angle)
    entre une consigne quaternion q_ref et une mesure quaternion q_mes.

    Args:
        q_ref (numpy.array): Quaternion de consigne [x, y, z, w]
        q_mes (numpy.array): Quaternion mesuré [x, y, z, w]

    Returns:
        numpy.array: Vecteur rotationnel de l'erreur (axis-angle)
    """
    # Calcul du quaternion d'erreur : q_e = q_ref^-1 ⊗ q_mes
    q_ref_inv = R.from_quat(q_ref).inv()  # Inverse de la consigne
    q_error = q_ref_inv * R.from_quat(q_mes)  # Multiplication quaternion

    # Convertir l'erreur en rotation vectorielle (axis-angle)
    axis_angle_error = q_error.as_rotvec()

    return axis_angle_error

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
