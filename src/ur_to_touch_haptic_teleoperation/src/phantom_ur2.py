import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState, OmniButtonEvent, OmniFeedback
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock
from scipy.signal import butter, lfilter
from optoforce import OptoForce22 as OptoForce
from optoforce.status import no_errors
import numpy as np
from sensor_msgs.msg import JointState

def dh_matrix(theta, d, a, alpha):
    """
    Calcule la matrice de transformation DH pour une articulation donnÃ©e.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta)*np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

class UR3Kinematics:
    def __init__(self):
        # ParamÃ¨tres DH modifiÃ©s du UR3e
        self.a1 = 0.0
        self.d1 = 0.1519
        self.alpha1 = 0.5*np.pi
        self.a2 = -0.24365
        self.d2 = 0.0
        self.alpha2 = 0.0
        self.a3 = -0.21325
        self.d3 = 0.0
        self.alpha3 = 0.0
        self.a4 = 0.0
        self.d4 = 0.11235
        self.alpha4 = 0.5*np.pi
        self.a5 = 0.0
        self.d5 = 0.08535
        self.alpha5 = - 0.5*np.pi
        self.alpha6 = 0.0
        self.d6 = 0.0815
        self.a6 = 0.0

    def fk_ur(self, joint_angles):
        """
        Calcule la cinÃ©matique directe (position et orientation de l'effecteur).
        """
        
        theta1 = joint_angles[5]
        theta2 = joint_angles[0]
        theta3 = joint_angles[1]
        theta4 = joint_angles[2]
        theta5 = joint_angles[3]
        theta6 = joint_angles[4]
   
        # Matrices de transformation DH modifiÃ©es
        T01 = dh_matrix(theta1, self.d1, self.a1, self.alpha1)
        T12 = dh_matrix(theta2, self.d2, self.a2, self.alpha2)
        T23 = dh_matrix(theta3, self.d3, self.a3, self.alpha3)
        T34 = dh_matrix(theta4, self.d4, self.a4, self.alpha4)
        T45 = dh_matrix(theta5, self.d5, self.a5, self.alpha5)
        T56 = dh_matrix(theta6, self.d6, self.a6, self.alpha6)

        # Multiplier les matrices de transformation pour obtenir T06
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)
        T05 = np.dot(T04, T45)
        T06 = np.dot(T05, T56)
        T60=np.linalg.inv(T06)

        return T60



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

        # Historique des données pour le filtrage
        self.linear_x_history = []
        self.linear_y_history = []
        self.linear_z_history = []
        self.Vecteurforceoutil=[]
        self.VecteurforceBase=[]

        # Paramètres du filtre
        self.cutoff_frequency = 2.5  # Fréquence de coupure en Hz
        self.sampling_frequency = 500.0  # Exemple : fréquence d'échantillonnage à ajuster si nécessaire
        self.b, self.a = butter(2, self.cutoff_frequency / (self.sampling_frequency / 2), btype='low')
        #subscriber au tpoic joint_stat 
        self.subscription_joint = self.create_subscription(JointState,'/joint_states',self.joint_stat_callback,10)
        self.joint_angles=None   
    def joint_state_callback(self, msg: JointState):
        self.joint_angles = list(msg.position)  
        
        
        # Subscriber au topic OmniState
        self.subscription_state = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.omni_state_callback,
            1000)

        # Subscriber au topic OmniButtonEvent
        self.subscription_button = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.omni_button_callback,
            10)

        # Publisher vers TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1000)
        self.publisher_omni = self.create_publisher(OmniFeedback, '/phantom/force_feedback', 1000)
        self.get_logger().info("OmniState to TwistStamped with Button Control Node started.")

    def apply_filter(self, data_history):
        """Filtrer les données avec un filtre passe-bas."""
        if len(data_history) < len(self.b):
            return 0.0  # Retourne 0 si pas assez de données
        return lfilter(self.b, self.a, data_history)[-1]

    def omni_state_callback(self, msg: OmniState):
        if self.grey_button_pressed == 0:
            self.get_logger().info("Grey button is pressed. Stopping publishing.")
            return

        current_time = self.clock.now().nanoseconds / 1e9  # Convertir en secondes
        dt = current_time - self.timer
        self.timer = current_time

        # Créer un message TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        #MGD
        ur3=UR3Kinematics()
        if self.joint_angles is not None:
            ur3 = UR3Kinematics()
            T60 = ur3.fk_ur(self.joint_angles)
            R60 = T60[:3, :3]

            # Convertir le vecteur de force en numpy array
            F_outil = np.array([self.wrench_msg.force.x, self.wrench_msg.force.y, self.wrench_msg.force.z])

            # Calculer les forces dans le repère de base
            F_base = np.dot(R60, F_outil)

            # Stocker les forces transformées
            self.VecteurforceBase = F_base.tolist()
            self.get_logger().info(f"Force en base: {self.VecteurforceBase}")
        
        # Copier les vitesses linéaires
        self.linear_x_history.append(msg.velocity.x * 0.01)
        self.linear_y_history.append(msg.velocity.y * 0.01)
        self.linear_z_history.append(msg.velocity.z * 0.01)

        # Orientation 
        q1 = self.last_orientation.pose.orientation
        q2 = msg.pose.orientation
        # Calcul des vitesses angulaires
        #twist_msg.twist.angular.x = -(2.0 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y) * 100.0
        #twist_msg.twist.angular.y = -(2.0 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x) * 100.0
        #twist_msg.twist.angular.z = -(2.0 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w) * 100.0

        # Appliquer le filtre
        twist_msg.twist.linear.x = self.apply_filter(self.linear_x_history)
        twist_msg.twist.linear.y = self.apply_filter(self.linear_y_history)
        twist_msg.twist.linear.z = self.apply_filter(self.linear_z_history)
        with OptoForce(speed_hz=100, filter_hz=15, zero=False) as force_sensor:
            self.wrench_msg.force.x = force_sensor.read(only_latest_data=False).Fx - 3.0
            self.wrench_msg.force.y = force_sensor.read(only_latest_data=False).Fy + 7.0
            self.wrench_msg.force.z = force_sensor.read(only_latest_data=False).Fz - 50.5
            self.Vecteurforceoutil[0]=self.wrench_msg.force.x
            self.Vecteurforceoutil[1]=self.wrench_msg.force.y
            self.Vecteurforceoutil[2]=self.wrench_msg.force.z
        
        # Filtrage de bruit par seuil (deadband)
        self.deadband_filter_noise(twist_msg, 0.001)

        # Publication des messages
        self.publisher_.publish(twist_msg)
        self.publisher_omni.publish(self.wrench_msg)
        self.get_logger().info(f"Published TwistStamped: {twist_msg}")
        #test
        # Mettre à jour l'orientation précédente
        self.last_orientation.pose.orientation = msg.pose.orientation

    def omni_button_callback(self, msg: OmniButtonEvent):
        self.grey_button_pressed = msg.grey_button
        self.wrench_msg.force.x = 0.0
        self.wrench_msg.force.y = 0.0
        self.wrench_msg.force.z = 0.0
        if self.grey_button_pressed == 0:
            self.get_logger().info("Grey button pressed. Stopping Twist commands.")
        else:
            self.get_logger().info("Grey button released. Resuming Twist commands.")

    def deadband_filter_noise(self, twist, deadband):
        """Appliquer un filtre deadband pour supprimer le bruit."""
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
