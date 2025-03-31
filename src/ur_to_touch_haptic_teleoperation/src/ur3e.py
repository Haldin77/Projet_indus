import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState, OmniButtonEvent, OmniFeedback
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
from scipy.signal import butter, lfilter
import tf_transformations as tf
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

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
        self.d1 = 0.15185
        self.alpha1 = 0.5*np.pi
        self.a2 = -0.24355
        self.d2 = 0.0
        self.alpha2 = 0.0
        self.a3 = -0.2132
        self.d3 = 0.0
        self.alpha3 = 0.0
        self.a4 = 0.0
        self.d4 = 0.13105
        self.alpha4 = 0.5*np.pi
        self.a5 = 0.0
        self.d5 = 0.08535
        self.alpha5 = - 0.5*np.pi
        self.alpha6 = 0.0
        self.d6 = 0.0921
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
        # theta1 = joint_angles[0]
        # theta2 = joint_angles[1]
        # theta3 = joint_angles[2]
        # theta4 = joint_angles[3]
        # theta5 = joint_angles[4]
        # theta6 = joint_angles[5]

   
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
        # T60=np.linalg.inv(T06)

        return T06



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
        self.wrench_input = WrenchStamped()
        self.wrench_msg = WrenchStamped()
        self.pl_scene = PoseStamped()
        # Historique des données pour le filtrage
        self.linear_x_history = []
        self.linear_y_history = []
        self.linear_z_history = []
        self.angular_x_history = []
        self.angular_y_history = []
        self.angular_z_history = []
        self.angular_w_history = []
        self.angular_velocity_x = []
        self.angular_velocity_y = []
        self.angular_velocity_z = []
        self.Vecteurforceoutil=[]
        self.VecteurforceBase=[]
        self.K = 1.5

        # Paramètres du filtre
        self.cutoff_frequency = 2.5  # Fréquence de coupure en Hz
        self.sampling_frequency = 500.0  # Exemple : fréquence d'échantillonnage à ajuster si nécessaire
        self.b, self.a = butter(2, self.cutoff_frequency / (self.sampling_frequency / 2), btype='low')
        #subscriber au tpoic joint_stat 
        self.subscription_joint = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,1000)
        self.joint_angles=[0,0,0,0,0,0]   
        # Subscriber au topic OmniState
        self.subscription_state = self.create_subscription(
            OmniState,
            '/phantom_state',
            self.omni_state_callback,
            1000)

        # Subscriber au topic OmniButtonEvent
        

        self.subscription_wrench = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            100)
        
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pl_scene_callback,
            100)
        
        # Publisher vers TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1000)
        self.publisher_omni = self.create_publisher(WrenchStamped, '/wrench', 1000)
        self.get_logger().info("OmniState to TwistStamped with Button Control Node started.")
        
    def joint_state_callback(self, msg: JointState):
        self.joint_angles = list(msg.position)
        
        

    def apply_filter(self, data_history,n):
        """Filtrer les données avec un filtre passe-bas."""
        if len(data_history) < len(self.b):
            return 0.0  # Retourne 0 si pas assez de données
        return lfilter(self.b, self.a, data_history)[n]
    
    
    def omni_state_callback(self, msg: OmniState):
        
        current_time = self.clock.now().nanoseconds / 1e9  # Convertir en secondes
        dt = current_time - self.timer
        self.timer = current_time

        # Créer un message TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        #MGD

        
            #self.get_logger().info(f"Force en base: {self.VecteurforceBase}")
        
        # Copier les vitesses linéaires
        self.linear_x_history.append(msg.velocity.x*10)
        self.linear_y_history.append(msg.velocity.y*10)
        self.linear_z_history.append(msg.velocity.z*10)


        # self.angular_x_history.append(msg.pose.orientation.x)
        # self.angular_y_history.append(msg.pose.orientation.y)
        # self.angular_z_history.append(msg.pose.orientation.z)
        quat = self.pl_scene.pose.orientation
        q1 = np.array([quat.x,quat.y,quat.z,quat.w])
        q2 = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        theta_axis = self.quaternion_to_axis_angle(q2,q1)*self.K
        
        if len(self.linear_x_history) > 20 :
            self.linear_x_history = self.linear_x_history[1:-1]
            self.linear_y_history = self.linear_y_history[1:-1]
            self.linear_z_history = self.linear_z_history[1:-1]
            # self.angular_x_history = self.angular_x_history[1:-1]
            # self.angular_y_history = self.angular_y_history[1:-1]
            # self.angular_z_history = self.angular_z_history[1:-1]
            # self.angular_velocity_x = self.angular_velocity_x[1:-1]
            # self.angular_velocity_y = self.angular_velocity_y[1:-1]
            # self.angular_velocity_z = self.angular_velocity_z[1:-1]
        # Orientation 
        # Calcul des vitesses angulaires
        # if len(self.angular_x_history) < 2 or len(self.angular_w_history) < 2 or len(self.angular_y_history) < 2 or len(self.angular_z_history) < 2:
        #     twist_msg.twist.angular.x = 0.0
        #     twist_msg.twist.angular.y = 0.0
        #     twist_msg.twist.angular.z = 0.0
        # else:
        #     # self.angular_velocity_x.append((2.0 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y))
        #     # self.angular_velocity_y.append((2.0 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x))
        #     # self.angular_velocity_z.append((2.0 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w))
        #     self.angular_velocity_x.append((q2[0] - q1[0])*self.K)
        #     self.angular_velocity_y.append((q2[1] - q1[1])*self.K)
        #     self.angular_velocity_z.append((q2[2] - q1[2])*self.K)

        # Appliquer le filtre
        twist_msg.twist.linear.x = -self.apply_filter(self.linear_x_history,-1)
        twist_msg.twist.linear.y = -self.apply_filter(self.linear_y_history,-1)
        twist_msg.twist.linear.z = self.apply_filter(self.linear_z_history,-1)
        twist_msg.twist.angular.x = theta_axis[0]#-(q2[0] - q1[0])*self.K
        twist_msg.twist.angular.y = theta_axis[1]#-(q2[1] - q1[1])*self.K
        twist_msg.twist.angular.z = theta_axis[2]#-(q2[2] - q1[2])*self.K  
        self.deadband_filter_noise(twist_msg, 0.00101)
        

        # Convertir les composantes de la force (Vector3) en un tableau numpy
        # Convertir les composantes de la force (Vector3) en un tableau numpy
        #force = np.array([self.wrench_msg.wrench.force.x,self.wrench_msg.wrench.force.y,self.wrench_msg.wrench.force.z])

        # Convertir la vitesse linéaire (Vector3) en un tableau numpy
        #linear_velocity = np.array([twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z])

        # Calculer la norme au carré de la force (produit scalaire de la force avec elle-même)

        # Vérifier si la norme de la force est supérieure à 10
        
        force = np.array([self.wrench_msg.wrench.force.x, 
                          self.wrench_msg.wrench.force.y, 
                          self.wrench_msg.wrench.force.z])

        # Convertir la vitesse linéaire (Vector3) en un tableau numpy
        linear_velocity = np.array([twist_msg.twist.linear.x, 
                                    twist_msg.twist.linear.y, 
                                    twist_msg.twist.linear.z])

        # Calculer la norme au carré de la force (produit scalaire de la force avec elle-même)
        norm = np.dot(force, force)

        # Vérifier si la norme de la force est supérieure à 10
        if norm >200:
            # # Calculer le produit scalaire entre la force et la vitesse linéaire
            if np.dot(force, linear_velocity) > 0:
            #     dot_product = np.dot(force, linear_velocity)

            #     # Calculer la projection de la vitesse linéaire sur la force
            #     projection = (dot_product / norm) * force

            #     # Calculer la symétrie de la vitesse linéaire par rapport au plan
            #     linear_velocity_sym = linear_velocity - 2* projection

            #     # Mettre à jour twist_msg avec la vitesse symétrique
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.linear.z = 0.0
        # Publication des messages
        self.publisher_.publish(twist_msg)
        #self.get_logger().info(f"Published TwistStamped: {twist_msg}")
        #test
        # Mettre à jour l'orientation précédente
        self.last_orientation.pose.orientation = msg.pose.orientation

    def wrench_callback(self, msg: WrenchStamped):
        """
        Mise a jour des efforts
        """
        if self.joint_angles is not None:
            ur3 = UR3Kinematics()
            T06 = ur3.fk_ur(self.joint_angles)
            R06 = T06[:3, :3]
            # with OptoForce(speed_hz=100, filter_hz=1.5, zero=True) as force_sensor:
            #     force = [force_sensor.read(only_latest_data=False).Fx,force_sensor.read(only_latest_data=False).Fy,force_sensor.read(only_latest_data=False).Fz]
            #     force[2] = force[2] / 10 
            #     force[1] = force[1] 
            #     force[0] = force[0] 
            #     print(force)
            # Convertir le vecteur de force en numpy array
            F_outil = np.array([self.wrench_input.wrench.force.x, self.wrench_input.wrench.force.y, self.wrench_input.wrench.force.z])
            # F_outil = [0,0,0]
            #self.get_logger().info(f"OUTIL : {F_outil}")
            # Calculer les forces dans le repère de base
            F_base = np.dot(R06, F_outil)
            #self.get_logger().info(f"BASE : {F_base}")

            # Stocker les forces transformées
            self.VecteurforceBase = F_base.tolist()
            self.wrench_input = msg
            self.wrench_msg.wrench.force.x = self.VecteurforceBase[0]*0.1
            self.wrench_msg.wrench.force.y = self.VecteurforceBase[1]*0.1
            self.wrench_msg.wrench.force.z = self.VecteurforceBase[2]*0.1

            self.publisher_omni.publish(self.wrench_msg)
    
    
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

    def pl_scene_callback(self, msg: PoseStamped):
        self.pl_scene = msg

    def quaternion_to_axis_angle(self, q_ref, q_mes):
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
            ur3 = UR3Kinematics()
            T06 = ur3.fk_ur(self.joint_angles)
            R06 = T06[:3, :3]
            axis_angle_error = np.dot(np.linalg.inv(R06), axis_angle_error)
            return axis_angle_error
    
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
