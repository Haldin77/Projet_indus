#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"

// Eigen is used for matrix operations
#include <Eigen/Dense>

// For convenience
using std::placeholders::_1;
using namespace std::chrono_literals;

/*
Calcule la matrice de transformation DH pour une articulation donné.
Equivalent de la fonction dh_matrix en Python.
*/
Eigen::Matrix4d dh_matrix(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::sin(theta) * std::cos(alpha),  std::sin(theta) * std::sin(alpha), a * std::cos(theta),
         std::sin(theta),  std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha), a * std::sin(theta),
                      0,                     std::sin(alpha),                     std::cos(alpha),                d,
                      0,                                 0,                                 0,                1;
    return T;
}

class UR3Kinematics {
public:
    UR3Kinematics() {
        // Paramètres DH modifiés du UR3e
        a1 = 0.0;
        d1 = 0.15185;
        alpha1 = 0.5 * M_PI;
        a2 = -0.24355;
        d2 = 0.0;
        alpha2 = 0.0;
        a3 = -0.2132;
        d3 = 0.0;
        alpha3 = 0.0;
        a4 = 0.0;
        d4 = 0.13105;
        alpha4 = 0.5 * M_PI;
        a5 = 0.0;
        d5 = 0.08535;
        alpha5 = -0.5 * M_PI;
        alpha6 = 0.0;
        d6 = 0.0921;
        a6 = 0.0;
    }

    /*
    Calcule la cinématique directe (position et orientation de l'effecteur).
    */
    Eigen::Matrix4d fk_ur(const std::vector<double> &joint_angles) {
        // Mapping des angles selon le Python original
        double theta1 = joint_angles[5];
        double theta2 = joint_angles[0];
        double theta3 = joint_angles[1];
        double theta4 = joint_angles[2];
        double theta5 = joint_angles[3];
        double theta6 = joint_angles[4];
        // theta1 = joint_angles[0]
        // theta2 = joint_angles[1]
        // theta3 = joint_angles[2]
        // theta4 = joint_angles[3]
        // theta5 = joint_angles[4]
        // theta6 = joint_angles[5]

        // Matrices de transformation DH modifiées
        Eigen::Matrix4d T01 = dh_matrix(theta1, d1, a1, alpha1);
        Eigen::Matrix4d T12 = dh_matrix(theta2, d2, a2, alpha2);
        Eigen::Matrix4d T23 = dh_matrix(theta3, d3, a3, alpha3);
        Eigen::Matrix4d T34 = dh_matrix(theta4, d4, a4, alpha4);
        Eigen::Matrix4d T45 = dh_matrix(theta5, d5, a5, alpha5);
        Eigen::Matrix4d T56 = dh_matrix(theta6, d6, a6, alpha6);

        // Multiplier les matrices de transformation pour obtenir T06
        Eigen::Matrix4d T02 = T01 * T12;
        Eigen::Matrix4d T03 = T02 * T23;
        Eigen::Matrix4d T04 = T03 * T34;
        Eigen::Matrix4d T05 = T04 * T45;
        Eigen::Matrix4d T06 = T05 * T56;
        // T60 = T06.inverse();

        return T06;
    }

private:
    double a1, d1, alpha1;
    double a2, d2, alpha2;
    double a3, d3, alpha3;
    double a4, d4, alpha4;
    double a5, d5, alpha5;
    double a6, d6, alpha6;
};

////////////////////////////////////////////////////////////////////////////////
// Helper function to perform lfilter equivalent in C++
// This implements a direct form IIR filter given coefficients b and a.
// It computes the filtered output for a given input signal vector x.
std::vector<double> lfilter(const std::vector<double> &b, const std::vector<double> &a, const std::vector<double> &x) {
    size_t N = x.size();
    size_t nb = b.size();
    size_t na = a.size();
    std::vector<double> y(N, 0.0);
    for (size_t n = 0; n < N; n++) {
        double acc = 0.0;
        // Apply numerator (b coefficients)
        for (size_t i = 0; i < nb; i++) {
            if (n >= i) {
                acc += b[i] * x[n - i];
            }
        }
        // Apply denominator (a coefficients), skipping a[0] which is assumed to be 1
        for (size_t j = 1; j < na; j++) {
            if (n >= j) {
                acc -= a[j] * y[n - j];
            }
        }
        y[n] = acc;
    }
    return y;
}

class OmniStateToTwistWithButton : public rclcpp::Node {
public:
    OmniStateToTwistWithButton() : Node("omni_to_twist_with_button_node") {
        // Initialisation de last_orientation
        last_orientation = omni_msgs::msg::OmniState();
        last_orientation.pose.orientation.x = 0.0;
        last_orientation.pose.orientation.y = 0.0;
        last_orientation.pose.orientation.z = 0.0;
        last_orientation.pose.orientation.w = 0.0;
        // Initialisation des variables
        auto now = this->get_clock()->now();
        timer = now.seconds();
        grey_button_pressed = false;  // État du bouton gris
        wrench_input = geometry_msgs::msg::WrenchStamped();
        wrench_msg = geometry_msgs::msg::WrenchStamped();
        // Historique des données pour le filtrage
        linear_x_history = std::vector<double>();
        linear_y_history = std::vector<double>();
        linear_z_history = std::vector<double>();
        angular_x_history = std::vector<double>();
        angular_y_history = std::vector<double>();
        angular_z_history = std::vector<double>();
        angular_w_history = std::vector<double>();

        Vecteurforceoutil = std::vector<double>();
        VecteurforceBase = std::vector<double>();

        // Paramètres du filtre
        cutoff_frequency = 2.5;  // Fréquence de coupure en Hz
        sampling_frequency = 500.0;  // Exemple : fréquence d'échantillonnage à ajuster si nécessaire

        // Pour reproduire butter(2, cutoff/(sampling_frequency/2), btype='low')
        // Nous utilisons des coefficients pré-calculés pour un filtre de 2ème ordre avec Wn = 0.01
        // Ces coefficients sont approximativement:
        b_coeff = {0.00041655, 0.00083311, 0.00041655};
        a_coeff = {1.0, -1.982386, 0.982041};

        // Subscriber au topic joint_state
        subscription_joint = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1000, std::bind(&OmniStateToTwistWithButton::joint_state_callback, this, _1));
        joint_angles = {0,0,0,0,0,0};

        // Subscriber au topic OmniState
        subscription_state = this->create_subscription<omni_msgs::msg::OmniState>(
            "/phantom_state", 1000, std::bind(&OmniStateToTwistWithButton::omni_state_callback, this, _1));

        // Subscriber au topic WrenchStamped
        subscription_wrench = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor_broadcaster/wrench", 100,
            std::bind(&OmniStateToTwistWithButton::wrench_callback, this, _1));

        // Publisher vers TwistStamped
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 1000);
        publisher_omni = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench", 1000);
        RCLCPP_INFO(this->get_logger(), "OmniState to TwistStamped with Button Control Node started.");
    }

private:
    // Member variables declarations
    omni_msgs::msg::OmniState last_orientation;
    double timer;
    bool grey_button_pressed;
    geometry_msgs::msg::WrenchStamped wrench_input;
    geometry_msgs::msg::WrenchStamped wrench_msg;
    std::vector<double> linear_x_history;
    std::vector<double> linear_y_history;
    std::vector<double> linear_z_history;
    std::vector<double> angular_x_history;
    std::vector<double> angular_y_history;
    std::vector<double> angular_z_history;
    std::vector<double> angular_w_history;
    std::vector<double> Vecteurforceoutil;
    std::vector<double> VecteurforceBase;
    double cutoff_frequency;
    double sampling_frequency;
    std::vector<double> b_coeff;
    std::vector<double> a_coeff;
    std::vector<double> joint_angles;

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint;
    rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr subscription_state;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_wrench;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_omni;

    // Callback for joint state
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_angles = msg->position;
    }

    // Function to filtrer les données avec un filtre passe-bas.
    double apply_filter(const std::vector<double>& data_history, int n) {
        if(data_history.size() < b_coeff.size()){
            return 0.0;  // Retourne 0 si pas assez de données
        }
        std::vector<double> filtered = lfilter(b_coeff, a_coeff, data_history);
        // Gestion des index négatifs (comme Python)
        int index = n;
        if(index < 0) {
            index = static_cast<int>(filtered.size()) + index;
        }
        if(index < 0 || index >= static_cast<int>(filtered.size())){
            return 0.0;
        }
        return filtered[index];
    }

    // Callback pour OmniState
    void omni_state_callback(const omni_msgs::msg::OmniState::SharedPtr msg) {
        double current_time = this->get_clock()->now().seconds();
        double dt = current_time - timer;
        timer = current_time;

        // Créer un message TwistStamped
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "base_link";
        //MGD

        // Copier les vitesses linéaires
        RCLCPP_INFO(this->get_logger(), "Message reçu sur le topic /wrench",msg->velocity.x);
        linear_x_history.push_back(msg->velocity.x * 100);
        linear_y_history.push_back(msg->velocity.y * 100);
        linear_z_history.push_back(msg->velocity.z * 100);
        angular_x_history.push_back(msg->pose.orientation.x);
        angular_y_history.push_back(msg->pose.orientation.y);
        angular_z_history.push_back(msg->pose.orientation.z);
        angular_w_history.push_back(msg->pose.orientation.w);

        // Création de q1 et q2 comme orientations
        geometry_msgs::msg::Quaternion q1;
        q1.x = apply_filter(angular_x_history, -2);
        q1.y = apply_filter(angular_y_history, -2);
        q1.z = apply_filter(angular_z_history, -2);
        q1.w = apply_filter(angular_w_history, -2);

        geometry_msgs::msg::Quaternion q2;
        q2.x = apply_filter(angular_x_history, -1);
        q2.y = apply_filter(angular_y_history, -1);
        q2.z = apply_filter(angular_z_history, -1);
        q2.w = apply_filter(angular_w_history, -1);
        // Orientation 
        // Calcul des vitesses angulaires
        if(angular_x_history.size() < 2 || angular_w_history.size() < 2 ||
           angular_y_history.size() < 2 || angular_z_history.size() < 2) {
            twist_msg.twist.angular.x = 0.0;
            twist_msg.twist.angular.y = 0.0;
            twist_msg.twist.angular.z = 0.0;
        } else {
            twist_msg.twist.angular.x = (2.0 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y)
            twist_msg.twist.angular.y = (2.0 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x)
            twist_msg.twist.angular.z = (2.0 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w)
            // twist_msg.twist.angular.x = (q2.x - q1.x) / dt;
            // twist_msg.twist.angular.y = (q2.y - q1.y) / dt;
            // twist_msg.twist.angular.z = (q2.z - q1.z) / dt;
        }

        // Appliquer le filtre
        twist_msg.twist.linear.x = -apply_filter(linear_x_history, -1);
        twist_msg.twist.linear.y = -apply_filter(linear_y_history, -1);
        twist_msg.twist.linear.z = apply_filter(linear_z_history, -1);

        // Filtrage de bruit par seuil (deadband)
        deadband_filter_noise(twist_msg, 0.00101);

        // Convertir la force en tableau numpy équivalent via Eigen vector
        Eigen::Vector3d force;
        force << wrench_msg.wrench.force.x,
                 wrench_msg.wrench.force.y,
                 wrench_msg.wrench.force.z;
        // Convertir la vitesse linéaire en Eigen vector
        Eigen::Vector3d linear_velocity;
        linear_velocity << twist_msg.twist.linear.x,
                           twist_msg.twist.linear.y,
                           twist_msg.twist.linear.z;

        // Calculer la norme au carré de la force (produit scalaire de la force avec elle-même)
        double norm = force.dot(force);

        // Vérifier si la norme de la force est supérieure à 10 (ici >2 dans le code)
        if (norm > 200) {
            if (force.dot(linear_velocity) > 0) {
                // twist_msg.twist.linear velocities annulées
                twist_msg.twist.linear.x = 0.0;
                twist_msg.twist.linear.y = 0.0;
                twist_msg.twist.linear.z = 0.0;
            }
        }
        // Publication des messages
        publisher->publish(twist_msg);
        //test
        // Mettre à jour l'orientation précédente
        last_orientation.pose.orientation = msg->pose.orientation;
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        /*
        Mise a jour des efforts
        */
        if (!joint_angles.empty()) {
            UR3Kinematics ur3;
            Eigen::Matrix4d T06 = ur3.fk_ur(joint_angles);
            Eigen::Matrix3d R06 = T06.block<3,3>(0,0);
            // with OptoForce(speed_hz=100, filter_hz=1.5, zero=True) as force_sensor:
            //     force = [force_sensor.read(only_latest_data=False).Fx,force_sensor.read(only_latest_data=False).Fy,force_sensor.read(only_latest_data=False).Fz]
            //     force[2] = force[2] / 10 
            //     force[1] = force[1] 
            //     force[0] = force[0] 
            //     print(force)
            // Convertir le vecteur de force en numpy array via Eigen
            Eigen::Vector3d F_outil;
            F_outil << wrench_input.wrench.force.x,
                       wrench_input.wrench.force.y,
                       wrench_input.wrench.force.z;
            // F_outil = [0,0,0]
            // Calculer les forces dans le repère de base
            Eigen::Vector3d F_base = R06 * F_outil;
            // Stocker les forces transformées
            VecteurforceBase.clear();
            VecteurforceBase.push_back(F_base(0));
            VecteurforceBase.push_back(F_base(1));
            VecteurforceBase.push_back(F_base(2));
            wrench_input = *msg;
            wrench_msg.wrench.force.x = VecteurforceBase[0] * 0.1;
            wrench_msg.wrench.force.y = VecteurforceBase[1] * 0.1;
            wrench_msg.wrench.force.z = VecteurforceBase[2] * 0.1;

            publisher_omni->publish(wrench_msg);
        }
    }

    void deadband_filter_noise(geometry_msgs::msg::TwistStamped &twist, double deadband) {
        // Appliquer un filtre deadband pour supprimer le bruit.
        if (std::fabs(twist.twist.linear.x) < deadband) {
            twist.twist.linear.x = 0.0;
        }
        if (std::fabs(twist.twist.linear.y) < deadband) {
            twist.twist.linear.y = 0.0;
        }
        if (std::fabs(twist.twist.linear.z) < deadband) {
            twist.twist.linear.z = 0.0;
        }
        if (std::fabs(twist.twist.angular.x) < deadband) {
            twist.twist.angular.x = 0.0;
        }
        if (std::fabs(twist.twist.angular.y) < deadband) {
            twist.twist.angular.y = 0.0;
        }
        if (std::fabs(twist.twist.angular.z) < deadband) {
            twist.twist.angular.z = 0.0;
        }
    }
};

int main(int argc, char * argv[])   
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniStateToTwistWithButton>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
