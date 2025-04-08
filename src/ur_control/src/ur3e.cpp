#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"

/////////////[  PARAMETRAGE ALGORITHME  ]///////////////

static const int LEN_HISTORY = 10;
static const int QUEUE_LENGTH = 10;

///////////[  PARAMETRAGE TELEOPERATION  ]//////////////

static const float SPEED_LINEAR_SCALE = 10;
static const float WRENCH_SCALE = 1.0;
static const float WRENCH_LIMIT = 200.0;
static const float DEADBAND = 0.00111;
static const float FORCE_DEADBAND = 0.005;

/////////////////////////////////////////////////////////

struct Quaternion
{
    double w, x, y, z;

    // Constructeur par défaut
    Quaternion(double w_val = 0, double x_val = 0, double y_val = 0, double z_val = 0)
        : w(w_val), x(x_val), y(y_val), z(z_val) {}

    // Fonction pour multiplier deux quaternions
    Quaternion multiply(const Quaternion &q) const
    {
        double w_res = w * q.w - x * q.x - y * q.y - z * q.z;
        double x_res = w * q.x + x * q.w + y * q.z - z * q.y;
        double y_res = w * q.y - x * q.z + y * q.w + z * q.x;
        double z_res = w * q.z + x * q.y - y * q.x + z * q.w;

        return Quaternion(w_res, x_res, y_res, z_res);
    }

    // Affichage du quaternion
    void print() const
    {
        std::cout << "(" << w << ", " << x << ", " << y << ", " << z << ")" << std::endl;
    }
};

class UR3Kinematics
{
public:
    UR3Kinematics()
    {
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
        a6 = 0.0;
        d6 = 0.0921;
        alpha6 = 0.0;
    }

    Eigen::Matrix4d dh_matrix(double theta, double d, double a, double alpha)
    {
        Eigen::Matrix4d T;
        T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
            sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1;
        return T;
    }

    Eigen::Matrix4d fk_ur(const std::vector<double> &joint_angles)
    {
        Eigen::Matrix4d T01 = dh_matrix(joint_angles[5], d1, a1, alpha1);
        Eigen::Matrix4d T12 = dh_matrix(joint_angles[0], d2, a2, alpha2);
        Eigen::Matrix4d T23 = dh_matrix(joint_angles[1], d3, a3, alpha3);
        Eigen::Matrix4d T34 = dh_matrix(joint_angles[2], d4, a4, alpha4);
        Eigen::Matrix4d T45 = dh_matrix(joint_angles[3], d5, a5, alpha5);
        Eigen::Matrix4d T56 = dh_matrix(joint_angles[4], d6, a6, alpha6);

        return T01 * T12 * T23 * T34 * T45 * T56;
    }

private:
    double a1, d1, alpha1;
    double a2, d2, alpha2;
    double a3, d3, alpha3;
    double a4, d4, alpha4;
    double a5, d5, alpha5;
    double a6, d6, alpha6;
};

class OmniStateToTwistWithButton : public rclcpp::Node
{
private:
    //////////////////////////[  VARIABLES  ]///////////////////////////
    geometry_msgs::msg::TwistStamped twist_msg;
    UR3Kinematics ur3;
    Eigen::Vector3d F_base;
    double norm;
    //_________FILTRE______________
    float cutoff_frequency;   // Frequence de coupure en Hz
    float sampling_frequency; // Frequence d'echantillonage
    const int size_a = 3;
    const int size_b = 3;
    float a_coef[3] = {1, -1.9556, 0.9565};
    float Kp_force = 0.6;
    float b_coef[3] = {0.0002414, 0.0004827, 0.0002414};

    //________HISTORIQUE____________
    double linear_x_history[LEN_HISTORY], linear_y_history[LEN_HISTORY], linear_z_history[LEN_HISTORY];
    double angular_x_history[LEN_HISTORY], angular_y_history[LEN_HISTORY], angular_z_history[LEN_HISTORY], angular_w_history[LEN_HISTORY];
    double linear_x_history_filtered[LEN_HISTORY], linear_y_history_filtered[LEN_HISTORY], linear_z_history_filtered[LEN_HISTORY];
    double angular_x_history_filtered[LEN_HISTORY], angular_y_history_filtered[LEN_HISTORY], angular_z_history_filtered[LEN_HISTORY], angular_w_history_filtered[LEN_HISTORY];
    double force_x_history[LEN_HISTORY], force_y_history[LEN_HISTORY], force_z_history[LEN_HISTORY];
    double force_x_history_filtered[LEN_HISTORY], force_y_history_filtered[LEN_HISTORY], force_z_history_filtered[LEN_HISTORY];
    //________FORCE_________________
    double VecteurForceOutil[3];
    double VecteurForceBase[3];
    geometry_msgs::msg::WrenchStamped wrench_input = geometry_msgs::msg::WrenchStamped(); // a garder ??
    geometry_msgs::msg::WrenchStamped wrench_msg = geometry_msgs::msg::WrenchStamped();   // a garder ??

    //________TIME__________________
    double timer;

    //_______ARTICULATIONS__________
    std::vector<double> joint_angles;
    double old_filtered_orientation[4] = {0.0, 0.0, 0.0, 0.0};
    float z_scale = 1;
    ///////////////////////[  SUB_AND_PUB  ]///////////////////////////
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint;
    rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr subscription_state;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_wrench;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_omni;

public:
    OmniStateToTwistWithButton() : Node("omni_to_twist_with_button_node")
    {

        /////////////////////[  SUBSCRITION  ]////////////////////////
        // Subscriber au topic /joint_state
        subscription_joint = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", QUEUE_LENGTH, std::bind(&OmniStateToTwistWithButton::joint_state_callback, this, std::placeholders::_1));

        // Subscriber au topic /Haply_state
        subscription_state = this->create_subscription<omni_msgs::msg::OmniState>(
            "/Haply_state", QUEUE_LENGTH, std::bind(&OmniStateToTwistWithButton::omni_state_callback, this, std::placeholders::_1));

        // Subscriber au topic /force_torque_sensor_broadcaster/wrench
        subscription_wrench = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor_broadcaster/wrench", QUEUE_LENGTH, std::bind(&OmniStateToTwistWithButton::wrench_callback, this, std::placeholders::_1));

        /////////////////////[  PUBLISHER  ]//////////////////////////
        // Publisher sur le topic /servo_node/delta_twist_cmds
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", QUEUE_LENGTH);

        // Publisher sur le topic /wrench
        publisher_omni = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/wrench", QUEUE_LENGTH);
        RCLCPP_INFO(this->get_logger(), "OmniState to TwistStamped with Button Control Node started.");

        /////////////////////[  INITIALISATION  ]////////////////
        //________FILTRE___________
        cutoff_frequency = 2.5;
        sampling_frequency = 500.0;

        //_______HISTORIQUE________
        for (int i = 0; i < LEN_HISTORY; i++)
        {
            // initialisation à 0
            linear_x_history[i] = 0.0;
            linear_y_history[i] = 0.0;
            linear_z_history[i] = 0.0;
            angular_x_history[i] = 0.0;
            angular_y_history[i] = 0.0;
            angular_z_history[i] = 0.0;
            angular_w_history[i] = 0.0;
            linear_x_history_filtered[i] = 0.0;
            linear_y_history_filtered[i] = 0.0;
            linear_z_history_filtered[i] = 0.0;
            angular_x_history_filtered[i] = 0.0;
            angular_y_history_filtered[i] = 0.0;
            angular_z_history_filtered[i] = 0.0;
            angular_w_history_filtered[i] = 0.0;
            force_x_history[i] = 0.0;
            force_y_history[i] = 0.0;
            force_z_history[i] = 0.0;
            force_x_history_filtered[i] = 0.0;
            force_y_history_filtered[i] = 0.0;
            force_z_history_filtered[i] = 0.0;
        }

        //______TIME______________
        auto now = this->get_clock()->now();
        timer = now.seconds();
    }
    //////////////////////[  METHODE  ]////////////////////////////////

    // suppression de apply filter, on va faire la verification direct

    void deadband_filter_noise(geometry_msgs::msg::TwistStamped &twist, double deadband)
    {
        // Appliquer un filtre deadband pour supprimer le bruit.
        if (std::fabs(twist.twist.linear.x) < deadband)
        {
            twist.twist.linear.x = 0.0;
        }
        if (std::fabs(twist.twist.linear.y) < deadband)
        {
            twist.twist.linear.y = 0.0;
        }
        if (std::fabs(twist.twist.linear.z) < deadband)
        {
            twist.twist.linear.z = 0.0;
        }
        if (std::fabs(twist.twist.angular.x) < deadband)
        {
            twist.twist.angular.x = 0.0;
        }
        if (std::fabs(twist.twist.angular.y) < deadband)
        {
            twist.twist.angular.y = 0.0;
        }
        if (std::fabs(twist.twist.angular.z) < deadband)
        {
            twist.twist.angular.z = 0.0;
        }
    }

    // renvoie la derniere data filtre en fonction des coefficients du filtre b et a et de l'historique des mesures non filtre
    double lfilter(const double *x, const double *y)
    {
        double output = 0.0;
        // application des coefficients b
        for (int i = 0; i < size_b; i++)
        {
            if (LEN_HISTORY >= i)
                output += b_coef[i] * x[LEN_HISTORY - i - 1];
        }
        // application des coefficients a
        for (int j = 1; j < size_a; j++)
        {
            if (LEN_HISTORY >= j)
                output -= a_coef[j] * y[LEN_HISTORY - j - 1];
        }
        return output / a_coef[0];
    }
    double lfilter_forces(const double *x, const double *y)
    {
        double output = 0.0;
        // application des coefficients b
        const int size_a_forces = 3;
        const int size_b_forces = 3;
        float a_coef_forces[5] = {1, -1.9378, 0.9397};
        float b_coef_forces[3] = {0.0004690, 0.0009379, 0.0004690};
        for (int i = 0; i < size_b_forces; i++)
        {
            if (LEN_HISTORY >= i)
                output += b_coef_forces[i] * x[LEN_HISTORY - i - 1];
        }
        // application des coefficients a
        for (int j = 1; j < size_a_forces; j++)
        {
            if (LEN_HISTORY >= j)
                output -= a_coef_forces[j] * y[LEN_HISTORY - j - 1];
        }
        return output / a_coef_forces[0];
    }

    //////////////////////[  CALLBACK  ]///////////////////////////////
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_angles = msg->position;
    }

    void omni_state_callback(const omni_msgs::msg::OmniState::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double current_time = now.seconds();
        double dt = current_time - timer;
        timer = current_time;
        if (msg->locked)
        {
            z_scale = 10.0;
        }
        else
        {
            z_scale = 1.0;
        }
        
        
        // Créer un message TwistStamped
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "base_link";

        // decallage de l'historique (push back)
        for (int i = 0; i < LEN_HISTORY - 1; i++)
        {
            linear_x_history[i] = linear_x_history[i + 1];
            linear_y_history[i] = linear_y_history[i + 1];
            linear_z_history[i] = linear_z_history[i + 1];
            angular_x_history[i] = angular_x_history[i + 1];
            angular_y_history[i] = angular_y_history[i + 1];
            angular_z_history[i] = angular_z_history[i + 1];
            angular_w_history[i] = angular_w_history[i + 1];
            linear_x_history_filtered[i] = linear_x_history_filtered[i + 1];
            linear_y_history_filtered[i] = linear_y_history_filtered[i + 1];
            linear_z_history_filtered[i] = linear_z_history_filtered[i + 1];
            angular_x_history_filtered[i] = angular_x_history_filtered[i + 1];
            angular_y_history_filtered[i] = angular_y_history_filtered[i + 1];
            angular_z_history_filtered[i] = angular_z_history_filtered[i + 1];
            angular_w_history_filtered[i] = angular_w_history_filtered[i + 1];
        }

        //__________________[  VITESSES  ]_______________________
        // Recuperation des vitesses linéaire du Haply omni
        linear_x_history[LEN_HISTORY - 1] = msg->velocity.x * SPEED_LINEAR_SCALE;
        linear_y_history[LEN_HISTORY - 1] = msg->velocity.y * SPEED_LINEAR_SCALE;
        linear_z_history[LEN_HISTORY - 1] = msg->velocity.z * SPEED_LINEAR_SCALE;

        // Application du filtre sur les vitesses linéaires
        linear_x_history_filtered[LEN_HISTORY - 1] = lfilter(linear_x_history, linear_x_history_filtered);
        linear_y_history_filtered[LEN_HISTORY - 1] = lfilter(linear_y_history, linear_y_history_filtered);
        linear_z_history_filtered[LEN_HISTORY - 1] = lfilter(linear_z_history, linear_z_history_filtered);

        twist_msg.twist.linear.x = -linear_x_history_filtered[LEN_HISTORY - 1];
        twist_msg.twist.linear.y = -linear_y_history_filtered[LEN_HISTORY - 1];
        twist_msg.twist.linear.z = linear_z_history_filtered[LEN_HISTORY - 1];

        //__________________[  ORIENTATIONS  ]___________________
        // Recuperation des orientations du Haply omni

        double angle = 0.0;                   // en degrés
        double radian = angle * M_PI / 180.0; // Convertir l'angle en radians
        double cos_half_angle = cos(radian / 2);
        double sin_half_angle = sin(radian / 2);
        Quaternion q_rotation(cos_half_angle, sin_half_angle, 0, 0);
        Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        Quaternion q_new = q_rotation.multiply(q);
        // Quaternion qrot2(cos_half_angle, 0, sin_half_angle, 0);
        // q_new = qrot2.multiply(q_new);
        angular_x_history[LEN_HISTORY - 1] = msg->pose.orientation.x;
        angular_y_history[LEN_HISTORY - 1] = msg->pose.orientation.y;
        angular_z_history[LEN_HISTORY - 1] = msg->pose.orientation.z;
        angular_w_history[LEN_HISTORY - 1] = msg->pose.orientation.w;
        geometry_msgs::msg::Quaternion q1;
        q1.x = old_filtered_orientation[0];
        q1.y = old_filtered_orientation[1];
        q1.z = old_filtered_orientation[2];
        q1.w = old_filtered_orientation[3];
        // Application du filtre sur les orientations
        angular_x_history_filtered[LEN_HISTORY - 1] = lfilter(angular_x_history, angular_x_history_filtered);
        angular_y_history_filtered[LEN_HISTORY - 1] = lfilter(angular_y_history, angular_y_history_filtered);
        angular_z_history_filtered[LEN_HISTORY - 1] = lfilter(angular_z_history, angular_z_history_filtered);
        angular_w_history_filtered[LEN_HISTORY - 1] = lfilter(angular_w_history, angular_w_history_filtered);

        geometry_msgs::msg::Quaternion q2;
        q2.x = angular_x_history_filtered[LEN_HISTORY - 1];
        q2.y = angular_y_history_filtered[LEN_HISTORY - 1];
        q2.z = angular_z_history_filtered[LEN_HISTORY - 1];
        q2.w = angular_w_history_filtered[LEN_HISTORY - 1];
        // Calcul des vitesses d'orientation
        std::cout << "x: " << angular_x_history_filtered[LEN_HISTORY - 1] << std::endl;
        twist_msg.twist.angular.z = -(2.0 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y)*z_scale;
        twist_msg.twist.angular.y = -(2.0 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x);
        twist_msg.twist.angular.x = (2.0 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w);
        // twist_msg.twist.angular.w = (angular_w_filtered - old_filtered_orientation[3]) / dt;

        // Mise à jour de old_filtered_orientation (stockage de l'orientation précédente)
        old_filtered_orientation[0] = angular_x_history_filtered[LEN_HISTORY - 1];
        old_filtered_orientation[1] = angular_y_history_filtered[LEN_HISTORY - 1];
        old_filtered_orientation[2] = angular_z_history_filtered[LEN_HISTORY - 1];
        old_filtered_orientation[3] = angular_w_history_filtered[LEN_HISTORY - 1];

        //_______________[  SECURITE D'EFFORT  ]________________________
        // annule les vitesses si la force reçu est supérieur à un certain seuil
        Eigen::Vector3d force_vector(F_base[0], F_base[1], F_base[2]);
        Eigen::Vector3d linear_velocity_outil(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z); // Exemple de vitesse linéaire

        norm = force_vector.squaredNorm(); // Norme au carré
        RCLCPP_INFO(this->get_logger(), "NORme : %f", norm);
        if (norm > WRENCH_LIMIT)
        {
            // Réinitialiser les vitesses
            double dot_product = force_vector.dot(linear_velocity_outil); // Produit scalaire
            if (dot_product < 0)
            {
                twist_msg.twist.linear.x = 0.0;
                twist_msg.twist.linear.y = 0.0;
                twist_msg.twist.linear.z = 0.0;
                twist_msg.twist.angular.x = 0.0;
                twist_msg.twist.angular.y = 0.0;
                twist_msg.twist.angular.z = 0.0;
            }
        }

        // filtrage des valeurs trop faible (consideré comme 0)
        deadband_filter_noise(twist_msg, DEADBAND);

        // Publication des consignes de vitesse (angulaire et lineaire)
        publisher_->publish(twist_msg);

        //calcul du temps de calcul
        double end_time = (this->get_clock()->now()).seconds();
        double delay = end_time - current_time;
        std::cout << "delay : " << delay << std::endl;
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if (!joint_angles.empty())
        {
            wrench_input = *msg;
            Eigen::Matrix4d T06 = ur3.fk_ur(joint_angles);
            Eigen::Matrix3d R06 = T06.block<3, 3>(0, 0);
            Eigen::Vector3d F_effecteur;
            auto F_previous = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d linear_velocity_outil(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);
            F_effecteur << wrench_input.wrench.force.x,
                wrench_input.wrench.force.y,
                wrench_input.wrench.force.z;

            F_base = R06 * F_effecteur;
            force_x_history[LEN_HISTORY - 1] = F_base[0];
            force_y_history[LEN_HISTORY - 1] = F_base[1];
            force_z_history[LEN_HISTORY - 1] = F_base[2];

            force_x_history_filtered[LEN_HISTORY - 1] = lfilter_forces(force_x_history, force_x_history_filtered);
            force_y_history_filtered[LEN_HISTORY - 1] = lfilter_forces(force_y_history, force_y_history_filtered);
            force_z_history_filtered[LEN_HISTORY - 1] = lfilter_forces(force_z_history, force_z_history_filtered);

            float alpha = 0.09;//(2 * M_PI * 3 * 500) / (2 * M_PI * 3 * 500 + 1);
            Eigen::Vector3d F_base_filtre;

            F_base_filtre[0] = F_base[0] * alpha + (1 - alpha) * F_previous[0];
            F_base_filtre[1] = F_base[1] * alpha + (1 - alpha) * F_previous[1];
            F_base_filtre[2] = F_base[2] * alpha + (1 - alpha) * F_previous[2];
            // positionné ici? utile? pq pas utiliser wrench_msg?
            if (norm < 20 )
            {
                Kp_force = 1.0;
            }
            else if (norm < 200)
            {
                Kp_force = 1.0*norm/10.0;
            }
            else{
                Kp_force = 20.0;
            }
            
            
            F_base_filtre[0] = F_base[0] * WRENCH_SCALE + Kp_force * twist_msg.twist.linear.x;
            F_base_filtre[1] = F_base[1] * WRENCH_SCALE + Kp_force * twist_msg.twist.linear.y;
            F_base_filtre[2] = F_base[2] * WRENCH_SCALE - Kp_force * twist_msg.twist.linear.z;
            F_base_filtre[0] = F_base_filtre[0] * alpha + (1 - alpha) * F_previous[0];
            F_base_filtre[1] = F_base_filtre[1] * alpha + (1 - alpha) * F_previous[1];
            F_base_filtre[2] = F_base_filtre[2] * alpha + (1 - alpha) * F_previous[2];
            // if (abs(F_base[0]) < FORCE_DEADBAND)
            // {
            //     wrench_msg.wrench.force.x = 0.0;
            // }
            // if (abs(F_base[1]) < FORCE_DEADBAND)
            // {
            //     wrench_msg.wrench.force.y = 0.0;
            // }
            // if (abs(F_base[2]) < FORCE_DEADBAND)
            // {
            //     wrench_msg.wrench.force.z = 0.0;
            // }
            F_previous = F_base_filtre;
            wrench_msg.wrench.force.x = F_base_filtre[0];
            wrench_msg.wrench.force.y = F_base_filtre[1];
            wrench_msg.wrench.force.z = F_base_filtre[2];

            publisher_omni->publish(wrench_msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniStateToTwistWithButton>();
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
