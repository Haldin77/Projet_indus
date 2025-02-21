#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include <eigen3/Eigen/Dense>

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

static const float SPEED_LINEAR_SCALE = 0.01;
static const float WRENCH_SCALE = 0.1;
static const float WRENCH_LIMIT = 2;
static const float DEADBAND = 0.00101;

/////////////////////////////////////////////////////////



class UR3Kinematics {
    public:

    UR3Kinematics() {
        a1 = 0.0; d1 = 0.15185; alpha1 = 0.5 * M_PI;
        a2 = -0.24355; d2 = 0.0; alpha2 = 0.0;
        a3 = -0.2132; d3 = 0.0; alpha3 = 0.0;
        a4 = 0.0; d4 = 0.13105; alpha4 = 0.5 * M_PI;
        a5 = 0.0; d5 = 0.08535; alpha5 = -0.5 * M_PI;
        a6 = 0.0; d6 = 0.0921; alpha6 = 0.0;
    }

    Eigen::Matrix4d dh_matrix(double theta, double d, double a, double alpha) {
        Eigen::Matrix4d T;
        T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
             sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
             0, sin(alpha), cos(alpha), d,
             0, 0, 0, 1;
        return T;
    }

    Eigen::Matrix4d fk_ur(const std::vector<double>& joint_angles) {
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


class OmniStateToTwistWithButton : public rclcpp::Node {
    public:
    OmniStateToTwistWithButton() : Node("omni_to_twist_with_button_node") {
        
        /////////////////////[  SUBSCRITION  ]////////////////////////
        // Subscriber au topic /joint_state
        subscription_joint = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", QUEUE_LENGTH, std::bind(&OmniStateToTwistWithButton::joint_state_callback, this, std::placeholders::_1));
        
        // Subscriber au topic /phantom_state
        subscription_state = this->create_subscription<omni_msgs::msg::OmniState>(
            "/phantom_state", QUEUE_LENGTH, std::bind(&OmniStateToTwistWithButton::omni_state_callback, this, std::placeholders::_1));
        
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
        for(int i=0; i<LEN_HISTORY; i++){
            //initialisation à 0
            linear_x_history[i] = 0.0;
            linear_y_history[i] = 0.0;
            linear_z_history[i] = 0.0;
            angular_x_history[i] = 0.0;
            angular_y_history[i] = 0.0;
            angular_z_history[i] = 0.0;
            angular_w_history[i] = 0.0;
        }

        //______TIME______________
        auto now = this->get_clock()->now();
        timer = now.seconds();
    }

    private:

    //////////////////////////[  VARIABLES  ]///////////////////////////
    //_________FILTRE______________
    float cutoff_frequency; //Frequence de coupure en Hz
    float sampling_frequency; //Frequence d'echantillonage
    const int size_a = 5;
    const int size_b = 5;
    float a_coef[5] = {1.0, -3.9433, 5.8326, -3.8352, 0.9459};
    float b_coef[5] = {0.0098, -0.0389, 0.0582, -0.0389, 0.0098};

    //________HISTORIQUE____________
    double linear_x_history[LEN_HISTORY], linear_y_history[LEN_HISTORY], linear_z_history[LEN_HISTORY];
    double angular_x_history[LEN_HISTORY], angular_y_history[LEN_HISTORY], angular_z_history[LEN_HISTORY], angular_w_history[LEN_HISTORY];

    //________FORCE_________________
    double VecteurForceOutil[3];
    double VecteurForceBase[3];
    geometry_msgs::msg::WrenchStamped wrench_input = geometry_msgs::msg::WrenchStamped();     // a garder ??
    geometry_msgs::msg::WrenchStamped wrench_msg = geometry_msgs::msg::WrenchStamped();       // a garder ??

    //________TIME__________________
    double timer;

    //_______ARTICULATIONS__________
    std::vector<double> joint_angles;
    double old_filtered_orientation[4] = {0.0, 0.0, 0.0, 0.0};

    ///////////////////////[  SUB_AND_PUB  ]///////////////////////////
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint;
    rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr subscription_state;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_wrench;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_omni;

    //////////////////////[  METHODE  ]////////////////////////////////
    
    //suppression de apply filter, on va faire la verification direct

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

    // renvoie la derniere data filtre en fonction des coefficients du filtre b et a et de l'historique des mesures non filtre
    double lfilter(const double* data){
        double output = 0.0;
        //application des coefficients b
        for(int i = 0; i < size_b; i++){
            if(LEN_HISTORY >= i)
                output += b_coef[i] * data[LEN_HISTORY-i];
        }
        //application des coefficients a
        for(int j = 1; j < size_a; j++){
            if(LEN_HISTORY >= j)
                output -= a_coef[j] * data[LEN_HISTORY-j];
        }
        return output/a_coef[0];
    }


    //////////////////////[  CALLBACK  ]///////////////////////////////
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_angles = msg->position;
    }

    void omni_state_callback(const omni_msgs::msg::OmniState::SharedPtr msg) {
        auto now = this->get_clock()->now();
        double current_time = now.seconds();
        double dt = current_time - timer;
        timer = current_time;

        // Créer un message TwistStamped
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "base_link";

        // decallage de l'historique (push back)
        for(int i = 0; i < LEN_HISTORY-1; i++){
            linear_x_history[i]  =  linear_x_history[i+1];
            linear_y_history[i]  =  linear_y_history[i+1];
            linear_z_history[i]  =  linear_z_history[i+1];
            angular_x_history[i] = angular_x_history[i+1];
            angular_y_history[i] = angular_y_history[i+1];
            angular_z_history[i] = angular_z_history[i+1];
            angular_w_history[i] = angular_w_history[i+1];
        }

        //__________________[  VITESSES  ]_______________________
        // Recuperation des vitesses linéaire du phantom omni
        linear_x_history[LEN_HISTORY-1] = msg->velocity.x * SPEED_LINEAR_SCALE;
        linear_y_history[LEN_HISTORY-1] = msg->velocity.y * SPEED_LINEAR_SCALE;
        linear_z_history[LEN_HISTORY-1] = msg->velocity.z * SPEED_LINEAR_SCALE;

        // Application du filtre sur les vitesses linéaires
        twist_msg.twist.angular.x = lfilter(linear_x_history);
        twist_msg.twist.angular.y = lfilter(linear_y_history);
        twist_msg.twist.angular.z = lfilter(linear_z_history);

        //__________________[  ORIENTATIONS  ]___________________
        // Recuperation des orientations du phantom omni
        angular_x_history[LEN_HISTORY-1] = msg->pose.orientation.x;
        angular_y_history[LEN_HISTORY-1] = msg->pose.orientation.y;
        angular_z_history[LEN_HISTORY-1] = msg->pose.orientation.z;
        angular_w_history[LEN_HISTORY-1] = msg->pose.orientation.w;

        // Application du filtre sur les orientations
        double angular_x_filtered = lfilter(angular_x_history);
        double angular_y_filtered = lfilter(angular_y_history);
        double angular_z_filtered = lfilter(angular_z_history);
        double angular_w_filtered = lfilter(angular_w_history);

        // Calcul des vitesses d'orientation
        twist_msg.twist.angular.x = (angular_x_filtered - old_filtered_orientation[0]) / dt;
        twist_msg.twist.angular.y = (angular_y_filtered - old_filtered_orientation[1]) / dt;
        twist_msg.twist.angular.z = (angular_z_filtered - old_filtered_orientation[2]) / dt;
        //twist_msg.twist.angular.w = (angular_w_filtered - old_filtered_orientation[3]) / dt;

        // Mise à jour de old_filtered_orientation (stockage de l'orientation précédente)
        old_filtered_orientation[0] = angular_x_filtered;
        old_filtered_orientation[1] = angular_y_filtered;
        old_filtered_orientation[2] = angular_z_filtered;
        old_filtered_orientation[3] = angular_w_filtered;
       
        //_______________[  SECURITE D'EFFORT  ]________________________
        // annule les vitesses si la force reçu est supérieur à un certain seuil
        double force[3] = {
            wrench_msg.wrench.force.x,
            wrench_msg.wrench.force.y,
            wrench_msg.wrench.force.z
        };
        
        // calcul de la norme de la force
        double norme_force = 0;
        for(int i = 0; i<3; i++){
            norme_force += force[i] * force[i];
        }
        norme_force = sqrt(norme_force);

        if(norme_force > WRENCH_LIMIT){
            twist_msg.twist.linear.x = 0.0;
            twist_msg.twist.linear.y = 0.0;
            twist_msg.twist.linear.z = 0.0;
        }

        //filtrage des valeurs trop faible (consideré comme 0)
        deadband_filter_noise(twist_msg, DEADBAND);

        // Publication des consignes de vitesse (angulaire et lineaire)
        publisher_->publish(twist_msg);
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
        if(!joint_angles.empty()){
            UR3Kinematics ur3;
            Eigen::Matrix4d T06 = ur3.fk_ur(joint_angles);
            Eigen::Matrix3d R06 = T06.block<3,3>(0,0);
            Eigen::Vector3d F_effecteur;
            F_effecteur << wrench_input.wrench.force.x,
                       wrench_input.wrench.force.y,
                       wrench_input.wrench.force.z;

            Eigen::Vector3d F_base = R06 * F_effecteur;
            wrench_input = *msg;    // positionné ici? utile? pq pas utiliser wrench_msg?
            wrench_msg.wrench.force.x = F_base[0] * WRENCH_SCALE;
            wrench_msg.wrench.force.y = F_base[1] * WRENCH_SCALE;
            wrench_msg.wrench.force.z = F_base[2] * WRENCH_SCALE;

            publisher_omni->publish(wrench_msg);
        }
    }
};

int main(int argc, char * argv[]){
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