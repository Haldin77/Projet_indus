#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" // Importer cette bibliothèque

class URSensorToHaptic : public rclcpp::Node
{
public:
    static const int NUM_SPINNERS = 1;
    static const int QUEUE_LENGTH = 10;
    static const std::string package_name;

    URSensorToHaptic()
        : Node(package_name + "_haptic"), movement_active_(false)
    {
        // Charger la configuration YAML
        std::string teleop_config_location = ament_index_cpp::get_package_share_directory(package_name) + "/config/teleop_config.yaml"; // Utilisation correcte
        teleop_config_ = YAML::LoadFile(teleop_config_location);
        force_scale_ = teleop_config_["force_scale"].as<double>();

        // Souscrire aux messages
        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/wrench", QUEUE_LENGTH, std::bind(&URSensorToHaptic::wrenchCallback, this, std::placeholders::_1));

        // Obtenir le nom de l'omni (paramètre ROS 2)
        this->get_parameter_or("/omni_state/omni_name", omni_name_, std::string("omni_device"));

        button_sub_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
            omni_name_ + "/button", QUEUE_LENGTH, std::bind(&URSensorToHaptic::buttonCallback, this, std::placeholders::_1));

        haptic_pub_ = this->create_publisher<omni_msgs::msg::OmniFeedback>(
            omni_name_ + "/force_feedback", QUEUE_LENGTH);

        // Activer le mouvement
        movement_active_ = false;
    }

private:
    void buttonCallback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg)
    {
        movement_active_ = static_cast<bool>(msg->grey_button);
    }

    void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        omni_msgs::msg::OmniFeedback feedback_vector;
        geometry_msgs::msg::Vector3 force_vector;
        geometry_msgs::msg::Vector3 pose_vector;  // "Lock Position" n'est pas pris en compte

        if (movement_active_)
        {
            force_vector = msg->wrench.force;
            transformForceToTouchFrame(force_vector);
        }

        scaleForceVector(force_vector);

        feedback_vector.force = force_vector;
        feedback_vector.position = pose_vector;
        haptic_pub_->publish(feedback_vector);
    }

    // Transformation des forces pour correspondre au cadre de "touch"
    void transformForceToTouchFrame(geometry_msgs::msg::Vector3& force_vector)
    {
        // Négation des valeurs pour définir la direction de la "réaction"
        force_vector.x *= -1;
        force_vector.z *= -1;
    }

    // Mise à l'échelle du vecteur de force
    void scaleForceVector(geometry_msgs::msg::Vector3& force_vector)
    {
        force_vector.x *= force_scale_;
        force_vector.y *= force_scale_;
        force_vector.z *= force_scale_;
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_sub_;
    rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr haptic_pub_;
    std::string omni_name_;

    bool movement_active_;
    YAML::Node teleop_config_;
    double force_scale_;
};

const std::string URSensorToHaptic::package_name = "ur_to_touch_haptic_teleoperation";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto to_haptic = std::make_shared<URSensorToHaptic>();
    rclcpp::spin(to_haptic);
    rclcpp::shutdown();
    return 0;
}
