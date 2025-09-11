#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/bool.hpp>


class UnityControl : public rclcpp::Node {
public:
    UnityControl() : Node("unity_control"), pos_x_(0.0), pos_y_(0.0), pos_z_(0.0), rot_x_(0.0), rot_y_(0.0), rot_z_(0.0), limit_off_(false) {
    
    effector_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tcp_pose_broadcaster/pose", 10, std::bind(&UnityControl::pose_effector_callback, this, std::placeholders::_1));

    unity_position_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/unity_position", 10, std::bind(&UnityControl::pose_unity_callback, this, std::placeholders::_1));

    robot_limit = this->create_subscription<std_msgs::msg::Bool>(
      "/robot_limit", 10, std::bind(&UnityControl::limit_out_callback, this, std::placeholders::_1));

    // client pour envoyer des scripts URScript
    urscript_client_ = this->create_publisher<std_msgs::msg::String>(
      "/urscript_interface/script_command", 10);
  }

private:

  void limit_out_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    
    limit_off_ = msg->data;
  }

  void pose_effector_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // MAJ de la position initiale du robot

    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    double angle = q.getAngle();
    tf2::Vector3 axis = q.getAxis();

    pos_x_ = msg->pose.position.x;
    pos_y_ = msg->pose.position.y;
    pos_z_ = msg->pose.position.z;
    rot_x_ = axis.x() * angle;
    rot_y_ = axis.y() * angle; 
    rot_z_ = axis.z() * angle;
  }
  void pose_unity_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {

    float new_x = msg->position.x;
    float new_y = msg->position.y;
    float new_z = msg->position.z;


    if ((std::abs(new_x - pos_x_) > 0.001 || std::abs(new_y - pos_y_) > 0.001 || std::abs(new_z - pos_z_) > 0.001) && !limit_off_) {
      // génère une commande URScript
      
      std::string script = "movel(p[" +
        std::to_string(new_x) + "," +
        std::to_string(new_y) + "," +
        std::to_string(new_z) + "," +
        std::to_string(rot_x_) + "," +
        std::to_string(rot_y_) + "," +
        std::to_string(rot_z_) + "], a=0.1, v=0.1)";

      // publie le script URScript
      auto msg = std_msgs::msg::String();
      msg.data = script;
      urscript_client_->publish(msg);
      //RCLCPP_INFO(this->get_logger(), "Envoyé URScript: %s", script.c_str());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr effector_position_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr unity_position_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urscript_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_limit;
  float pos_x_, pos_y_, pos_z_;
  float rot_x_, rot_y_, rot_z_;
  bool limit_off_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnityControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}