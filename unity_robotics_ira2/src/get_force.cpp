#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>



class GetForce : public rclcpp::Node
{
public:
  GetForce()
  : Node("get_force"),
    pose_ready_(false),                       
    tcp_rotation_(Eigen::Vector3d::Zero())
  {
    force_tcp_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/force_torque_sensor_broadcaster/wrench", 10,
      std::bind(&GetForce::force_callback, this, std::placeholders::_1));

    pose_tcp_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tcp_pose_broadcaster/pose", 10,
      std::bind(&GetForce::pose_callback, this, std::placeholders::_1));
  }

private:
  void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    force_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);

    if (pose_ready_) {
      Eigen::Matrix3d R_B_E = R_B_E_;         
      Eigen::Vector3d force_in_base = R_B_E * force_;
      RCLCPP_INFO(get_logger(), "Force base: [%.2f, %.2f, %.2f]",
                  force_in_base.x(), force_in_base.y(), force_in_base.z());
    } else {
      RCLCPP_WARN(get_logger(), "TCP rotation not yet received.");
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                      msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    tcp_rotation_ = Eigen::Vector3d(r, p, y);

    R_B_E_ << m[0][0], m[0][1], m[0][2],
              m[1][0], m[1][1], m[1][2],
              m[2][0], m[2][1], m[2][2];

    pose_ready_ = true;     
  }



  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_tcp_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_tcp_;

  bool pose_ready_;                
  Eigen::Vector3d tcp_rotation_;    
  Eigen::Matrix3d R_B_E_;  
  Eigen::Vector3d force_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetForce>());
  rclcpp::shutdown();
  return 0;
}