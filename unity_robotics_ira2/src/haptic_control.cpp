#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include <thread>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <unity_robotics_ira2/msg/haptic_info.hpp>

using namespace ur_rtde;

class HapticControl : public rclcpp::Node
{
public:
  HapticControl()
  : Node("haptic_control"), rtde_control("192.168.1.101"), rtde_receive("192.168.1.101"), gripper("192.168.1.101", 63352, true),
    velocity(0.1), acceleration(0.1), dt(1.0/125), lookahead_time(0.1), gain(500),
    reference_position_({0.0, 0.0, 0.0}), first_position(false), init_force_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
    reference_orientation_({0.0, 0.0, 0.0}), current_position_({0.0, 0.0, 0.0}), current_orientation_({0.0, 0.0, 0.0}),
    ref_ori_robot_({0.0, 0.0, 0.0}), ref_pos_robot_({0.0, 0.0, 0.0}), gripper_closed(false)
  {
    // Souscription au topic des données haptiques
    haptic_sub_ = this->create_subscription<unity_robotics_ira2::msg::HapticInfo>(
      "/haptic_info", 10,
      std::bind(&HapticControl::haptic_callback, this, std::placeholders::_1));

    force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/tcp_force", 10);


    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(8),
      std::bind(&HapticControl::get_force, this));

    std::vector<double> init_pose = {-2.3, -1.81, -1.57, -1.27, 1.64, 0.1};
    rtde_control.moveJ(init_pose, velocity, acceleration);
    gripper.connect();
    gripper.activate();
    gripper.open(1.0, 0.0, RobotiqGripper::WAIT_FINISHED);
    init_force_ = rtde_receive.getActualTCPForce();

  }

  ~HapticControl()
  {
    // Save latencies and errors to CSV files
    std::ofstream out("latencies.csv");
    for (double lat : latencies_) {
      out << lat << "\n";
    }
    out.close();

    std::ofstream out2("pose_errors.csv");
    for (auto err : pose_errors_) {
      out2 << err.first << "," << err.second << "\n";
    }
    out2.close();
    gripper.disconnect();
    rtde_control.servoStop();
    rtde_control.stopScript();
  }

private:

  bool in_workspace(const std::array<double,6>& pose)
  {
    //const double x_min = -0.4, x_max = -0.08;
    //const double y_min = -0.34, y_max = -0.17;
    const double z_min =0.012 , z_max = 0.33;
    //const double rx_min = 1.1, rx_max = 1.3;
    //const double ry_min = 2.65, ry_max = 3.1;
    //const double rz_min = -0.16, rz_max = -0.01;

    if (  pose[2] < z_min || pose[2] > z_max) {
      return false;
    }

    return true;
  }

  void get_force()
  {
    // get robot force
    std::vector<double> tcp_force = rtde_receive.getActualTCPForce();
    if (tcp_force.size() != 6) {
      RCLCPP_ERROR(this->get_logger(), "Erreur de récupération de la force TCP.");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      return;
    }
    //RCLCPP_INFO(this->get_logger(), "Force TCP: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  //tcp_force[0], tcp_force[1], tcp_force[2],
                  //tcp_force[3], tcp_force[4], tcp_force[5]);

    // publish force
    geometry_msgs::msg::WrenchStamped force_msg;
    force_msg.header.stamp = this->now();
    force_msg.header.frame_id = "base_link";
    force_msg.wrench.force.x = tcp_force[0] - init_force_[0];
    force_msg.wrench.force.y = tcp_force[1] - init_force_[1];
    force_msg.wrench.force.z = tcp_force[2] - init_force_[2];
    force_msg.wrench.torque.x = tcp_force[3] - init_force_[3];
    force_msg.wrench.torque.y = tcp_force[4] - init_force_[4];
    force_msg.wrench.torque.z = tcp_force[5] - init_force_[5];
    force_pub_->publish(force_msg);
  }

  void haptic_callback(const unity_robotics_ira2::msg::HapticInfo msg)
  {
    //rclcpp::Time msg_time = msg->header.stamp;
    rclcpp::Time msg_time = this->now();
    int btn;
    btn = msg.button;
    
    if (btn ==0) {
        first_position = true;
    }

    else if (btn == 2) { // grip
          if (!gripper_closed) {
            gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
            gripper.setUnit(RobotiqGripper::SPEED, RobotiqGripper::UNIT_DEVICE);
            gripper.setUnit(RobotiqGripper::FORCE, RobotiqGripper::UNIT_DEVICE);
            gripper.move(255, 5, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            while (RobotiqGripper::MOVING == gripper.objectDetectionStatus())
            {
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            gripper_closed = true;
          } else {
            gripper.open(1.0, 0.0, RobotiqGripper::WAIT_FINISHED);
            gripper_closed = false;
          }
        }
    else if (btn == 1) { //button move

        current_position_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_orientation_ = {roll, pitch, yaw}; 
          // get robot pose 
        std::vector<double> tcp_pose = rtde_receive.getActualTCPPose();
        if (tcp_pose.size() != 6) {
          RCLCPP_ERROR(this->get_logger(), "Erreur de récupération de la pose TCP.");
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          return;
        }

        // copy data under lock
        std::array<double, 3> curr_pos, curr_ori;
        bool have_target = false;
          
        curr_pos = {current_position_[0], current_position_[1], current_position_[2]};
        curr_ori = {current_orientation_[0], current_orientation_[1], current_orientation_[2]};
          

        if (first_position == true) {
              reference_position_ = {curr_pos[0], curr_pos[1], curr_pos[2]};
              reference_orientation_ = {curr_ori[0], curr_ori[1], curr_ori[2]};
              ref_pos_robot_ = {tcp_pose[0], tcp_pose[1], tcp_pose[2]};
              ref_ori_robot_ = {tcp_pose[3], tcp_pose[4], tcp_pose[5]};
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
              first_position = false;
              return;
              
        }

        // compute deltas and target
        std::array<double, 3> ref_pos, ref_ori;
            
        //std::lock_guard<std::mutex> lock(mutex_);
        ref_pos = {reference_position_[0], reference_position_[1], reference_position_[2]};
        ref_ori = {reference_orientation_[0], reference_orientation_[1], reference_orientation_[2]};

        
        std::array<double,3> delta_position = {curr_pos[0] - ref_pos[0],
                                                  curr_pos[1] - ref_pos[1],
                                                  curr_pos[2] - ref_pos[2]};
                                            
        std::array<double,3> delta_orientation = {curr_ori[0] - ref_ori[0],
                                                    curr_ori[1] - ref_ori[1],
                                                    curr_ori[2] - ref_ori[2]};

        std::array<double,6> target;

        // calculate target with scaling on orientation
        target = {delta_position[0] + ref_pos_robot_[0],
                  delta_position[1] + ref_pos_robot_[1],
                  delta_position[2] + ref_pos_robot_[2],
                  delta_orientation[0]*0.04 + ref_ori_robot_[0],
                  delta_orientation[1]*0.04 + ref_ori_robot_[1],
                  delta_orientation[2]*0.04 + ref_ori_robot_[2]};

        //RCLCPP_INFO(this->get_logger(), "delta pos: [%.2f, %.2f, %.2f]", delta_position[0], delta_position[1], delta_position[2]);

          
        // safety check
        if (!in_workspace(target)) {
          RCLCPP_WARN(this->get_logger(), "Consigne hors de l'espace de travail. Arrêt du mouvement.");
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          return;
        }

        bool kinematics_inv = rtde_control.getInverseKinematicsHasSolution({target[0], target[1], target[2], target[3], target[4], target[5]});

        if (!kinematics_inv) {
          RCLCPP_ERROR(this->get_logger(), "Erreur de cinématique inverse. Arrêt du mouvement.");
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          return;
        }

        // move robot
        auto t_start = rtde_control.initPeriod();
        rtde_control.servoL({target[0], target[1], target[2], target[3], target[4], target[5]},
                            velocity, acceleration, dt, lookahead_time, gain);
        rtde_control.waitPeriod(t_start);

        rclcpp::Time now = this->now();
        // latency calculation
        double latency = (now - msg_time).seconds();
        latencies_.push_back(latency);
        if (latencies_.size() > 1000) {
          latencies_.erase(latencies_.begin());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        std::vector<double> new_tcp_pose = rtde_receive.getActualTCPPose();

        double position_error = std::sqrt(std::pow(new_tcp_pose[0] - target[0], 2) +
                                          std::pow(new_tcp_pose[1] - target[1], 2) +
                                          std::pow(new_tcp_pose[2] - target[2], 2));

        double orientation_error = std::sqrt(std::pow(new_tcp_pose[3] - target[3], 2) +
                                              std::pow(new_tcp_pose[4] - target[4], 2) +
                                              std::pow(new_tcp_pose[5] - target[5], 2));
            
        pose_errors_.push_back({position_error, orientation_error});
        if (pose_errors_.size() > 1000) {
          pose_errors_.erase(pose_errors_.begin());
        }
    }
  }

  // Paramètres
  RTDEControlInterface rtde_control;
  RTDEReceiveInterface rtde_receive;
  RobotiqGripper gripper;
  bool gripper_closed;
  bool first_position;
  // Positions
  std::vector<double> reference_position_;
  std::vector<double> reference_orientation_;
  std::vector<double> current_position_;
  std::vector<double> current_orientation_;
  std::vector<double> ref_pos_robot_;
  std::vector<double> ref_ori_robot_;
  std::vector<double> init_force_;
  std::vector<double> latencies_;
  std::vector<std::pair<double, double>> pose_errors_;
  // Paramètres de contrôle
  double velocity, acceleration, dt, lookahead_time, gain;


  // ROS
  rclcpp::Subscription<unity_robotics_ira2::msg::HapticInfo>::SharedPtr haptic_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HapticControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
