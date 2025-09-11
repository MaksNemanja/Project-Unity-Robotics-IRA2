#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

class JointsSubscriber : public rclcpp::Node
{
public:
    JointsSubscriber()
    : Node("joints_subscriber"), position_({0.0, -1.57, 0.0, -1.57, 0.0, 0.0}),
      rtde_control("192.168.56.101"), rtde_receive("192.168.56.101"),
      velocity(0.1), acceleration(0.1), dt(1.0/500), lookahead_time(0.1), gain(200)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_pub", 10, std::bind(&JointsSubscriber::joints_callback, this, std::placeholders::_1));

        control_thread_ = std::thread(&JointsSubscriber::control_loop, this);

        
    }

private:
    void joints_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() != 6) return;

        for (size_t i = 0; i < 6; ++i)
            position_[i] = msg->position[i];
        position_[0] -= 1.57;
    }

    void control_loop()
    {
        while (rclcpp::ok() && running_)
        {
            auto t_start = rtde_control.initPeriod();
            rtde_control.servoJ(position_, velocity, acceleration, dt, lookahead_time, gain);
            rtde_control.waitPeriod(t_start);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::vector<double> position_; // Position des articulations
    RTDEControlInterface rtde_control;
    RTDEReceiveInterface rtde_receive;
    std::thread control_thread_;
    bool running_ = true;
    double velocity, acceleration, dt, lookahead_time, gain;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointsSubscriber>());
    rclcpp::shutdown();
    return 0;
}