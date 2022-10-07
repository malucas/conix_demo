#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

class CmdInterface : public rclcpp::Node
{
    public:
        CmdInterface()
        : Node("command_interface"), count_(0)
        {
            std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
            std::cin.ignore();

            timer_ = this->create_wall_timer(200ms, std::bind(&CmdInterface::timer_callback, this));
            publisher_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);
        }
    
    private:
        void timer_callback()
        {
            ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
            
            high_cmd_ros.head[0] = 0xFE;
            high_cmd_ros.head[1] = 0xEF;
            high_cmd_ros.level_flag = HIGHLEVEL;
            high_cmd_ros.mode = 0;
            high_cmd_ros.gait_type = 0;
            high_cmd_ros.speed_level = 0;
            high_cmd_ros.foot_raise_height = 0;
            high_cmd_ros.body_height = 0;
            high_cmd_ros.euler[0] = 0;
            high_cmd_ros.euler[1] = 0;
            high_cmd_ros.euler[2] = 0;
            high_cmd_ros.velocity[0] = 0.0f;
            high_cmd_ros.velocity[1] = 0.0f;
            high_cmd_ros.yaw_speed = 0.0f;
            high_cmd_ros.reserve = 0;

            publisher_->publish(high_cmd_ros);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr publisher_;
        uint count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdInterface>());
    rclcpp::shutdown();
    return 0;
}