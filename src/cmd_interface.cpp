#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "unitree_legged_sdk/comm.h"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;
using std::placeholders::_1;

class CmdInterface : public rclcpp::Node
{
    public:
        CmdInterface()
        : Node("command_interface"), count_ {0}
        {
            std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
            std::cin.ignore();

            timer_ = this->create_wall_timer(200ms, std::bind(&CmdInterface::timer_callback, this));
            high_cmd_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "cmd_vel_sub", 1, std::bind(&CmdInterface::cmd_vel_callback, this, _1));
        }
    
    private:
        void timer_callback()
        {
            high_cmd_ros_.head[0] = 0xFE;
            high_cmd_ros_.head[1] = 0xEF;
            high_cmd_ros_.level_flag = HIGHLEVEL;
            high_cmd_ros_.mode = 0;
            high_cmd_ros_.gait_type = 0;
            high_cmd_ros_.speed_level = 0;
            high_cmd_ros_.foot_raise_height = 0;
            high_cmd_ros_.body_height = 0;
            high_cmd_ros_.euler[0] = 0;
            high_cmd_ros_.euler[1] = 0;
            high_cmd_ros_.euler[2] = 0;
            high_cmd_ros_.velocity[0] = 0.0f;
            high_cmd_ros_.velocity[1] = 0.0f;
            high_cmd_ros_.yaw_speed = 0.0f;
            high_cmd_ros_.reserve = 0;

            high_cmd_pub_->publish(high_cmd_ros_);
        }

        void cmd_vel_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            return;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pos_sub_;

        ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros_;
        uint count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdInterface>());
    rclcpp::shutdown();
    return 0;
}