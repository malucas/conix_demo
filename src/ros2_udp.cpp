#include <memory>
#include <sstream>
#include <string>
#include <cmath>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

class Ros2Udp : public rclcpp::Node 
{
    public:
        Ros2Udp()
        : Node("ros2_udp"),
          high_udp_(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)),
          high_count_ {0},
          high_cmd_ {0},
          high_state_ {0}
        {
            using std::placeholders::_1;

            pub_high_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
            sub_high_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
                "high_cmd", 1, std::bind(&Ros2Udp::high_cmd_callback, this, _1));

            high_udp_.InitCmdData(high_cmd_);

            RCLCPP_INFO(this->get_logger(), "Initialized udp node!");
        }
        
    private:
        void high_cmd_callback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received high command! ", high_count_);

            high_cmd_ = rosMsg2Cmd(msg);

            high_udp_.SetSend(high_cmd_);
            high_udp_.Send();

            ros2_unitree_legged_msgs::msg::HighState high_state_ros;

            high_udp_.Recv();
            high_udp_.GetRecv(high_state_);

            high_state_ros = state2rosMsg(high_state_);

            pub_high_->publish(high_state_ros);

            high_count_++;
        }

        rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high_;

        long high_count_;
        UDP high_udp_;
        HighCmd high_cmd_;
        HighState high_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2Udp>());
    rclcpp::shutdown();

    return 0;
}