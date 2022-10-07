#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

using std::placeholders::_1;

class StateToJointConv: public rclcpp::Node
{
    public:
        StateToJointConv()
        : Node("state_to_joint_converter")
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            subscription_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighState>(
                "high_state", 10, std::bind(&StateToJointConv::high_state_callback, this, _1));
        }

    private:
        void high_state_callback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr high_state) const
        {
            // auto time = this->get_clock().now();

            sensor_msgs::msg::JointState joint_state;
            // joint_state.header.stamp = time;

            joint_state.name.push_back("FR_hip_joint");
            joint_state.name.push_back("FR_thigh_joint");
            joint_state.name.push_back("FR_calf_joint");
            joint_state.name.push_back("FL_hip_joint");
            joint_state.name.push_back("FL_thigh_joint");
            joint_state.name.push_back("FL_calf_joint");
            joint_state.name.push_back("RR_hip_joint");
            joint_state.name.push_back("RR_thigh_joint");
            joint_state.name.push_back("RR_calf_joint");
            joint_state.name.push_back("RL_hip_joint"); 
            joint_state.name.push_back("RL_thigh_joint");
            joint_state.name.push_back("RL_calf_joint");
        
            joint_state.position.push_back(double(high_state->motor_state[0].q));
            joint_state.position.push_back(double(high_state->motor_state[1].q));
            joint_state.position.push_back(double(high_state->motor_state[2].q));
            joint_state.position.push_back(double(high_state->motor_state[3].q));
            joint_state.position.push_back(double(high_state->motor_state[4].q));
            joint_state.position.push_back(double(high_state->motor_state[5].q));
            joint_state.position.push_back(double(high_state->motor_state[6].q));
            joint_state.position.push_back(double(high_state->motor_state[7].q));
            joint_state.position.push_back(double(high_state->motor_state[8].q));
            joint_state.position.push_back(double(high_state->motor_state[9].q));
            joint_state.position.push_back(double(high_state->motor_state[10].q)); 
            joint_state.position.push_back(double(high_state->motor_state[11].q));

            publisher_->publish(joint_state);
        }

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateToJointConv>());
  rclcpp::shutdown();
  return 0;
}