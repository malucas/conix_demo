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

using namespace std::chrono_literals;

class HighStateSim: public rclcpp::Node
{
    public:
        HighStateSim()
        : Node("high_state_sim"), count_(0)
        {
            // Declare and acquire frames
            parent_frame_ = this->declare_parameter("parent_frame", "world");
            child_frame_ = this->declare_parameter("child_frame", "base");

            timer_ = this->create_wall_timer(
                100ms, std::bind(&HighStateSim::timer_callback, this));

            publisher_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);

            tf_broadcaster_ = 
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            RCLCPP_INFO(this->get_logger(), "Initialized high state publisher.");
        }

    private:
        void timer_callback()
        {
            auto time = this->get_clock()->now();

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = time;
            t.header.frame_id = parent_frame_;
            t.child_frame_id = child_frame_;

            t.transform.translation.x = Xpos;
            t.transform.translation.y = Ypos;
            t.transform.translation.z = 1.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);

            ros2_unitree_legged_msgs::msg::HighState high_state;

            high_state.motor_state[0].q = pos_0;
            high_state.motor_state[1].q = pos_1;
            high_state.motor_state[2].q = pos_2;
            high_state.motor_state[3].q = pos_3;
            high_state.motor_state[4].q = pos_4;
            high_state.motor_state[5].q = pos_5;
            high_state.motor_state[6].q = pos_6;
            high_state.motor_state[7].q = pos_7;
            high_state.motor_state[8].q = pos_8;
            high_state.motor_state[9].q = pos_9;
            high_state.motor_state[10].q = pos_10;
            high_state.motor_state[11].q = pos_11;

            publisher_->publish(high_state);

            pos_0 = sin(count);
            pos_1 = sin(count);
            pos_2 = sin(count);
            pos_3 = 0.0;
            pos_4 = 0.0;
            pos_5 = 0.0;
            pos_6 = 0.0;
            pos_7 = 0.0;
            pos_8 = 0.0;
            pos_9 = sin(count);
            pos_10 = sin(count);
            pos_11 = sin(count);

            Xpos = 2.0 * cos(count);
            Ypos = 2.0 * sin(count);

            count += 0.1;
        }

        double pos_0 = 0.0;
        double pos_1 = 0.0;
        double pos_2 = 0.0;
        double pos_3 = 0.0;
        double pos_4 = 0.0;
        double pos_5 = 0.0;
        double pos_6 = 0.0;
        double pos_7 = 0.0;
        double pos_8 = 0.0;
        double pos_9 = 0.0;
        double pos_10 = 0.0;
        double pos_11 = 0.0;

        double Xpos = 1.0;
        double Ypos = 0.0;

        double count = 0.0;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr publisher_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        size_t count_;
        std::string parent_frame_;
        std::string child_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HighStateSim>());
  rclcpp::shutdown();
  return 0;
}