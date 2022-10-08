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

class JointSim: public rclcpp::Node
{
    public:
        JointSim()
        : Node("joint_sim"), count_(0)
        {
            // Declare and acquire frames
            parent_frame_ = this->declare_parameter("parent_frame", "odom");
            child_frame_ = this->declare_parameter("child_frame", "base");

            timer_ = this->create_wall_timer(
                50ms, std::bind(&JointSim::timer_callback, this));

            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            tf_broadcaster_ = 
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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

            // tf_broadcaster_->sendTransform(t);

            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = time;

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
        
            joint_state.position.push_back(FR_hip_joint_);
            joint_state.position.push_back(FR_thigh_joint_);
            joint_state.position.push_back(FR_calf_joint_);
            joint_state.position.push_back(FL_hip_joint_);
            joint_state.position.push_back(FL_thigh_joint_);
            joint_state.position.push_back(FL_calf_joint_);
            joint_state.position.push_back(RR_hip_joint_);
            joint_state.position.push_back(RR_thigh_joint_);
            joint_state.position.push_back(RR_calf_joint_);
            joint_state.position.push_back(RL_hip_joint_);
            joint_state.position.push_back(RL_thigh_joint_); 
            joint_state.position.push_back(RL_calf_joint_);

            RCLCPP_INFO(this->get_logger(), "Publishing joint state.");
            publisher_->publish(joint_state);

            FR_hip_joint_ = sin(count);
            FR_thigh_joint_ = sin(count);
            FR_calf_joint_ = sin(count);
            FL_hip_joint_ = 0.0;
            FL_thigh_joint_ = 0.0;
            FL_calf_joint_ = 0.0;
            RR_hip_joint_ = 0.0;
            RR_thigh_joint_ = 0.0;
            RR_calf_joint_ = 0.0;
            RL_hip_joint_ = sin(count);
            RL_thigh_joint_ = sin(count);
            RL_calf_joint_ = sin(count);

            Xpos = 2.0 * cos(count);
            Ypos = 2.0 * sin(count);

            count += 0.1;
        }

        double FR_hip_joint_ = 0.0;
        double FR_thigh_joint_ = 0.0;
        double FR_calf_joint_ = 0.0;
        double FL_hip_joint_ = 0.0;
        double FL_thigh_joint_ = 0.0;
        double FL_calf_joint_ = 0.0;
        double RR_hip_joint_ = 0.0;
        double RR_thigh_joint_ = 0.0;
        double RR_calf_joint_ = 0.0;
        double RL_hip_joint_ = 0.0;
        double RL_thigh_joint_ = 0.0;
        double RL_calf_joint_ = 0.0;

        double Xpos = 1.0;
        double Ypos = 0.0;

        double count = 0.0;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        size_t count_;
        std::string parent_frame_;
        std::string child_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointSim>());
  rclcpp::shutdown();
  return 0;
}