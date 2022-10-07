#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node
{
    public:
        StatePublisher()
        : Node("state_publisher"), count_(0)
        {
            publisher_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("topic", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&StatePublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            auto message = ros2_unitree_legged_msgs::msg::HighState();
            RCLCPP_INFO(this->get_logger(), "Publishing:'%d'", count_);
            publisher_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher>());
  rclcpp::shutdown();
  return 0;
}