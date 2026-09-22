#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MyCustomNode : public rclcpp::Node
{
public:
    MyCustomNode() : Node("my_node_name"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello World");
        timer_ = this->create_wall_timer(
            1s, std::bind(&MyCustomNode::print_hello, this));
    }

private:
    void print_hello()
    {
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_++);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
