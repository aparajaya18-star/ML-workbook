#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ChatSubscriber : public rclcpp::Node
{
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
public:
    ChatSubscriber() : Node("chat_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chat",
            10,
            std::bind(&ChatSubscriber::callback, this, std::placeholders::_1)
        );
    }
private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    }
};

int main(int argc, char *argv[])
{
    // creates node
    rclcpp::init(argc, argv);
    // event loop to keep the node running and processing incoming messages
    rclcpp::spin(std::make_shared<ChatSubscriber>());
    // kills the node and cleans up resources
    rclcpp::shutdown();
    return 0;
}