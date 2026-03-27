#include <thread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ChatPublisher : public rclcpp::Node
{
    // Publisher object to send messages to the "chat" topic
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::thread input_thread_;

public:
    ChatPublisher() : Node("chat_publisher")
    {
        //Initializing the publisher 
        publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);
        // Start the input loop in a separate thread
        input_thread_ = std::thread(&ChatPublisher::inputLoop, this);
        input_thread_.detach(); 
        // Detach the thread to allow it to run independently   
    }

private:
    void sendMessage(const std::string &text)
    {
        auto msg = std_msgs::msg::String();
        msg.data = text;

        RCLCPP_INFO(this->get_logger(), "Sending message: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    void inputLoop()
    {
        std::string input;

        while (rclcpp::ok())
        {
            std::cout << "You: ";
            std::getline(std::cin, input);

            if (input == "exit")
            {
                rclcpp::shutdown();
                break;
            }

            sendMessage(input);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<ChatPublisher>()); // Keep the node running to allow message publishing
    rclcpp::shutdown(); // Cleanly shutdown the ROS 2 system
    return 0;
}