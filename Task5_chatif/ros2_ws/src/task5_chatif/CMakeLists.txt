cmake_minimum_required(VERSION 3.8)
project(task5_chatif)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Publisher executable
add_executable(chat_publisher src/chat_publisher.cpp)
ament_target_dependencies(chat_publisher rclcpp std_msgs)

# Subscriber executable
add_executable(chat_subscriber src/chat_subscriber.cpp)
ament_target_dependencies(chat_subscriber rclcpp std_msgs)

install(TARGETS
  chat_publisher
  chat_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()