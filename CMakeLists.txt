cmake_minimum_required(VERSION 3.8)
project(luggage_av_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()


# Add the action client executable
ament_auto_add_executable(wait_for_passenger_action_client 
  src/wait_for_passenger_action_client_node.cpp
  src/wait_for_passenger_action_client_class.cpp) # dependencies
# Install the executable
install(TARGETS wait_for_passenger_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Add the action server executable
ament_auto_add_executable(wait_for_passenger_action_server
  src/wait_for_passenger_action_server_node.cpp
  src/wait_for_passenger_action_server_class.cpp) # dependencies
# Install the executable
install(TARGETS wait_for_passenger_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launchfile
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 