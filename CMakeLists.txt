cmake_minimum_required(VERSION 3.8)
project(luggage_av_msgs)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SendRoute.action"
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

include_directories(include)

# Install directories
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install the action files
install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()