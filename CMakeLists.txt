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
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces for the action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SendRoute.action"
  DEPENDENCIES
  geometry_msgs
  nav2_msgs
  std_msgs
)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

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


set(DEPENDENCIES
  # these must all be the names of cpp files in the src/ directory
  cancel_route
  send_route_client
  send_route_server
)


foreach(Dependency IN ITEMS ${DEPENDENCIES})

  # Build the client executable
  add_executable(${Dependency} src/${Dependency}.cpp)
  ament_target_dependencies(${Dependency}
    rclcpp
    rclcpp_action
    geometry_msgs
    std_msgs
    rosidl_default_runtime
  )

  # Link the generated action code
  target_link_libraries(${Dependency}
    ${PROJECT_NAME}__rosidl_typesupport_cpp
  )

  # Install the executable files
  install(TARGETS
    ${Dependency}
    DESTINATION lib/${PROJECT_NAME}
  )

endforeach()



ament_package()