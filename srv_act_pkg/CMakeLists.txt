cmake_minimum_required(VERSION 3.8)
project(srv_act_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckNumber.srv"
  "msg/Numbers.msg"
  DEPENDENCIES 
  ADD_LINTER_TESTS
)

# Set support for using custom interfaces in C++ code
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


# create executables 
add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp) 
target_link_libraries(service_server "${cpp_typesupport_target}")

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp) 
target_link_libraries(service_client "${cpp_typesupport_target}")

add_executable(message_publisher src/message_publisher.cpp)
ament_target_dependencies(message_publisher rclcpp std_msgs) 
target_link_libraries(message_publisher "${cpp_typesupport_target}")

install(TARGETS
  service_server
  service_client
  message_publisher
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()