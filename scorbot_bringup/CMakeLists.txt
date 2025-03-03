cmake_minimum_required(VERSION 3.8)
project(scorbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

install(
  DIRECTORY launch rviz world src
  DESTINATION share/${PROJECT_NAME}/
)

#Install Python modules
ament_python_install_package(${PROJECT_NAME})

#Install python scripts
install(PROGRAMS
  # Declare programs in src directory
  src/trajectory_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()