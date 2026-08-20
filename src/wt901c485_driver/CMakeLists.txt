cmake_minimum_required(VERSION 3.8)
project(wt901c485_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wt901c485_node src/wt901c485_node.cpp)
target_compile_features(wt901c485_node PUBLIC cxx_std_17)
ament_target_dependencies(wt901c485_node geometry_msgs rclcpp sensor_msgs)

install(TARGETS wt901c485_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
