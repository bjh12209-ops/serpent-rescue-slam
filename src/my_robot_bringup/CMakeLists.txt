cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# This package installs launch/config assets only. Runtime dependencies are
# declared in package.xml and do not need CMake discovery here.
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_slam_telemetry test/test_slam_telemetry.py)
  ament_add_pytest_test(test_person_pose_detector test/test_person_pose_detector.py)
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  scripts/mission_control_gateway.py
  scripts/person_pose_detector.py
  scripts/slam_health_monitor.py
  scripts/slam_telemetry.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../apps/mission_control_ui/
  DESTINATION share/${PROJECT_NAME}/ui
)

ament_package()
