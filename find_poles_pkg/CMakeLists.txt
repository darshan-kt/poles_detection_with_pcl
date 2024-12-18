cmake_minimum_required(VERSION 3.8)
project(find_poles_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(pole_detection src/find_poles.cpp)
ament_target_dependencies(pole_detection rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pole_detection ${PCL_LIBRARIES})

add_executable(midpoint src/poles_midpoint.cpp)
ament_target_dependencies(midpoint rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(midpoint ${PCL_LIBRARIES})

install(TARGETS
  pole_detection
  midpoint
  DESTINATION lib/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

