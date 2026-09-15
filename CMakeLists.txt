cmake_minimum_required(VERSION 3.16)
project(lidar_camera_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)

add_library(interpolation STATIC src/interpolation.cpp)
target_include_directories(interpolation PUBLIC src)
target_include_directories(interpolation SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_options(interpolation PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(interpolation PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)

add_executable(interpolated_node src/interpolated_lidar.cpp)
add_executable(lidar_camera_node src/lidar_camera_node.cpp)
foreach(target interpolated_node lidar_camera_node)
  ament_target_dependencies(${target} rclcpp rcl_interfaces sensor_msgs std_msgs cv_bridge message_filters pcl_conversions)
  target_include_directories(${target} SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${target} interpolation ${OpenCV_LIBRARIES})
endforeach()
foreach(target interpolation interpolated_node lidar_camera_node)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()

install(TARGETS interpolated_node lidar_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md VALIDATION.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(interpolation_test test/interpolation_test.cpp)
  target_link_libraries(interpolation_test interpolation)
  add_test(NAME interpolation_test COMMAND interpolation_test)

  find_package(ament_cmake_pytest REQUIRED)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  ament_add_pytest_test(ros_smoke_test test/ros_smoke_test.py
    TIMEOUT 60
    ENV "ROS_DOMAIN_ID=187" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  ament_add_pytest_test(startup_test test/startup_test.py
    TIMEOUT 90
    ENV "ROS_DOMAIN_ID=188" "ROS_LOCALHOST_ONLY=1"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
endif()
ament_package()
