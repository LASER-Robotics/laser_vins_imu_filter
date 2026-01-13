cmake_minimum_required(VERSION 3.8)
project(laser_vins_imu_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_uav_lib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  laser_uav_lib
)

find_library(laser_uav_lib laser_uav_lib)

include_directories(
  include
  ${laser_uav_lib_INCLUDE_DIRS}     
)

# Create the executable
set(executable_name vins_imu_filter)

add_executable(${executable_name}
  src/vins_imu_filter.cpp
)

# Create the component library
set(library_name vins_imu_filter_node)
add_library(${library_name} SHARED
  src/vins_imu_filter_node.cpp
)

# Link dependencies to both executable and library
ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# Link the library to the executable
target_link_libraries(${executable_name}
  ${library_name}
)

# Register the component
rclcpp_components_register_nodes(${library_name} "vins_imu_filter::VinsImuFilter")

install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
