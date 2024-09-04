cmake_minimum_required(VERSION 3.8)
project(potential_fields)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(apf_interfaces REQUIRED)

add_library(basic_apf_component SHARED src/basic_apf.cpp)
target_include_directories(basic_apf_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(basic_apf_component rclcpp visualization_msgs geometry_msgs sensor_msgs tf2_ros tf2 rclcpp_components apf_interfaces)

rclcpp_components_register_node(
  basic_apf_component
  PLUGIN "potential_fields::BasicAPF"
  EXECUTABLE basic_apf
)
add_executable(goal_tf_publisher src/goal_tf_publisher.cpp)
ament_target_dependencies(goal_tf_publisher rclcpp geometry_msgs tf2_ros)

ament_export_targets(export_basic_apf_component)
install(TARGETS basic_apf_component
  EXPORT export_basic_apf_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  goal_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
