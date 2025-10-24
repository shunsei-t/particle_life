cmake_minimum_required(VERSION 3.5)
project(particle_life)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(particle_life_node src/particle_life_node.cpp)
ament_target_dependencies(particle_life_node rclcpp sensor_msgs)

install(TARGETS
  particle_life_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
