cmake_minimum_required(VERSION 3.5)
project(sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(FILES map.pgm map.yaml
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/target_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
