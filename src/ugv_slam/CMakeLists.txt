cmake_minimum_required(VERSION 3.8)
project(ugv_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/topic_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
