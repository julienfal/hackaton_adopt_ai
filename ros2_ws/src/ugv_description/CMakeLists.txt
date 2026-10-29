cmake_minimum_required(VERSION 3.10...3.30)
project(ugv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
