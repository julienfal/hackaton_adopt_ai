cmake_minimum_required(VERSION 3.10...3.30)
project(ugv_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
