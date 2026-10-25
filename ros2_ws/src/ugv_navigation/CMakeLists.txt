cmake_minimum_required(VERSION 3.8)
project(ugv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)




add_executable(ugv_navigation)

target_sources(ugv_navigation PRIVATE
  src/pure_pursuit_node.cpp
  src/geometry.cpp
  )

target_include_directories(ugv_navigation PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ugv_navigation 
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  )


install(TARGETS ugv_navigation
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry 
  test/test_geometry.cpp
  src/geometry.cpp
  )
  target_include_directories(test_geometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
