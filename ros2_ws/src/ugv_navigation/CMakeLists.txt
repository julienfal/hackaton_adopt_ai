cmake_minimum_required(VERSION 3.10...3.30)
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

# --- 1. Déclarer l'exécutable et ses sources ---
add_executable(ugv_navigation)

target_sources(ugv_navigation PRIVATE
  src/pure_pursuit_node.cpp
  src/geometry.cpp
  )

target_include_directories(ugv_navigation PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# --- 2. Lier les dépendances au format moderne (Standard Ubuntu 26.04 / ROS 2 Lyrical) ---
target_link_libraries(ugv_navigation PUBLIC
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# --- 3. Règles d'installation ---
install(TARGETS ugv_navigation
  DESTINATION lib/${PROJECT_NAME})

# --- 4. Section de Tests ---
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
  # Signature "plain" (sans PUBLIC) pour s'accorder avec ament_add_gtest
  target_link_libraries(test_geometry rclcpp::rclcpp) 
endif()

# --- 5. Clôture du package ---
ament_package()
