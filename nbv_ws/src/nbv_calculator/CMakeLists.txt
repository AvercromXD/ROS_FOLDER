cmake_minimum_required(VERSION 3.8)
project(nbv_calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vdb_mapping_ros2 REQUIRED)
find_package(vdb_mapping REQUIRED)
find_package(nbv_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vdb_mapping_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nbv_mapping REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
  vdb_mapping_interfaces
  pcl_conversions
  vdb_mapping
  vdb_mapping_ros2
  nbv_interfaces
  geometry_msgs
  OpenCV
  cv_bridge
  image_geometry
  tf2_geometry_msgs
  lifecycle_msgs
  nbv_mapping
)


add_library(${PROJECT_NAME} SHARED src/nbv_action_server.cpp)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
add_executable(nbv_node
  src/nbv_node.cpp
)

add_executable(nbv_action_server src/nbv_action_server.cpp)
ament_target_dependencies(nbv_node rclcpp vdb_mapping_ros2 vdb_mapping nbv_mapping)
ament_target_dependencies(nbv_action_server rclcpp)


target_link_libraries(nbv_action_server
    ${PROJECT_NAME}
    )

target_include_directories(nbv_action_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


target_link_libraries(nbv_node
    ${PROJECT_NAME}
    )

target_include_directories(nbv_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  nbv_node
  nbv_action_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
