cmake_minimum_required(VERSION 3.8)
project(nbv_mapping)

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
find_package(rclcpp_lifecycle REQUIRED)

set(LIFECYCLE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  vdb_mapping
  vdb_mapping_ros2
)

include_directories(include)
add_library(${PROJECT_NAME} SHARED src/vdb_lifecycle.cpp)
ament_target_dependencies(${PROJECT_NAME} ${LIFECYCLE_DEPENDS})


add_executable(${PROJECT_NAME}_node src/mapping_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${LIFECYCLE_DEPENDS} )

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
