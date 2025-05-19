cmake_minimum_required(VERSION 3.8)
project(avoidance_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    PCL
    pcl_conversions
    tf2
    tf2_ros
    tf2_sensor_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


# Set dependecies
set(dependencies
  std_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  PCL
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# main node
add_executable(main
  src/main.cpp
  src/voxelgrid-cube.cpp
)
ament_target_dependencies(main ${dependencies})
target_link_directories(main PUBLIC ${PCL_LIBRARY_DIRS})

# test node
add_executable(test_vel
  src/test_vel.cpp
)
ament_target_dependencies(test_vel ${dependencies})
target_link_directories(test_vel PUBLIC ${PCL_LIBRARY_DIRS})

# side node
# add_executable(camera_side
#   src/camera_side.cpp
# )
# ament_target_dependencies(camera_side ${dependencies})
# target_link_directories(camera_side PUBLIC ${PCL_LIBRARY_DIRS})


install(TARGETS
  main
  test_vel
  # camera_side
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
