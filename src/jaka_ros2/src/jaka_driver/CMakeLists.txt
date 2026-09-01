cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
# find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Optionally find additional dependencies
# find_package(Boost REQUIRED)

# Declare ROS 2 message and service dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare executable targets
add_executable(jaka_driver
  src/jaka_driver.cpp
  src/phase5_joint_trajectory.cpp
)
add_executable(sdk_test src/sdk_test.cpp)
add_executable(frame_config_readonly src/frame_config_readonly.cpp)
add_executable(client src/client.cpp)
add_executable(servoj_demo src/servoj_demo.cpp)

if(BUILD_TESTING)
  enable_testing()
  add_executable(phase5_joint_trajectory_test
    test/phase5_joint_trajectory_test.cpp
    src/phase5_joint_trajectory.cpp
  )
  target_include_directories(phase5_joint_trajectory_test PUBLIC include)
  add_test(NAME phase5_joint_trajectory_test COMMAND phase5_joint_trajectory_test)
endif()

# Link libraries
ament_target_dependencies(jaka_driver
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
  control_msgs
)
ament_target_dependencies(sdk_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
)
ament_target_dependencies(frame_config_readonly
  rclcpp
)
ament_target_dependencies(client
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
  control_msgs
)
ament_target_dependencies(servoj_demo
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  # moveit_msgs
  action_msgs
)

# For including directories in ROS 2
target_include_directories(jaka_driver
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(client
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(sdk_test
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(frame_config_readonly
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(servoj_demo
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)



# JAKA SDK runtime library.  The main workspace uses colcon symlink-install,
# so installed executables may actually execute from <workspace>/build/<pkg>.
# Keep a build-tree runtime copy one directory above each package build dir and
# use an ORIGIN-relative RPATH in both build and install layouts.
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(JAKA_SDK_SOURCE_LIBRARY ${LIBRARY_DIR}/libjakaAPI.so)
set(JAKA_SDK_BUILD_RUNTIME_DIR ${CMAKE_BINARY_DIR}/..)
set(JAKA_SDK_BUILD_LIBRARY ${JAKA_SDK_BUILD_RUNTIME_DIR}/libjakaAPI.so)
file(COPY ${JAKA_SDK_SOURCE_LIBRARY} DESTINATION ${JAKA_SDK_BUILD_RUNTIME_DIR})

foreach(jaka_sdk_target jaka_driver sdk_test frame_config_readonly client servoj_demo)
  target_link_libraries(${jaka_sdk_target} ${JAKA_SDK_BUILD_LIBRARY})
  set_target_properties(${jaka_sdk_target} PROPERTIES
    BUILD_RPATH "$ORIGIN/.."
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH "$ORIGIN/.."
  )
endforeach()

# Install the JAKA SDK runtime library beside the package executables' parent
# lib directory so the relative INSTALL_RPATH above remains relocatable.
install(FILES ${LIBRARY_DIR}/libjakaAPI.so
  DESTINATION lib
)

# Install executables
install(TARGETS
  jaka_driver
  sdk_test
  frame_config_readonly
  client
  servoj_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
