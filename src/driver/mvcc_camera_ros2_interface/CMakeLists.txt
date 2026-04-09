cmake_minimum_required(VERSION 3.8)
project(mvcc_camera_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
 "srv/ExecuteCommand.srv"
 "srv/GetEnumValue.srv"
 "srv/GetFloatValue.srv"
 "srv/GetIntegerValue.srv"
 "srv/GetStringValue.srv"
 "srv/GetBoolValue.srv"
 "srv/SetEnumValue.srv"
 "srv/SetFloatValue.srv"
 "srv/SetIntegerValue.srv"
 "srv/SetStringValue.srv"
 "srv/SetBoolValue.srv"
 "msg/FrameInfo.msg"
 "msg/ChunkDataContent.msg"
 DEPENDENCIES sensor_msgs
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
