cmake_minimum_required(VERSION 3.5)
project(mvcc_camera_ros2)
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 

execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH_OUTPUT)
if(ARCH_OUTPUT MATCHES "x86_64|amd64")
    set(HOST_PLATFORM "64")
elseif(ARCH_OUTPUT MATCHES "i686")
    set(HOST_PLATFORM "32")
elseif(ARCH_OUTPUT MATCHES "aarch64")
    set(HOST_PLATFORM "aarch64")
elseif(ARCH_OUTPUT MATCHES "armhf")
    set(HOST_PLATFORM "armhf")
elseif(ARCH_OUTPUT MATCHES "arm-none")
    set(HOST_PLATFORM "arm-none")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)  # If using OpenCV for image handling
find_package(OpenCV REQUIRED)
find_package(mvcc_camera_ros2_interface REQUIRED)
find_package(message_filters REQUIRED)
find_package(fmt REQUIRED)
find_package(rcpputils REQUIRED)



include_directories($ENV{MVCAM_COMMON_RUNENV}/../include)
find_library(MVCC_SDK_LIB MvCameraControl PATHS $ENV{MVCAM_COMMON_RUNENV}/${HOST_PLATFORM})

add_executable(hik_camera_image_pub src/create_camera_image_pub.cpp src/hik_camera.cpp src/camera_parameter_service_manager.cpp)
target_link_libraries(hik_camera_image_pub ${MVCC_SDK_LIB})
ament_target_dependencies(hik_camera_image_pub rclcpp sensor_msgs std_msgs cv_bridge OpenCV mvcc_camera_ros2_interface) 
 
 
add_executable(hik_camera_image_sub src/create_camera_image_sub.cpp)
target_link_libraries(hik_camera_image_sub ${MVCC_SDK_LIB} fmt)
ament_target_dependencies(hik_camera_image_sub rclcpp sensor_msgs std_msgs cv_bridge OpenCV message_filters rcpputils mvcc_camera_ros2_interface)

add_executable(hik_camera_params_test src/camera_params_test.cpp)
ament_target_dependencies(hik_camera_params_test rclcpp mvcc_camera_ros2_interface)


install(TARGETS 
  hik_camera_image_pub
  hik_camera_image_sub
  hik_camera_params_test
  DESTINATION lib/${PROJECT_NAME})
 
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()
