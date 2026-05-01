cmake_minimum_required(VERSION 3.8)
project(car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(CAR_CONTROL_ENABLE_NATIVE_OPTIMIZATION "Enable native CPU tuning for car_control" ON)

if(POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4 CACHE PATH "OpenCV 4.8 config path" FORCE)
find_package(OpenCV REQUIRED CONFIG)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
        /usr/local/include/apriltag
)

link_directories(
        /usr/local/lib
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

add_executable(car_control src/car_control_node.cpp src/tag_detector.cpp)
ament_target_dependencies(car_control rclcpp sensor_msgs OpenCV cv_bridge image_transport)

target_link_libraries(car_control
        ${OpenCV_LIBS}
        apriltag
)

if(CAR_CONTROL_ENABLE_NATIVE_OPTIMIZATION AND NOT CMAKE_CROSSCOMPILING AND
   CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(car_control PRIVATE -march=native -mtune=native)
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT car_control_ipo_supported OUTPUT car_control_ipo_error)
if(car_control_ipo_supported)
  set_property(TARGET car_control PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

install(TARGETS
  car_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")

ament_package()
