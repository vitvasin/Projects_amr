cmake_minimum_required(VERSION 3.8)
project(hardware_interface_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(led_interface src/led_interface.cpp)

# Link against LibSerial
find_library(LIBSERIAL_LIB libserial.so REQUIRED)
if(NOT LIBSERIAL_LIB)
  message(FATAL_ERROR "LibSerial library not found!")
endif()

target_include_directories(led_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(led_interface
  ${LIBSERIAL_LIB}
)

ament_target_dependencies(led_interface
  rclcpp
  std_msgs
)

install(TARGETS
  led_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
