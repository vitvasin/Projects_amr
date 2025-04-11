cmake_minimum_required(VERSION 3.8)
project(hardware_interface_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(led_interface src/led_interface.cpp)
add_executable(ultrasonic_interface src/ultrasonic_interface.cpp)
add_executable(cmd_vel_interface src/cmd_vel_interface.cpp)
add_executable(bidirectional_interface src/bidirectional_interface.cpp)


# Link against LibSerial
find_library(LIBSERIAL_LIB libserial.so REQUIRED)
if(NOT LIBSERIAL_LIB)
  message(FATAL_ERROR "LibSerial library not found!")
endif()

target_include_directories(led_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(ultrasonic_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(cmd_vel_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(bidirectional_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(led_interface ${LIBSERIAL_LIB})
target_link_libraries(ultrasonic_interface ${LIBSERIAL_LIB})
target_link_libraries(cmd_vel_interface ${LIBSERIAL_LIB})
target_link_libraries(bidirectional_interface ${LIBSERIAL_LIB})

# Add ament dependencies
ament_target_dependencies(led_interface rclcpp std_msgs)
ament_target_dependencies(ultrasonic_interface rclcpp std_msgs)
ament_target_dependencies(cmd_vel_interface rclcpp std_msgs geometry_msgs)
ament_target_dependencies(bidirectional_interface rclcpp std_msgs geometry_msgs)

# Install targets
install(TARGETS
  led_interface
  ultrasonic_interface
  cmd_vel_interface
  bidirectional_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
