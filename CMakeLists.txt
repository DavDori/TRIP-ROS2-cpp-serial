cmake_minimum_required(VERSION 3.5)
project(trip_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

add_library(trip_serial_interface include/${PROJECT_NAME}/trip_serial_interface.cpp)
add_library(motor include/${PROJECT_NAME}/motor.cpp)
add_library(encoder include/${PROJECT_NAME}/encoder.cpp)
add_library(differential_drive_model include/${PROJECT_NAME}/differential_drive_model.cpp)

add_executable(trip_base src/trip_base.cpp)
add_executable(trip_unicycle src/trip_unicycle.cpp)


target_link_libraries(trip_base 
  encoder
  motor
  trip_serial_interface)
target_link_libraries(trip_unicycle 
  encoder
  differential_drive_model
  motor
  trip_serial_interface)

target_include_directories(trip_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(trip_unicycle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(trip_base 
  rclcpp 
  sensor_msgs 
  std_msgs
  serial)
ament_target_dependencies(trip_unicycle 
  rclcpp 
  geometry_msgs
  sensor_msgs
  serial)

install(TARGETS trip_base trip_unicycle
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test 
    test/main.cpp
    test/encoder_unittest.cpp
    test/motor_unittest.cpp
    test/differential_drive_model_unittest.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test 
    rclcpp 
    geometry_msgs 
    sensor_msgs
    std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test trip_serial_interface)
  target_link_libraries(${PROJECT_NAME}_test encoder)
  target_link_libraries(${PROJECT_NAME}_test motor)
  target_link_libraries(${PROJECT_NAME}_test differential_drive_model)
endif()


ament_package()
