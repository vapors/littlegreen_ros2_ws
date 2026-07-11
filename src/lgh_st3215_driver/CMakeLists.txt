cmake_minimum_required(VERSION 3.8)
project(lgh_st3215_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoTelemetry.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)


add_library(${PROJECT_NAME}_core
  src/protocol.cpp
  src/serial_port.cpp
  src/joint_map.cpp
  src/servo_bus.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core
  yaml-cpp
  Threads::Threads
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

add_executable(lgh_st3215_driver_node
  src/servo_driver_node.cpp
)

target_link_libraries(lgh_st3215_driver_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(lgh_st3215_driver_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
)

target_include_directories(lgh_st3215_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(lgh_st3215_driver_node PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lgh_st3215_driver_node "${cpp_typesupport_target}")

install(
  TARGETS
    ${PROJECT_NAME}_core
    lgh_st3215_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  if(TARGET test_protocol)
    target_link_libraries(test_protocol ${PROJECT_NAME}_core)
    target_include_directories(test_protocol PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(yaml-cpp rosidl_default_runtime)

ament_package()
