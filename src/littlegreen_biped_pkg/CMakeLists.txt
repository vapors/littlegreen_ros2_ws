cmake_minimum_required(VERSION 3.8)
project(littlegreen_biped_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



# Set ONNX Runtime path
#set(ONNXRUNTIME_DIR "$ENV{HOME}/libs/onnxruntime-linux-aarch64-1.22.0")
#include_directories(${ONNXRUNTIME_DIR}/include)
#link_directories(${ONNXRUNTIME_DIR}/lib)


# ---------------------------------------------------------
# ONNX Runtime
#
# Priority:
#   1. Use environment variable ONNXRUNTIME_DIR if provided.
#   2. Otherwise choose the default path based on architecture.
# ---------------------------------------------------------

set(ONNXRUNTIME_VERSION "1.22.0")

if(DEFINED ENV{ONNXRUNTIME_DIR} AND
   NOT "$ENV{ONNXRUNTIME_DIR}" STREQUAL "")

  set(ONNXRUNTIME_DIR "$ENV{ONNXRUNTIME_DIR}")

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")

  set(
    ONNXRUNTIME_DIR
    "$ENV{HOME}/libs/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}"
  )

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")

  set(
    ONNXRUNTIME_DIR
    "$ENV{HOME}/libs/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}"
  )

else()

  message(
    FATAL_ERROR
    "Unsupported processor architecture: ${CMAKE_SYSTEM_PROCESSOR}. "
    "Set ONNXRUNTIME_DIR manually."
  )

endif()

message(STATUS "ONNX Runtime directory: ${ONNXRUNTIME_DIR}")

find_path(
  ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_DIR}/include"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(
  ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  PATHS "${ONNXRUNTIME_DIR}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)


# Policy runtime and offline ONNX tensor-contract probe.
add_executable(littlegreen_biped_node src/littlegreen_biped_node.cpp)
add_executable(policy_onnx_contract_probe src/policy_onnx_contract_probe.cpp)



target_include_directories(
  littlegreen_biped_node
  PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(
  policy_onnx_contract_probe
  PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
)

find_package(yaml-cpp REQUIRED)
#target_link_libraries(littlegreen_biped_node yaml-cpp)

# Link ONNX and ROS dependencies
#target_link_libraries(littlegreen_biped_node
#  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
#)

target_link_libraries(
  littlegreen_biped_node
  yaml-cpp
  ${ONNXRUNTIME_LIBRARY}
)

target_link_libraries(
  policy_onnx_contract_probe
  ${ONNXRUNTIME_LIBRARY}
)

ament_target_dependencies(littlegreen_biped_node
  rclcpp
  ament_index_cpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  yaml-cpp
)

# Compile the runtime and tensor-contract probe with the v2.8.0 contract helpers.
target_compile_features(littlegreen_biped_node PUBLIC cxx_std_17)
target_compile_features(policy_onnx_contract_probe PUBLIC cxx_std_17)

# Install executable
install(TARGETS littlegreen_biped_node policy_onnx_contract_probe
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# Install policy deployment audit and runtime metrics tools with stable ros2 run names.
install(PROGRAMS scripts/policy_bundle_audit.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME policy_bundle_audit)
install(PROGRAMS scripts/policy_runtime_metrics.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME policy_runtime_metrics)
install(PROGRAMS scripts/annotate_phase_guided_policy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME annotate_phase_guided_policy)

# Install resources
install(DIRECTORY src/checkpoints
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/configs
  DESTINATION share/${PROJECT_NAME})
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES POLICY_SHADOW.md POLICY_LIVE.md POLICY_DEBUG.md POLICY_DEBUG_V2.md
  DESTINATION share/${PROJECT_NAME}
)
# Optional testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_policy_observation_contract
    test/test_policy_observation_contract.cpp
  )
  if(TARGET test_policy_observation_contract)
    target_include_directories(
      test_policy_observation_contract
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_compile_features(test_policy_observation_contract PUBLIC cxx_std_17)
  endif()

  ament_add_pytest_test(
    test_policy_bundle_audit
    test/test_policy_bundle_audit.py
  )
  ament_add_pytest_test(
    test_annotate_phase_guided_policy
    test/test_annotate_phase_guided_policy.py
  )
endif()

ament_package()


