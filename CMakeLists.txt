# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
cmake_minimum_required(VERSION 3.8)
project(socketcan_adapter)

# Project-wide language standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings & link hygiene
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

# Enable ASAN in Debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling AddressSanitizer (ASAN) for Debug build")
  set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ASAN_FLAGS}")
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  can_msgs
)

# Primary library
add_library(socketcan_adapter SHARED
  src/socketcan_adapter.cpp
  src/socketcan_bridge_node.cpp
  src/can_frame.cpp
)
target_include_directories(socketcan_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(socketcan_adapter PRIVATE "SOCKETCAN_ADAPTER_BUILDING_LIBRARY")
ament_target_dependencies(socketcan_adapter ${dependencies})

# Executable using the library
add_executable(socketcan_bridge src/socketcan_bridge.cpp)
target_include_directories(socketcan_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(socketcan_bridge socketcan_adapter)
ament_target_dependencies(socketcan_bridge ${dependencies})

# Installs
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS socketcan_adapter
  EXPORT ${PROJECT_NAME}_TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS socketcan_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(EXPORT ${PROJECT_NAME}_TARGETS
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(Catch2 2 REQUIRED)
  set(test_dependencies Catch2)

  add_executable(can_frame_test test/can_frame_test.cpp)
  target_link_libraries(can_frame_test Catch2::Catch2 socketcan_adapter)
  ament_target_dependencies(can_frame_test ${test_dependencies} ${dependencies})
  ament_add_test(
    can_frame_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/can_frame_test -r junit -o test_results/${PROJECT_NAME}/can_frame_test_output.xml
    ENV CATCH_CONFIG_CONSOLE_WIDTH=120
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

  # Don't try to run vcan-based tests in CI just yet
  if(NOT DEFINED ENV{CI})
    add_executable(socketcan_adapter_test test/socketcan_adapter_test.cpp)
    target_link_libraries(socketcan_adapter_test Catch2::Catch2 socketcan_adapter)
    ament_target_dependencies(socketcan_adapter_test ${test_dependencies} ${dependencies})
    ament_add_test(
      socketcan_adapter_test
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND ${CMAKE_CURRENT_BINARY_DIR}/socketcan_adapter_test -r junit -o test_results/${PROJECT_NAME}/socketcan_adapter_test_output.xml
      ENV CATCH_CONFIG_CONSOLE_WIDTH=120
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
  endif()
endif()

# Clean export of library targets (no separate include/lib exports needed)
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_package()
