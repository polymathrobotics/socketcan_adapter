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
  find_package(Catch2 2 REQUIRED)

  # Provides catch_discover_tests()
  include(Catch)

  # Unit tests
  add_executable(can_frame_tests
    test/can_frame_test.cpp
  )
  target_link_libraries(can_frame_tests
    PRIVATE socketcan_adapter Catch2::Catch2WithMain
  )
  catch_discover_tests(can_frame_tests)

  # Optional hardware / integration tests (only if CAN_AVAILABLE is set)
  if(DEFINED ENV{CAN_AVAILABLE})
    add_executable(socketcan_adapter_hardware_tests
      test/socketcan_adapter_test.cpp
    )
    target_link_libraries(socketcan_adapter_hardware_tests
      PRIVATE socketcan_adapter Catch2::Catch2WithMain
    )
    catch_discover_tests(socketcan_adapter_hardware_tests)
    message(STATUS "CAN_AVAILABLE set - including hardware tests")
  else()
    message(STATUS "CAN_AVAILABLE not set - skipping hardware tests (set CAN_AVAILABLE=1 to enable)")
  endif()
endif()

# Clean export of library targets (no separate include/lib exports needed)
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_package()
