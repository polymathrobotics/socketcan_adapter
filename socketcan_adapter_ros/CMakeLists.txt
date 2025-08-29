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
project(socketcan_adapter_ros)

# Project-wide language standards
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

# Distro detection (override with -BUILD_HUMBLE=ON/OFF)
if(NOT DEFINED BUILD_HUMBLE)
  if(DEFINED ENV{ROS_DISTRO} AND "$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(BUILD_HUMBLE TRUE)
    message(STATUS "ROS_DISTRO=humble -> using Catch2 v2")
  else()
    set(BUILD_HUMBLE FALSE)
    message(STATUS "ROS_DISTRO>humble -> Catch2 v3 (Jazzy/Rolling)")
  endif()
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_msgs REQUIRED)
find_package(socketcan_adapter REQUIRED)

# Primary library
add_library(socketcan_adapter_ros SHARED
  src/socketcan_bridge_node.cpp
)
target_include_directories(socketcan_adapter_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(socketcan_adapter_ros PRIVATE "SOCKETCAN_ADAPTER_ROS_BUILDING_LIBRARY")

# Executable using the library
add_executable(socketcan_bridge src/socketcan_bridge.cpp)
target_include_directories(socketcan_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(socketcan_adapter_ros PUBLIC
  ${can_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  socketcan_adapter::socketcan_adapter
)
target_link_libraries(socketcan_bridge PUBLIC socketcan_adapter_ros)

# Installs
install(DIRECTORY include/ DESTINATION include)
install(TARGETS socketcan_adapter_ros
  EXPORT ${PROJECT_NAME}_TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS socketcan_bridge DESTINATION lib/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}_TARGETS
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_export_dependencies(socketcan_adapter rclcpp rclcpp_lifecycle can_msgs)
ament_package()
