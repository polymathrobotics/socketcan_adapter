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

# Ubuntu detection (override with -DBUILD_JAMMY=ON/OFF)
if(NOT DEFINED BUILD_JAMMY)
  set(BUILD_JAMMY OFF)
  if(EXISTS "/etc/os-release")
    file(READ "/etc/os-release" _OS_RELEASE)
    string(REGEX MATCH "VERSION_CODENAME=([^\n\r]+)" _match "${_OS_RELEASE}")
    if(CMAKE_MATCH_1)
      set(_UBUNTU_CODENAME "${CMAKE_MATCH_1}")
      string(TOLOWER "${_UBUNTU_CODENAME}" _UBUNTU_CODENAME)
      if(_UBUNTU_CODENAME STREQUAL "jammy")
        set(BUILD_JAMMY ON)
        message(STATUS "Ubuntu=jammy (22.04) -> Catch2 v2")
      else()
        set(BUILD_JAMMY OFF)
        message(STATUS "Ubuntu codename='${_UBUNTU_CODENAME}' -> defaulting to Catch2 v3")
      endif()
    endif()
  endif()
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

# Primary library
add_library(socketcan_adapter SHARED
  src/socketcan_adapter.cpp
  src/can_frame.cpp
)
target_include_directories(socketcan_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(socketcan_adapter PRIVATE "SOCKETCAN_ADAPTER_BUILDING_LIBRARY")

# Installs
install(DIRECTORY include/ DESTINATION include)
install(TARGETS socketcan_adapter
  EXPORT ${PROJECT_NAME}_TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT ${PROJECT_NAME}_TARGETS
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Tests
if(BUILD_TESTING)
  include(CTest)
  # Find correct Catch2 major version per distro
  if(BUILD_JAMMY)
    find_package(Catch2 2 REQUIRED)
  else()
    find_package(Catch2 3 REQUIRED)
  endif()

  # Try Catch's CMake helpers if present; otherwise we’ll fall back
  include(Catch OPTIONAL)

  add_executable(can_frame_tests test/can_frame_test.cpp)
  target_link_libraries(can_frame_tests PRIVATE socketcan_adapter Catch2::Catch2WithMain)

  if(COMMAND catch_discover_tests)
    catch_discover_tests(can_frame_tests)
  else()
    add_test(NAME can_frame_tests COMMAND can_frame_tests)
  endif()

  if(DEFINED ENV{CAN_AVAILABLE})
    add_executable(socketcan_adapter_hardware_tests test/socketcan_adapter_test.cpp)
    target_link_libraries(socketcan_adapter_hardware_tests PRIVATE socketcan_adapter Catch2::Catch2WithMain)
    if(COMMAND catch_discover_tests)
      catch_discover_tests(socketcan_adapter_hardware_tests)
    else()
      add_test(NAME socketcan_adapter_hardware_tests COMMAND socketcan_adapter_hardware_tests)
    endif()
    message(STATUS "CAN_AVAILABLE set - including hardware tests")
  else()
    message(STATUS "CAN_AVAILABLE not set - skipping hardware tests (set CAN_AVAILABLE=1 to enable)")
  endif()
endif()

# Export
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_package()
