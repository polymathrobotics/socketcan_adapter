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
project(axiomatic_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options("-Wl,--no-undefined")
endif()

if(NOT DEFINED BUILD_JAMMY)
  set(BUILD_JAMMY OFF)
  if(EXISTS "/etc/os-release")
    file(READ "/etc/os-release" _OS_RELEASE)
    string(REGEX MATCH "VERSION_CODENAME=([^\n\r]+)" _match "${_OS_RELEASE}")
    if(CMAKE_MATCH_1)
      string(TOLOWER "${CMAKE_MATCH_1}" _UBUNTU_CODENAME)
      if(_UBUNTU_CODENAME STREQUAL "jammy")
        set(BUILD_JAMMY ON)
      endif()
    endif()
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(CLI11 REQUIRED)
find_package(socketcan_adapter REQUIRED)

# Axiomatic Adapter Library
add_library(axiomatic_adapter SHARED
  src/axiomatic_adapter.cpp
  src/axiomatic_frame_parser.cpp
)

target_compile_features(axiomatic_adapter PUBLIC c_std_99 cxx_std_17)
target_include_directories(axiomatic_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(axiomatic_adapter PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
target_link_libraries(axiomatic_adapter
  PUBLIC socketcan_adapter::socketcan_adapter
  PRIVATE
    Boost::system
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS axiomatic_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Axiomatic SocketCAN Bridge
add_executable(axiomatic_socketcan_bridge
  src/axiomatic_socketcan_bridge.cpp
  src/axiomatic_socketcan_bridge_node.cpp
)

target_include_directories(axiomatic_socketcan_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(axiomatic_socketcan_bridge PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
target_link_libraries(axiomatic_socketcan_bridge
  axiomatic_adapter
  CLI11::CLI11
  socketcan_adapter::socketcan_adapter
)

# Install the bridge executable
install(TARGETS axiomatic_socketcan_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  if(BUILD_JAMMY)
    find_package(Catch2 2 REQUIRED)
  else()
    find_package(Catch2 3 REQUIRED)
  endif()
  include(Catch OPTIONAL)

  option(HARDWARE_CONNECTED "Enable tests that require a connected Axiomatic device and CAN interface" OFF)

  # Pure-software tests: framer logic and the send-length wire encoding via a
  # localhost TCP loopback. No Axiomatic hardware required.
  #
  # Sources are compiled directly into each test executable rather than linked
  # via the shared library. This makes the tests independent of any older
  # libaxiomatic_adapter.so that might be on LD_LIBRARY_PATH from a prior
  # install (e.g. /opt/polymath) and would otherwise shadow the freshly-built
  # symbols at runtime.
  add_executable(test_axiomatic_frame_parser
    test/axiomatic_frame_parser_test.cpp
    src/axiomatic_frame_parser.cpp
  )
  target_include_directories(test_axiomatic_frame_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_axiomatic_frame_parser PRIVATE
    socketcan_adapter::socketcan_adapter
    Catch2::Catch2WithMain
  )
  if(COMMAND catch_discover_tests)
    catch_discover_tests(test_axiomatic_frame_parser)
  else()
    add_test(NAME test_axiomatic_frame_parser COMMAND test_axiomatic_frame_parser)
  endif()

  add_executable(test_axiomatic_send_wire_format
    test/axiomatic_send_wire_format_test.cpp
    src/axiomatic_adapter.cpp
    src/axiomatic_frame_parser.cpp
  )
  target_include_directories(test_axiomatic_send_wire_format PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_axiomatic_send_wire_format PRIVATE
    socketcan_adapter::socketcan_adapter
    Boost::system
    Catch2::Catch2WithMain
  )
  if(COMMAND catch_discover_tests)
    catch_discover_tests(test_axiomatic_send_wire_format)
  else()
    add_test(NAME test_axiomatic_send_wire_format COMMAND test_axiomatic_send_wire_format)
  endif()

  add_executable(test_axiomatic_concurrent_io
    test/axiomatic_concurrent_io_test.cpp
    src/axiomatic_adapter.cpp
    src/axiomatic_frame_parser.cpp
  )
  target_include_directories(test_axiomatic_concurrent_io PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_axiomatic_concurrent_io PRIVATE
    socketcan_adapter::socketcan_adapter
    Boost::system
    Catch2::Catch2WithMain
  )
  if(COMMAND catch_discover_tests)
    catch_discover_tests(test_axiomatic_concurrent_io)
  else()
    add_test(NAME test_axiomatic_concurrent_io COMMAND test_axiomatic_concurrent_io)
  endif()

  if(HARDWARE_CONNECTED)
    add_executable(test_axiomatic_adapter test/axiomatic_adapter_test.cpp)
    target_link_libraries(test_axiomatic_adapter PRIVATE
      axiomatic_adapter
      socketcan_adapter::socketcan_adapter
      Catch2::Catch2WithMain
    )
    if(COMMAND catch_discover_tests)
      catch_discover_tests(test_axiomatic_adapter)
    else()
      add_test(NAME test_axiomatic_adapter COMMAND test_axiomatic_adapter)
    endif()
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
