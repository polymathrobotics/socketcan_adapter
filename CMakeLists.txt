cmake_minimum_required(VERSION 3.8)
project(socketcan_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  can_msgs
)

add_library(socketcan_adapter SHARED
            src/socketcan_adapter.cpp
            src/socketcan_bridge_node.cpp
            src/can_frame.cpp)

target_compile_features(socketcan_adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(socketcan_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(socketcan_adapter PRIVATE "SOCKETCAN_ADAPTER_BUILDING_LIBRARY")

ament_target_dependencies(socketcan_adapter ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS socketcan_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(socketcan_bridge src/socketcan_bridge.cpp)
target_include_directories(socketcan_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(socketcan_bridge socketcan_adapter)

ament_target_dependencies(socketcan_bridge ${dependencies})

install(TARGETS socketcan_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(Catch2 2 REQUIRED)
  set(test_dependencies
    Catch2
  )

  add_executable(can_frame_test test/can_frame_test.cpp)
  target_link_libraries(can_frame_test Catch2::Catch2 socketcan_adapter)
  ament_target_dependencies(
    can_frame_test
    ${test_dependencies}
    ${dependencies}
  )

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
    ament_target_dependencies(socketcan_adapter_test
      ${test_dependencies}
      ${dependencies}
    )
    ament_add_test(
      socketcan_adapter_test
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND ${CMAKE_CURRENT_BINARY_DIR}/socketcan_adapter_test -r junit -o test_results/${PROJECT_NAME}/socketcan_adapter_test_output.xml
      ENV CATCH_CONFIG_CONSOLE_WIDTH=120
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
  endif()

endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  socketcan_adapter
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
