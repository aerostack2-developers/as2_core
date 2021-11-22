cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_core)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES 
  src/node.cpp
  src/sensor.cpp
  src/aerial_platform.cpp
  src/platform_state_machine.cpp
  src/utils.cpp
)

add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs sensor_msgs nav_msgs as2_msgs geometry_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp std_msgs std_srvs sensor_msgs nav_msgs as2_msgs geometry_msgs
  )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# add_executable(platform_state_machine_test tests/platform_state_machine_test.cpp src/platform_state_machine.cpp)
# target_link_libraries(platform_state_machine_test ${PROJECT_NAME}) 


if(BUILD_TESTING)
  # find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  # ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

  include(./tests/CMakeLists.txt)

endif()


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

    
    # if(BUILD_TESTING)
    #   find_package(ament_lint_auto REQUIRED)
    #   ament_lint_auto_find_test_dependencies()
    # endif()
    
ament_package()
