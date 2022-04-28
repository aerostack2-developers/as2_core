cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_core)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  as2_msgs
  std_srvs
  geometry_msgs
  rclcpp_lifecycle
  rclcpp_action
  yaml-cpp
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()



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
  src/tf_utils.cpp
  src/rate.cpp
  src/core_functions.cpp
  src/control_mode_utils/control_mode_utils.cpp
  src/yaml_utils/yaml_utils.cpp
)

# set(INCLUDE_HPP_FILES 
#   include/${PROJECT_NAME}/core_libs.hpp

# )

add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES} )
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# add_executable(platform_state_machine_test tests/platform_state_machine_test.cpp src/platform_state_machine.cpp)
# target_link_libraries(platform_state_machine_test ${PROJECT_NAME}) 


if(BUILD_TESTING)
  # find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  # ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/)

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
