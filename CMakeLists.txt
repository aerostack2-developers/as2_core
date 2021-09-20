cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME aerostack2_core)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aerostack2_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  include/${PROJECT_NAME}
)

set(HEADER_HPP_FILES 
  include/${PROJECT_NAME}/node.hpp
  include/${PROJECT_NAME}/sensor.hpp
  include/${PROJECT_NAME}/aerial_platform.hpp
  
)

set(SOURCE_CPP_FILES 
  src/node.cpp
  src/sensor.cpp
  src/aerial_platform.cpp
)

add_library(${PROJECT_NAME} ${HEADER_HPP_FILES} ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs aerostack2_msgs)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)


if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
