
find_package(gtest QUIET)
if (${Gtest_FOUND})
  MESSAGE(STATUS "Found Gtest.")
else (${Gtest_FOUND})
  MESSAGE(STATUS "Could not locate Gtest.")
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

endif(${Gtest_FOUND})

include(GoogleTest)

enable_testing()
add_executable(platform_state_machine_test ./tests/platform_state_machine_test.cpp)
target_link_libraries(platform_state_machine_test gtest_main ${PROJECT_NAME})

gtest_discover_tests(platform_state_machine_test)