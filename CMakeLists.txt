cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME pid_controller)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  include/${PROJECT_NAME}/pid.hpp
  include/${PROJECT_NAME}/pid_1d.hpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})

# Specify the language for the target as C++
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Export library headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Add main executable
# add_executable(${PROJECT_NAME}_main src/${PROJECT_NAME}_main.cpp)
# target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)
  set(TEST_DEPENDENCIES
    benchmark
  )

  foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
  endforeach()

  file(GLOB_RECURSE TEST_SOURCE "tests/**/*_test.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()

  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)

  file(GLOB_RECURSE TEST_SOURCE "tests/**/*_gtest.cpp")
  MESSAGE(STATUS "TEST_SOURCE: ${TEST_SOURCE}")

  foreach(TEST_FILE ${TEST_SOURCE})
    MESSAGE(STATUS "TEST_FILE: ${TEST_FILE}")
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest_main ${PROJECT_NAME})
  endforeach()

  file(GLOB_RECURSE TEST_SOURCE "tests/**/*_benchmark.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
  endforeach()
endif()
