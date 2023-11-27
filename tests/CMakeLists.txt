# Tests CMakeLists.txt

file(GLOB_RECURSE ALL_SOURCE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../include/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../include/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../tests/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../tests/*.cpp"
)

# CppLint tests
add_test(
  NAME RunCppLint
  COMMAND cpplint ${ALL_SOURCE_FILES} --counting=detailed --recursive
)

# ClangFormat tests
foreach(SOURCE_FILE ${ALL_SOURCE_FILES})
  add_test(NAME clang-format-${SOURCE_FILE}
    COMMAND clang-format -output-replacements-xml -style=file ${SOURCE_FILE})
  set_tests_properties(clang-format-${SOURCE_FILE} PROPERTIES
    FAIL_REGULAR_EXPRESSION "<replacement "
    PASS_REGULAR_EXPRESSION "")
endforeach()

# Executable tests
file(GLOB_RECURSE TEST_SOURCE "*_test.cpp")

if(TEST_SOURCE)
  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()
endif()

# Google Test tests
file(GLOB_RECURSE GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)

  foreach(TEST_FILE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest_main ${PROJECT_NAME})

    add_test(
      NAME ${PROJECT_NAME}_${TEST_NAME}
      COMMAND ${PROJECT_NAME}_${TEST_NAME}
    )
  endforeach()
endif()

# Google Benchmark tests
file(GLOB_RECURSE BTEST_SOURCE "*_benchmark.cpp")

if(BTEST_SOURCE)
  find_package(benchmark REQUIRED)

  foreach(TEST_FILE ${BTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
  endforeach()
endif()