# Collect all test source files in this directory.
file(GLOB TEST_SOURCES "*.cpp")

# Reuse the production source files directly (no main.cpp — it defines its own
# main(); GTest::gtest_main provides the test runner main() instead).
file(GLOB SIM_SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(abs_tests ${TEST_SOURCES} ${SIM_SOURCES})

target_include_directories(abs_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(abs_tests GTest::gtest_main)

# Register all TEST() / TEST_F() cases with CTest automatically.
include(GoogleTest)
gtest_discover_tests(abs_tests)
