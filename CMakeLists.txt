cmake_minimum_required(VERSION 3.14)
project(abs_ecu_faultguard VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include directory for headers
include_directories(include)

# ── Main simulation executable ────────────────────────────────────────────────
file(GLOB SOURCES "src/*.cpp")
add_executable(abs_ecu_sim main.cpp ${SOURCES})

# ── Unit tests via GoogleTest (FetchContent — no manual install needed) ────────
include(FetchContent)
FetchContent_Declare(
    googletest
    URL      https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
# On Windows, prevent GoogleTest from overriding the /MT or /MD flag
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_subdirectory(tests)
