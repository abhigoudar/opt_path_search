cmake_minimum_required(VERSION 3.5)
project(opt_path_search)

set(CMAKE_CXX_STANDARD 14)

find_package(Wget REQUIRED)

execute_process(
  COMMAND ${WGET_EXECUTABLE}
    -O "${CMAKE_SOURCE_DIR}/include/json.hpp"
    "https://github.com/nlohmann/json/releases/download/v3.12.0/json.hpp"
  RESULT_VARIABLE wget_result
  OUTPUT_VARIABLE wget_output
  ERROR_VARIABLE wget_error
)

if(NOT wget_result EQUAL 0)
  message(FATAL_ERROR "wget failed with error: ${wget_error}")
else()
  message(STATUS "Header: ${CMAKE_SOURCE_DIR}/include/json.hpp")
endif()

include_directories(
    include
)

add_executable(plan_shortest_time 
    src/plan_shortest_time.cpp
    src/plan_shortest_time_exe.cpp
)

install(
    TARGETS plan_shortest_time
    DESTINATION bin
)