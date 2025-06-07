cmake_minimum_required(VERSION 3.5)
project(opt_path_search)

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