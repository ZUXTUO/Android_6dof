cmake_minimum_required(VERSION 3.22.1)
project(fusion)

add_library(fusion SHARED
        ekf.cpp
        fusion.cpp)

find_library(log-lib log)

target_include_directories(fusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(fusion ${log-lib})
