cmake_minimum_required(VERSION 3.5)

project(roadnet-match LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src)

add_executable(roadnet-match main.cpp)

target_link_libraries(roadnet-match PUBLIC src)

include_directories(head)
