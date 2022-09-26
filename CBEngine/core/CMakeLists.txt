cmake_minimum_required(VERSION 3.10)
project(citybrain)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")


add_subdirectory(src/common)
add_subdirectory(src/config)
add_subdirectory(src/engine)
add_subdirectory(src/roadnet)
add_subdirectory(src/vehicle)
add_subdirectory(externals/pybind11)

add_executable(citybrain main.cc)

target_link_libraries(citybrain PUBLIC engine)
target_link_libraries(citybrain PUBLIC config)
target_link_libraries(citybrain PUBLIC pybind11::embed
)

target_include_directories(citybrain PUBLIC )

include_directories(./src/head
./head/compacted_graph.h
./head/config.h
./head/core.h
./head/roadnet.h
./head/vehicle.h
./head/trafficflow.h
)

AUX_SOURCE_DIRECTORY(src/common SRC_LIST)
AUX_SOURCE_DIRECTORY(src/config SRC_LIST)
AUX_SOURCE_DIRECTORY(src/engine SRC_LIST)
AUX_SOURCE_DIRECTORY(src/roadnet SRC_LIST)
AUX_SOURCE_DIRECTORY(src/vehicle SRC_LIST)
AUX_SOURCE_DIRECTORY(src/modules SRC_LIST)
pybind11_add_module(cbengine src/cbengine.cpp ${SRC_LIST})
#target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_LIB_NAME})
