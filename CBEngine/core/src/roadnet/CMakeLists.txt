project(citybrain)

aux_source_directory(. ROAD_SRCS)

add_library(roadnet ${ROAD_SRCS})

target_link_libraries(roadnet common)

include_directories( "${project_BINARY_DIR}/common"
                     "${project_BINARY_DIR}/config"
                     "${project_BINARY_DIR}/vehicle"
                     "${project_BINARY_DIR}/engine"
                     "${project_BINARY_DIR}/roadnet"
                           )

include_directories(../head)