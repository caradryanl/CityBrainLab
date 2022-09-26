project(citybrain)

aux_source_directory(. ENGINE_SRCS)

add_library(engine ${ENGINE_SRCS})

target_link_libraries(engine common)
target_link_libraries(engine config)
target_link_libraries(engine vehicle)
target_link_libraries(engine roadnet)

include_directories( "${project_BINARY_DIR}/common"
                     "${project_BINARY_DIR}/config"
                     "${project_BINARY_DIR}/vehicle"
                     "${project_BINARY_DIR}/engine"
                     "${project_BINARY_DIR}/roadnet"
                           )
include_directories(../head)
