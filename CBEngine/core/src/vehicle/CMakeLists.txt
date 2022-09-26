project(citybrain)

aux_source_directory(. VEHICLE_SRCS)

add_library(vehicle ${VEHICLE_SRCS})

target_link_libraries(vehicle common)
target_link_libraries(vehicle roadnet)

include_directories( "${project_BINARY_DIR}/common"
                     "${project_BINARY_DIR}/config"
                     "${project_BINARY_DIR}/vehicle"
                     "${project_BINARY_DIR}/engine"
                     "${project_BINARY_DIR}/roadnet"
                           )



include_directories(../head)