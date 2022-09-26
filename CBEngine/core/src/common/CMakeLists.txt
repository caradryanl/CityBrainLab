project(citybrain)

aux_source_directory(. COMMON_SRCS)

add_library(common ${COMMON_SRCS})

include_directories(../head)