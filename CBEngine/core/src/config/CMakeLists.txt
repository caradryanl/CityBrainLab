project(citybrain)

aux_source_directory(. CONFIG_SRCS)

add_library(config ${CONFIG_SRCS})

include_directories(../head)