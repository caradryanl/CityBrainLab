project(roadnet-match)

aux_source_directory(. SRCS)
add_library(src STATIC ${SRCS})

include_directories(../head)
