CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(imatlib)

file(GLOB IMATFILES "*.cc")

include_directories(imatlib ${ATLAS_CLAPACK_INCLUDE_DIR})

add_library(imatlib ${IMATFILES})


install(TARGETS imatlib DESTINATION lib)
