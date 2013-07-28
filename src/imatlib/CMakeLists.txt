CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(imatlib)

file(GLOB IMATFILES "*.cc")


add_library(imatlib ${IMATFILES})


install(TARGETS imatlib DESTINATION lib)
