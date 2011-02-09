CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(RMLE)

include_directories("./")

file(GLOB RMLEFILES "*.cc")


add_library(RMLE ${RMLEFILES})


install(TARGETS RMLE DESTINATION lib)
