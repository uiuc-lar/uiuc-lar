CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(lexicon)

file(GLOB LEXFILES "*.cpp")


add_library(lexicon ${LEXFILES})
target_link_libraries(lexicon torch RMLE imatlib)
install(TARGETS lexicon DESTINATION lib)
