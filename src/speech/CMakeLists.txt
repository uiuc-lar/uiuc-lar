CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(speech)

add_library(speech mfcc.cpp)
install(TARGETS speech DESTINATION lib)
