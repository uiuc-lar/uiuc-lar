CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

ADD_DEFINITIONS(-DUSE_DOUBLE)
SET(CMAKE_C_FLAGS "-w")

add_subdirectory(speech)
add_subdirectory(imatlib)
add_subdirectory(RMLE)
add_subdirectory(lexicon)
add_subdirectory(robotools)
add_subdirectory(boh11demo)
add_subdirectory(randExpDemo)
add_subdirectory(vision)
add_subdirectory(balancing)
add_subdirectory(graspDemo)
add_subdirectory(interactiondemo)
