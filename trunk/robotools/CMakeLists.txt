PROJECT(ACTIONDEMO)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
LINK_DIRECTORIES(/home/logan/uiuc-lar/lib /usr/lib/atlas /usr/local/lib)

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              

#ADD_EXECUTABLE(actionDemo main.cpp)
ADD_EXECUTABLE(performAction actionPerformer.cpp)
ADD_EXECUTABLE(dataPumper dataPumper.cpp)
ADD_EXECUTABLE(bottleLogger bottleLogger.cpp)
ADD_EXECUTABLE(floatToRgb floatToRgb.cpp)
ADD_EXECUTABLE(portToScreen portToScreen.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(performAction ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(dataPumper ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(bottleLogger ${YARP_LIBRARIES} ${ICUB_LIBRARIES} imatlib torch blas lapack)
TARGET_LINK_LIBRARIES(floatToRgb ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(portToScreen ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
