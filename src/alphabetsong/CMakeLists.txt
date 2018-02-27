

cmake_minimum_required(VERSION 2.0) #this line is required, I just put some version < current version...

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(BECKMAN_ECARD)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)


# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              

ADD_EXECUTABLE(ecard ecard.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(ecard ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
