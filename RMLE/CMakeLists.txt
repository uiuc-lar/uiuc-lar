CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(RMLE)

FIND_PACKAGE(YARP)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_DIR}/conf)

INCLUDE_DIRECTORIES("./" ${YARP_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/lib/atlas-base/atlas /usr/lib)
# if using Ubuntu < 12.10, atlas libs may be in a different location, use this line instead if necessary
#LINK_DIRECTORIES(/home/logan/uiuc-lar/lib /usr/lib/atlas /usr/lib)


# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")              
SET(CMAKE_CXX_FLAGS_DEBUG "-g")


file(GLOB RMLEFILES "*.cc")

ADD_LIBRARY(RMLE ${RMLEFILES})
ADD_EXECUTABLE(hmmRMLE  hmmRMLE.cpp)

TARGET_LINK_LIBRARIES(hmmRMLE  ${YARP_LIBRARIES} RMLE imatlib torch blas lapack gsl)

install(TARGETS RMLE DESTINATION lib)
install(TARGETS hmmRMLE DESTINATION bin)
