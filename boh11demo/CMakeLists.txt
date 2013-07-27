cmake_minimum_required(VERSION 2.6)

PROJECT(BOHDEMO)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/lib/atlas-base/atlas /usr/lib)
# if using Ubuntu < 12.10, atlas libs may be in a different location, use this line instead if necessary
#LINK_DIRECTORIES(/home/logan/uiuc-lar/lib /usr/lib/atlas /usr/lib)


# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              
SET(CMAKE_CXX_FLAGS_DEBUG "-g")

ADD_EXECUTABLE(armFwdKin  fwdConverter.cpp)
ADD_EXECUTABLE(maDetect   actionVad.cpp)
ADD_EXECUTABLE(vaDetect audioProcessing.cpp)
ADD_EXECUTABLE(phoneticClassifier phoneClassifier.cpp)
ADD_EXECUTABLE(lexiconLearner lexiconLearner.cpp)
ADD_EXECUTABLE(associativeMemory associativeMemory.cpp)
ADD_EXECUTABLE(demoGate demoGate.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(vaDetect ${YARP_LIBRARIES} ${ICUB_LIBRARIES} speech fftw3)
TARGET_LINK_LIBRARIES(phoneticClassifier ${YARP_LIBRARIES} ${ICUB_LIBRARIES} RMLE imatlib torch blas lapack)
TARGET_LINK_LIBRARIES(lexiconLearner RMLE imatlib lexicon ${YARP_LIBRARIES} ${ICUB_LIBRARIES} torch blas lapack gsl)
TARGET_LINK_LIBRARIES(associativeMemory ${YARP_LIBRARIES} ${ICUB_LIBRARIES} RMLE imatlib torch blas lapack)
TARGET_LINK_LIBRARIES(armFwdKin ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(maDetect ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(demoGate ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

INSTALL(TARGETS vaDetect phoneticClassifier lexiconLearner associativeMemory armFwdKin maDetect demoGate DESTINATION bin)
