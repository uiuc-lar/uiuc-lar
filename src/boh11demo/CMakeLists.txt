cmake_minimum_required(VERSION 2.6)

PROJECT(BOHDEMO)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Atlas)

# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
LINK_DIRECTORIES(/usr/lib)

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
TARGET_LINK_LIBRARIES(phoneticClassifier ${YARP_LIBRARIES} ${ICUB_LIBRARIES} RMLE imatlib torch blas lapack lapack_atlas)
TARGET_LINK_LIBRARIES(lexiconLearner RMLE imatlib lexicon ${YARP_LIBRARIES} ${ICUB_LIBRARIES} torch blas lapack lapack_atlas gsl)
TARGET_LINK_LIBRARIES(associativeMemory ${YARP_LIBRARIES} ${ICUB_LIBRARIES} RMLE imatlib torch blas lapack lapack_atlas)
TARGET_LINK_LIBRARIES(armFwdKin ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(maDetect ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(demoGate ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

INSTALL(TARGETS vaDetect phoneticClassifier lexiconLearner associativeMemory armFwdKin maDetect demoGate DESTINATION bin)
