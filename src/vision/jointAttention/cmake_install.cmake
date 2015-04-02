# Install script for directory: /home/niehaus4/uiuc-lar/src/vision/jointAttention

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/niehaus4/uiuc-lar")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/faceDetector")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/faceDetector")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/shakeSalience")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/shakeSalience")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/jointAttention")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/objectSalience")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSalience")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/objectFeatures")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectFeatures")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/randObjSeg")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/randObjSeg")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/objectSegmentation")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objectSegmentation")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/topDownObjectMap")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/topDownObjectMap")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/csSalience")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/csSalience")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/gazeEstimator2")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/gazeEstimator2")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/jointAttention3d")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/jointAttention3d")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/src/vision/jointAttention/objSegTrack")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack"
         OLD_RPATH "/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/objSegTrack")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

