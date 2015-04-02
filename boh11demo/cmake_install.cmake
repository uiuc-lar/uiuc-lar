# Install script for directory: /home/niehaus4/uiuc-lar/boh11demo

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
    SET(CMAKE_INSTALL_CONFIG_NAME "")
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
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/boh11demo/vaDetect")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect"
         OLD_RPATH "/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/vaDetect")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/boh11demo/phoneticClassifier")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier"
         OLD_RPATH "/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/phoneticClassifier")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/boh11demo/associativeMemory")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory"
         OLD_RPATH "/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/associativeMemory")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/boh11demo/armFwdKin")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin"
         OLD_RPATH "/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/armFwdKin")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/boh11demo/maDetect")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect"
         OLD_RPATH "/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/maDetect")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate"
         RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/niehaus4/uiuc-lar/boh11demo/demoGate")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate"
         OLD_RPATH "/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty::::::::::::::::::::::::::::"
         NEW_RPATH "/home/niehaus4/uiuc-lar/lib:/usr/local/lib:/home/logan/uiuc-lar/lib:/usr/lib/atlas:/usr/local/robot/ipopt/lib/coin:/usr/local/robot/ipopt/lib/coin/ThirdParty")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/demoGate")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

