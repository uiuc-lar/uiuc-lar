cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME maze)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# Search for source code.
file(GLOB folder_source *.cpp *.cc *.c)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_DIR}/conf ${ICUB_DIR}/conf)

# Automatically add include directories if needed.
foreach(header_file ${folder_header})
  get_filename_component(p ${header_file} PATH)
  include_directories(${p})
endforeach(header_file ${folder_header})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              
SET(CMAKE_CXX_FLAGS_DEBUG "-g")

# Set up our main executable.
if (folder_source)
  add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
  TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

