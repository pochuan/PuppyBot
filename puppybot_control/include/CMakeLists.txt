################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_COMPILER "/opt/local/bin/c++")

#Name the project
project(matrix)

#Set the build mode to release by default
SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the executable#################

ADD_DEFINITIONS(-DPR_MISSING_NEWMAT10_LIB)

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
FILE(GLOB ALL_SRC "*.cpp")

#Set the destination:
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../lib/${CMAKE_BUILD_TYPE})

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_library(matrix STATIC ${ALL_SRC})
