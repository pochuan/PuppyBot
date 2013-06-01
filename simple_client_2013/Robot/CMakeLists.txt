################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(simple_client_2013)

#Set the build mode to release by default
#SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the executable#################
#Set the include directories
SET(PR_INC_DIR ../include/)

INCLUDE_DIRECTORIES(${PR_INC_DIR} /opt/local/include /opt/local/include/opencv /opt/local/include/opencv2)
ADD_DEFINITIONS(-DPR_MISSING_NEWMAT10_LIB)

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(ALL_SRC RobotCom.cpp
						unitMsg.cpp
#						ExampleUsage.cpp)
						main.cpp)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(simple_client_2013 ${ALL_SRC})

################Add the PR Library#################
find_library( PR_LIBRARY_DEBUG NAMES matrix
            PATHS   ${PR_INC_DIR}../lib/
            PATH_SUFFIXES Debug )

find_library( PR_LIBRARY_RELEASE NAMES matrix
            PATHS   ${PR_INC_DIR}../lib/
            PATH_SUFFIXES Release )

SET( PR_LIBRARY debug     ${PR_LIBRARY_DEBUG}
              optimized ${PR_LIBRARY_RELEASE} )


target_link_libraries(simple_client_2013 ${PR_LIBRARY} -L/opt/local/lib -L/usr/local/lib -lopencv_core -lopencv_features2d -lopencv_highgui -lopencv_imgproc)

