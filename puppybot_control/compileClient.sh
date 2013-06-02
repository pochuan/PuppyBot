#!/bin/bash

#
# Script to compile the KUKA youBot client
#
# Place this script file in the directory "simple_client_2013" which also contains "include", "lib" and "Robot"
# To run: If you are in the directory "sh compileClient.sh"
#
# Date written: June 01, 2013
#

###### Confirming command ######
echo "Begin make..."


###### Going to ./include ######
cd ./include

rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake Makefile

cmake .
make


###### Going to ./Robot ######
cd ../Robot

rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake Makefile

cmake .
make

###### Move Debug file to Release if it exists ######
cd ../lib

mv ./Debug/libmatrix.a ./Release/


###### Confirming make ######
echo "Done Making!"
echo "To run: (1) Go to ./Robot (2) ./simple_client_2013"
