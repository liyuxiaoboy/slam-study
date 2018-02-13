cmake_minimum_required(VERSION 2.6)
project(eigenmatrix)

include_directories( "/usr/include/eigen3" )

add_executable(eigenmatrix main.cpp)

install(TARGETS eigenmatrix RUNTIME DESTINATION bin)

set(CMAKE_BUILD_TYPE "Debug")

