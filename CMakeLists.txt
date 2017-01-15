project(landmarkprm)
cmake_minimum_required(VERSION 2.8)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x -pg -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -O3")

#add_subdirectory(include)
#add_subdirectory(src)
#add_subdirectory(python)

add_executable(bug_trap bug_trap.cpp)



