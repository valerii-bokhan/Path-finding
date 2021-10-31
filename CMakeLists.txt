cmake_minimum_required(VERSION 3.10)

project(Test)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS Src/*.cpp Src/*.h)

include_directories(Src)
 
add_executable(Test ${SOURCES})

target_compile_features(Test PUBLIC cxx_std_17)
set_target_properties(Test PROPERTIES CXX_EXTENSIONS OFF)