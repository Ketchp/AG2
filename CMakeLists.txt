cmake_minimum_required(VERSION 3.25)
project(AG2)

add_compile_options(-pedantic -Wall)
#add_link_options(-fsanitize=address)

set(CMAKE_CXX_STANDARD 20)


add_executable(pt01 pt01.cpp)
add_executable(pt02 pt02.cpp)

project(AG2)
