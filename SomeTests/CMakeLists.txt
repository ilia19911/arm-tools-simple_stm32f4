cmake_minimum_required(VERSION 3.28)

include(${CMAKE_CURRENT_LIST_DIR}/.project/project_init.cmake)

project(tests_project)

find_package(GTest REQUIRED)
find_package(CMSIS_M7 REQUIRED)

add_executable(SimpleTest SimpleTest.cpp)
target_link_libraries(SimpleTest GTest::gtest_main CMSIS::CMSIS_M7)

#dump lib
add_library(MCU_F4 INTERFACE)




