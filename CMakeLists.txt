cmake_minimum_required(VERSION 3.28)


if(TEST_TYPE STREQUAL "SOME_TEST")
    message("TEST SOME_TEST")
    add_subdirectory(SomeTests)
elseif (NOT TEST_TYPE)
    add_subdirectory(McuPoject)
endif ()

project(7C3)

add_executable(main_project main.cpp)

target_link_libraries(main_project MCU_F4)

if(CMAKE_CROSSCOMPILING)
    STM32_ADD_HEX_BIN_TARGETS(main_project)
    STM32_PRINT_SIZE_OF_TARGETS(main_project)
endif ()
