cmake_minimum_required(VERSION 3.28)

include(.project/project_init.cmake)
project(mcu_project)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(Platforms/Stm32f411xe)

set(SOURCE_LIST
        ${CMAKE_CURRENT_LIST_DIR}/Source/freertos.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/main.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Source/stm32f4xx_hal_msp.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/stm32f4xx_hal_timebase_tim.c
        ${CMAKE_CURRENT_LIST_DIR}/Source/stm32f4xx_it.cpp
)

file(GLOB_RECURSE RTOS_SOURCE_LIST "${CMAKE_CURRENT_LIST_DIR}/cubemx/Middlewares/Third_Party/FreeRTOS/Source/*")

SET(PLATFORM_SOURCE_LIST
        ${SOURCE_LIST}
        ${SYSTEM_SRC}
        ${RTOS_SOURCE_LIST}
)

SET(PLATFORM_INCLUDE_LIST
        ${CMAKE_CURRENT_LIST_DIR}/Include
        ${SYSTEM_INC}
        ${CMAKE_CURRENT_LIST_DIR}/cubemx/Middlewares/Third_Party/FreeRTOS/Source/include
        ${CMAKE_CURRENT_LIST_DIR}/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        ${CMAKE_CURRENT_LIST_DIR}/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
)

add_library(MCU_F4 ${PLATFORM_SOURCE_LIST})

ADD_HAL_STM32F4_TO_TARGET(MCU_F4 "${CMAKE_CURRENT_LIST_DIR}/Include/stm32f4xx_hal_conf.h")

find_package(CMSIS_M4 REQUIRED)
target_link_libraries(MCU_F4 PUBLIC
        CORTEX_M4
        CMSIS::CMSIS_M4
)

target_compile_definitions(MCU_F4 PUBLIC STM32F411xE)
target_include_directories(MCU_F4 PUBLIC ${INC} ${PLATFORM_INCLUDE_LIST})
target_link_options(MCU_F4  PUBLIC "-Wl,-Map=${PROJECT_NAME}.map" "-Wl,-T${LD_SCRIPT}" )

