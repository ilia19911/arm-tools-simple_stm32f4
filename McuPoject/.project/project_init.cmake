if (APPLE)
    set(PROFILE_BUILD ${CMAKE_CURRENT_LIST_DIR}/Macos_build_conan_profile)
elseif (UNIX)
    set(PROFILE_BUILD ${CMAKE_CURRENT_LIST_DIR}/Linux_build_conan_profile)
else ()
    set(PROFILE_BUILD ${CMAKE_CURRENT_LIST_DIR}/Windows_build_conan_profile)
endif ()


execute_process(
        COMMAND conan install ${CMAKE_CURRENT_LIST_DIR} -pr:b=${PROFILE_BUILD} -pr:h=${CMAKE_CURRENT_LIST_DIR}/conan_profile --output-folder=${CMAKE_BINARY_DIR} -r=arm-tools #    --build missing
        RESULT_VARIABLE CONAN_INSTALL_RESULT
        OUTPUT_VARIABLE CONAN_INSTALL_OUTPUT
)
if(CONAN_INSTALL_RESULT)
    message(FATAL_ERROR "Conan install failed: ${CONAN_INSTALL_RESULT}")
endif()

#option(GCC_VERBOSE "Enable verbose GCC output" ON)
include(${CMAKE_BINARY_DIR}/conan_toolchain.cmake)
