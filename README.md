# Related projects

[CMSIS_5](https://github.com/ARM-software/CMSIS_5)

[conan-gcc](https://github.com/ilia19911/arm_tools-conan_gcc)

[conan-gtest](https://github.com/ilia19911/arm_tools-conan-gtest)

[conan-hal](https://github.com/ilia19911/arm_tools-conan_hal)


# arm-tools-simple_stm32f4
You can find here an example of using conan toolchain or library in relation to embedded. This project solves the problem of using gtest for a high level. Switching between PC and MCU will be done using a flag passed to cmake . In this implementation, the conan manager does not require an extension for the IDE since its call is integrated into cmake . You do not necessarily need to have a compiler or a built gtest on your computer since the package manager itself will pull in the necessary dependencies during the cmake configuration.


# Motivation

The motivation for creating this build system comes from the availability of all necessary tools for building ARM32 projects freely on the internet, but the lack of any satisfactory build system implementation. I believe that a build system should allow easy switching between compilers, separation of hardware logic from business logic, and support multiple versions of CMSIS HAL without needing to copy them into each project.

Additionally, CMSIS often does not include precompiled NN and DSP libraries, so these must be compiled manually. Many users search for alternative ways to integrate DSP files into their projects for various reasons (e.g., large binary size), even though DSP/Source/CMakeLists.txt contains clear build instructions. Moreover, CMSIS can be built for a PC, simplifying algorithm testing.

Typically, one project assumes one target with flags for your processor, which is not convenient when you have tests running on a PC or projects where more than one firmware must be compiled, for example for f1 and f4. This project solves this problem and allows the user to fine-tune targets to suit their goals, even with tests on PC

When generating a project in CubeMX, users often struggle to easily control which version of libraries will be used in the project, and sometimes CubeMX hasn't yet been updated to the latest versions, such as HAL, CMSIS, or FatFS. As a result, depending on the machine where the project was generated, the libraries may have different versions, leading to potential compatibility issues or even the use of outdated libraries. This project solves the problem by allowing easy switching between different library versions or even locking a specific version for the entire development team.

CMSIS provides driver interface descriptions in C, but today, writing embedded projects purely in C for STM32 is becoming less common due to increasing algorithmic complexity. It’s essential to adapt to modern development needs. Furthermore, the compiler flags required to reduce binary size and apply proper processor settings remain a mystery to many users.

If you disagree with any of my points or have resources to share, I welcome any feedback or constructive criticism.

# Requirements
- Python 3 and required libraries (install the necessary ones for your system)
- Conan client installed (find instructions at https://conan.io/downloads)
  
# Fast start

  To use my packages, add my repository to the list of Conan repositories, and you will be able to find the required GCC version with the necessary host and target options.
  
  ```
  conan remote add arm-tools  https://artifactory.nextcloud-iahve.ru/artifactory/api/conan/arm-tools
  ```

# How to use


