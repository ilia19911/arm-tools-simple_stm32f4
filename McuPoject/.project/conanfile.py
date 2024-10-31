from conan import ConanFile, tools
from conan.tools.cmake import CMake
import re

class CompressorRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    # options = {"Device": ["STM32H7", "STM32F4", "STM32F3", "STM32F1", "STM32F0"]}

    def package_id(self):
        print("PROJECT_PACKAGE_ID")

    def configure(self):
        print("PROJECT_CONFIGURE")
        # self.DEVICE = self.buildenv.vars(self).get("DEVICE")
        # self.HSE_VALUE = self.buildenv.vars(self).get("HSE_VALUE")
        # pattern = r"STM32[A-Za-z][0-9]"
        # Поиск совпадения
        # match = re.search(pattern, self.DEVICE)
        # self.SERIES = match.group(0)
        # if match:
        #     print("DEVICE: ", self.DEVICE)
        #     print("SERIES: ", match.group(0))
        #     self.options.Device = match.group(0)
        #     if match.group(0) == "STM32H7":
        #         self.FVP_PLATFORM = "ARMCM7"
        #         self.ARM_CPU = "cortex-m7"
        #     if match.group(0) == "STM32F4":
        #         self.FVP_PLATFORM = "ARMCM4"
        #         self.ARM_CPU = "cortex-m4"

    def tool_requirements(self):
        print("PROJECT_TOOL_REQUIREMENTS")
    def build_requirements(self):
        print("PROJECT_BUILD_REQUIREMENTS")

    def requirements(self):
        print("PROJECT_REQUIREMENTS")


    def generate(self):
        print("PROJECT_GENERATE")
        # toolchain = tools.cmake.CMakeToolchain.filename

        # print("ARM_CPU: ", self.ARM_CPU)
        # print("HSE_VALUE: ", self.HSE_VALUE)
        # print("DEVICE: ", self.DEVICE)
        # print("FVP_PLATFORM: ", self.FVP_PLATFORM)
        # print("toolchain: ", toolchain)
        # with open(toolchain, 'r') as template_file:
        #     template_content = template_file.read()
        # with open(toolchain, "w") as file:
        #     file.write(f"set(SERIES {self.SERIES})\n")
        #     file.write(f"set(ARM_CPU {self.ARM_CPU})\n")
        #     file.write(f"set(HSE_VALUE {self.HSE_VALUE} )\n")
        #     file.write(f"set(DEVICE {self.DEVICE} )\n")
        #     file.write(f"set(FVP_PLATFORM {self.FVP_PLATFORM} )\n")
        #     file.write(template_content)

    def build(self):
        print("PROJECT_BUILD")
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package_info(self):
        print("PROJECT_PACKAGE_INFO")
