cmake_minimum_required(VERSION 3.10)

project(l431_bld LANGUAGES C CXX)

MESSAGE(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}.")
MESSAGE(STATUS "MCU: " ${MCU})
MESSAGE(STATUS "LDSCRIPT: " ${LDSCRIPT})
MESSAGE(STATUS "ASM_SOURCES: " ${ASM_SOURCES})
MESSAGE(STATUS "MAP_FILE: " ${MAP_FILE})

# 添加编译参数
add_compile_options(-Wall -fdata-sections -ffunction-sections -Os)
add_link_options(-Wall -fdata-sections -ffunction-sections -Os)

# 递归调用子文件的 CMakeLists.txt
# add_subdirectory(lib)

set_property(SOURCE ${ASM_SOURCES} PROPERTY LANGUAGE C)
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

file(GLOB_RECURSE SOURCE_FILES
  "Core/Src/*.c"
  "Drivers/STM32L4xx_HAL_Driver/Src/*.c"
  "User/RING_FIFO/src/*.c"
  "User/FLASH/src/*.c"
  "User/task_event/core/src/*.c"
  "User/task_event/user/src/*.c"
  "User/task_event/app/src/*.c"
  "User/libcrc/crc32_mpeg2/*.c"
  "User/other/src/*.c"
)
MESSAGE(STATUS "SOURCE_FILES: " ${SOURCE_FILES})

# 目标所需的源文件
add_executable(l431_bld.elf ${SOURCE_FILES} ${ASM_SOURCES})

# 目标所需宏定义
target_compile_definitions(l431_bld.elf PUBLIC
  USE_FULL_LL_DRIVER
  USE_HAL_DRIVER
  STM32L431xx
)

# 目标所需的头文件路径
target_include_directories(l431_bld.elf PUBLIC
  "Core/Inc"
  "Drivers/STM32L4xx_HAL_Driver/Inc"
  "Drivers/STM32L4xx_HAL_Driver/Inc/Legacy"
  "Drivers/CMSIS/Include"
  "Drivers/CMSIS/Device/ST/STM32L4xx/Include"
  "User/RING_FIFO/inc"
  "User/FLASH/inc"
  "User/task_event/core/inc"
  "User/task_event/user/inc"
  "User/libcrc/crc32_mpeg2"
  "User/other/inc"
)

# 目标所需链接库
target_link_libraries(l431_bld.elf -lc -lm -lnosys)

# 目标所需链接选项
target_link_options(l431_bld.elf PUBLIC
  -specs=nano.specs
  -T${LDSCRIPT}
  -Wl,-Map=${MAP_FILE},--cref
  -Wl,--gc-sections
)

# 目标编译后自定义指令
add_custom_command(
  TARGET l431_bld.elf POST_BUILD
  COMMAND pwd
  COMMAND ${TOOLCHAINS_PATH}${TOOLCHAINS_PREFIX}objcopy -O binary -S l431_bld.elf l431_bld.bin
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build
)

