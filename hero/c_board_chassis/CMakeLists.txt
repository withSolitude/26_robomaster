cmake_minimum_required(VERSION 3.22)

# 工程名（同时作为 target 名）
set(CMAKE_PROJECT_NAME test_robowaker)

# 使用你原来放在 cmake/ 目录下的交叉编译工具链
# 一定要在 project() 之前 include
include("cmake/gcc-arm-none-eabi.cmake")

# 生成 compile_commands.json，方便 CLion/clangd 联动
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 默认构建类型
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# 开启 C / C++ / ASM 语言
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)

# C / C++ 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# 1. 先创建一个“空”的可执行 target
#    后面 CubeMX 子目录和 User_File 里的代码都会往这个 target 里塞
add_executable(${CMAKE_PROJECT_NAME}
        User_File/2_Device/Vofa/dvc_Vofa.cpp
        User_File/2_Device/Vofa/dvc_Vofa.h
        User_File/2_Device/Image_Transform/dvc_image_transform.cpp
        User_File/2_Device/Image_Transform/dvc_image_transform.h
        User_File/2_Device/Image_Transform/dvc_image_transform.cpp
        User_File/2_Device/Image_Transform/dvc_image_transform.h
        User_File/1_Middleware/1_Driver/UART/drv_uart.cpp
        User_File/3_Robot/Chassis/Chassis_Control.cpp
        User_File/3_Robot/Chassis/Chassis_Control.h
        User_File/4_Interaction/ita_robot.cpp
        User_File/4_Interaction/ita_robot.h
        User_File/5_Task/tsk_config_and_callback.cpp
        User_File/5_Task/tsk_config_and_callback.h
        User_File/3_Robot/Gimbal/Gimbal_Control.cpp
        User_File/3_Robot/Gimbal/Gimbal_Control.h
        User_File/3_Robot/Gimbal/Gimbal_Motor_Control.cpp
        User_File/3_Robot/Gimbal/Gimbal_Motor_Control.h
        User_File/1_Middleware/1_Driver/I2C/drv_i2c.cpp
        User_File/1_Middleware/1_Driver/I2C/drv_i2c.h
        User_File/2_Device/IMU/dvc_ahrs_cboard.cpp
        User_File/2_Device/IMU/dvc_ahrs_cboard.h
        User_File/3_Robot/Shoot/Shoot_Control.cpp
        User_File/3_Robot/Shoot/Shoot_Control.h
        User_File/3_Robot/Shoot/Shoot_Motor_Control.cpp
        User_File/3_Robot/Shoot/Shoot_Motor_Control.h
        User_File/1_Middleware/2_Algorithm/RLS/alg_rls.cpp
        User_File/1_Middleware/2_Algorithm/RLS/alg_rls.h
        User_File/2_Device/Vofa/dvc_Vofa_Tuner.cpp
        User_File/2_Device/Vofa/dvc_Vofa_Tuner.h
        User_File/3_Robot/Slave_Connection_With_Master/Slave_With_Master.cpp
        User_File/3_Robot/Slave_Connection_With_Master/Slave_With_Master.h)

# 2. 加载 CubeMX 生成的 HAL、启动文件、链接脚本等
#    cmake/stm32cubemx/CMakeLists.txt 里会用 target_sources/target_link_libraries
#    操作 ${CMAKE_PROJECT_NAME}，所以一定要在它之前 add_executable
add_subdirectory(cmake/stm32cubemx)

# 3. -------- 用户代码部分：User_File 下所有 .c / .cpp --------

# 收集 User_File 下面所有 .c / .cpp
file(GLOB_RECURSE USER_SOURCES
        "User_File/*.c"
        "User_File/*.cpp"
)

# 把这些源文件加到 test_robowaker 这个可执行文件里
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

# 头文件搜索路径（按你现在的目录结构一层层加上）
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        User_File
        User_File/1_Middleware
        User_File/1_Middleware/1_Driver
        User_File/1_Middleware/2_Algorithm
        User_File/2_Device
        User_File/use
)

# 如果用到了 CMSIS DSP（你原工程里就有这些宏），保持不变
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
)

# 4. 链接 CubeMX 生成的 stm32cubemx 库
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        # 如果以后再需要额外链接 CMSIS DSP 静态库，可以在这里加：
        # arm_cortexM4lf_math
)