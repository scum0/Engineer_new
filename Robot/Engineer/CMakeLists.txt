# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 ARM_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "ARM_BOARD")
#set(BOARD "CHASSIS_BOARD")


# =========================================================================
# 2. 声明 Engineer 目标文件库（OBJECT库，仅编译目标文件不链接）
# =========================================================================
add_library(Engineer OBJECT
)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Engineer PRIVATE
        BOARD=${BOARD}
)

# ========== 统一编译选项，解决 ARM/Thumb 指令集不匹配 ==========
target_compile_options(Engineer PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)

# ===================== 两个板型共用的基础源文件（目前所有的源文件） =====================
set(BASE_SOURCES
)

# ===================== ARM_BOARD 扩展源文件 =====================
set(ARM_EXTRA_SOURCES
        Arm/pyro_mission_planer.cpp
        Arm/pyro_init_thread.cpp
        Arm/fsm/arm_fsm.cpp
        Arm/fsm/arm_self_ctrl_state.cpp
        Arm/fsm/arm_idle_state.cpp
        Arm/fsm/arm_normal_state.cpp
        Arm/arm_module.cpp
        Application/arm_app.cpp
)

# ===================== CHASSIS_BOARD 扩展源文件 =====================
set(CHASSIS_EXTRA_SOURCES
        # 未来可添加 CHASSIS 独有的文件
)

# ===================== 组合最终源文件列表 =====================
if (BOARD STREQUAL "ARM_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${ARM_EXTRA_SOURCES}
    )
    # 针对 ARM_BOARD 的特殊宏定义（作用于 PYRo 目标）
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
    )
    # 为 Engineer 自身添加宏定义
    target_compile_definitions(Engineer PRIVATE
            DR16_UART=PYRO_UART5
            SELF_CTRL_UART=PYRO_UART10
            DEBUG_UART=PYRO_UART1
            INTERBOARD_UART=PYRO_UART7
    )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${CHASSIS_EXTRA_SOURCES})
    # 针对 CHASSIS_BOARD 的特殊宏定义（作用于 PYRo 目标）

    # 为 Engineer 自身添加宏定义

else()
    message(FATAL_ERROR "Unsupported BOARD type: ${BOARD}. Choose ARM_BOARD or CHASSIS_BOARD.")
endif()

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Arm
        ${CMAKE_CURRENT_SOURCE_DIR}/Arm/Kinematics
)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)