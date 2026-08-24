/**
 * @file arm_app.cpp
 * @brief 机械臂外部应用入口，与module业务解耦
 * @template-divide 模块arm_module_t仅负责控制逻辑；本文件负责：
 *        1.C语言兼容初始化入口，供RTOS启动调用
 *        2.外部指令接收线程，构造arm_cmd_t调用set_command下发
 *        3.硬件依赖deps统一初始化，调用模块configure注入
 */
#include "arm_control_execute.h"
#include "arm_module_params.h"
#include "cmsis_os.h"

namespace pyro
{
static arm_deps_t *arm_deps_ptr = nullptr;
static arm_cmd_t *arm_cmd_ptr   = nullptr;

static void arm_deps_init()
{
    arm_deps_ptr = new arm_deps_t();
}

// 外部指令接收线程
static void arm_rxcmd_task(void *args)
{
    while(1)
    {
        // 此处替换上层通信/轨迹规划赋值逻辑,把数据拷贝一份送入环形缓冲区
        // arm_module_t::instance()->set_command(*arm_cmd_ptr);
        vTaskDelay(1);
    }
}

// 示例
// static void arm_rxcmd_task(void *args)
// {
//     arm_cmd_t& cmd_ref = *arm_cmd_ptr;

//     while(1)
//     {
//         // 1. 填充各关节目标角度
//         cmd_ref.axis_target_pos[0] = 0.0f;
//         cmd_ref.axis_target_pos[1] = 0.0f;
//         cmd_ref.axis_target_pos[2] = 0.3f;
//         cmd_ref.axis_target_pos[3] = 0.0f;
//         cmd_ref.axis_target_pos[4] = 0.0f;
//         cmd_ref.axis_target_pos[5] = 0.0f;
//         cmd_ref.axis_target_pos[6] = 0.0f; 
//         
//         //结构体可以直接相等赋值（前提结构体内不能有指针与函数）

//         // 2. 设置指令模式：主动控制
//         cmd_ref.mode = cmd_base_t::mode_t::ACTIVE;

//         // 3. 调用模板接口，线程安全送入环形缓冲
//         arm_module_t::instance()->set_command(cmd_ref);

//         // 周期1ms读取一次上层数据
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }
} // namespace pyro

extern "C" void engineer_arm_init(void *args)
{
    vTaskDelay(10);
    pyro::arm_deps_init();
    pyro::arm_cmd_ptr = new pyro::arm_cmd_t();
    auto arm_mod = pyro::arm_module_t::instance();
    
    arm_mod->configure(*pyro::arm_deps_ptr);
    // 启动模板内置定时控制任务，自动执行_init/_update_feedback/_fsm_execute
    arm_mod->start();
    // 指令接收线程
    //原来自主写的 engineer_arm_mission 的执行线程，整体并入 module_base 模板内部
    xTaskCreate(pyro::arm_rxcmd_task, "arm_rxcmd", 512, nullptr, configMAX_PRIORITIES - 2, nullptr);
    vTaskDelete(nullptr);
}
