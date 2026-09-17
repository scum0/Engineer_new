#include "arm_fsm.h"

using namespace pyro;

void arm_top_fsm_t::idle_state_t::enter(owner *owner)
{
    // 等待缓冲区命令执行完
    while (!arm_module_ptr->_is_empty())
        vTaskDelay(1);
}

void arm_top_fsm_t::idle_state_t::execute(owner *owner)
{
    arm_module_ptr->_arm_disable();
    vTaskSuspend(nullptr);      // 挂起状态机任务
}

void arm_top_fsm_t::idle_state_t::exit(owner *owner)
{
    arm_module_ptr->_arm_enable();
}
