#include "arm_fsm.h"

using namespace pyro;

void arm_top_fsm_t::idle_state_t::enter(owner *owner)
{
    // 等待缓冲区命令执行完
    owner->ptp_transition(resets{});
    while (!arm_module_ptr->_is_empty())
        vTaskDelay(1);
    arm_module_ptr->_set_idle_state(true);
}

void arm_top_fsm_t::idle_state_t::execute(owner *owner)
{
    arm_module_ptr->_arm_disable();
    vTaskSuspend(nullptr);      // 挂起状态机任务
}

void arm_top_fsm_t::idle_state_t::exit(owner *owner)
{
    arm_module_ptr->_set_idle_state(false);
    arm_module_ptr->_arm_enable();
}
