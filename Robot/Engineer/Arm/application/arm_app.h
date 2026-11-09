#ifndef __PYRO_ARM_APP_H__
#define __PYRO_ARM_APP_H__

#include "pyro_task.h"

namespace pyro
{

/**
 * @brief 遥控器事件处理任务
 */
class arm_rc_task_t final : public task_base_t
{
  public:
    arm_rc_task_t();

  protected:
    status_t init() override;
    void run_loop() override;
};

/**
 * @brief 状态机执行任务
 */
class fsm_task_t final : public task_base_t
{
  public:
    fsm_task_t();

  protected:
    status_t init() override;
    void run_loop() override;
};

/**
 * @brief 获取遥控器事件处理任务单例
 */
arm_rc_task_t &arm_rc_task();

/**
 * @brief 获取状态机执行任务单例
 */
fsm_task_t &fsm_task();

} // namespace pyro

#endif // __PYRO_ARM_APP_H__
