#ifndef __PYRO_ARM_REMOTE_CONTROL_H__
#define __PYRO_ARM_REMOTE_CONTROL_H__

#include <cstdint>

namespace pyro
{

/**
 * @brief 遥控器输入映射后的底盘控制量
 */
struct arm_rc_t
{
    bool rc_online{false};
    uint16_t enable{0};
    float vx{0.0f};
    float vy{0.0f};
    float wz{0.0f};
    uint16_t lift_auto{0};
};

/**
 * @brief 读取当前遥控器并生成底盘控制量快照
 */
arm_rc_t get_arm_rc();

} // namespace pyro

#endif // __PYRO_ARM_REMOTE_CONTROL_H__
