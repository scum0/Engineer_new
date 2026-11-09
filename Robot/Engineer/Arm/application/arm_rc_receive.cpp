/**
 * @file arm_remote_control.cpp
 * @brief 遥控器输入到机械臂板底盘控制量的映射
 */

#include "arm_rc_receive.h"

#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_rc_core.h"
#include "pyro_rw_lock.h"
#include "pyro_vt03_rc_drv.h"

namespace pyro
{
constexpr float AXIS_SCALE = 3.0f;

arm_rc_t get_arm_rc()
{
    arm_rc_t command{};

#ifdef VT03_UART
    if (vt03_drv_t::instance().check_online())
    {
        read_scope_lock lock(vt03_drv_t::instance().get_lock());
        const auto &vrc = rc_drv_t::read();

        command.vx = vrc.axes.ly * AXIS_SCALE;
        command.vy = vrc.axes.lx * AXIS_SCALE;
        command.wz = vrc.axes.rx * AXIS_SCALE;
        command.enable =
            (vrc.switches.gear.current_pos != sw_pos_t::DOWN) ? 1 : 0;
        command.rc_online = true;
    }
#endif

#ifdef DR16_UART
    if (!command.rc_online && dr16_drv_t::instance().check_online())
    {
        read_scope_lock lock(dr16_drv_t::instance().get_lock());
        const auto &vrc = rc_drv_t::read();

        command.vx = vrc.axes.ly * AXIS_SCALE;
        command.vy = vrc.axes.lx * AXIS_SCALE;
        command.wz = vrc.axes.rx * AXIS_SCALE;

        // 左拨杆向下 → 失能，其余档位使能
        command.enable =
            (vrc.switches.left.current_pos != sw_pos_t::DOWN) ? 1 : 0;

        // 左拨杆向上 → 收起(2)，否则放下(1)
        command.lift_auto =
            (vrc.switches.left.current_pos == sw_pos_t::UP) ? 2 : 1;
        command.rc_online = true;
    }
#endif

    return command;
}

} // namespace pyro
