#ifndef ARM_CONTROL_EXECUTE_H
#define ARM_CONTROL_EXECUTE_H
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_module_base.h"
#include "cmsis_os.h"
#include "arm_module_params.h"
#include "arm_fsm.h"

#include <array>
#include <cassert>

namespace pyro
{

// 枚举名字完全保留原样，不改名
enum axis_limit_t
{
    CONSTRAINT,
    NO_CONSTRAINT
};

/**
 * @class Axis_control_t
 * @brief 单轴独立闭环控制器
 * @single-duty 封装单轴限位、偏移补偿、位置环+扭矩双PID、反馈更新逻辑
 */
class Axis_control_t
{
public:
    Axis_control_t(motor_base_t *motor, pid_t *pos_pid, pid_t *rot_pid);
    ~Axis_control_t();

    void set_target(float position);
    void disable_constraint(){_axis_limit = NO_CONSTRAINT;}
    void set_feedback_pos_offset(float offset){_pos_offset = offset;}
    void set_limit(float lower_limit, float upper_limit)
    {
        _axis_limit = CONSTRAINT;
        _lower_limit = lower_limit;
        _upper_limit = upper_limit;
    }

    void update_feedback();
    void pid_control();
    void get_current_position(float &position){position = _feedback_position;}

private:
    motor_base_t * const _motor;
    pid_t * const _pos_pid;
    pid_t * const _rot_pid;

    axis_limit_t _axis_limit;

    float _target_position;
    // float _target_rotate; // 僵尸变量，注释保留，不删除成员位置，避免内存布局变化
    float _target_rotate [[maybe_unused]] {0.0f};

    float _feedback_position;
    float _feedback_rotate;
    float _upper_limit;
    float _lower_limit;
    float _pos_offset;
};

/**
 * @class arm_module_t
 * @brief 机械臂顶层模块类，通用模板module_base_t业务实现类
 * @template-inherit CRTP继承语法：module_base_t<当前业务类, 参数包>
 * @template-interface 必须实现3个纯虚函数，由模板内部1ms定时循环自动调度：
 *        1._init() 上电一次性初始化（模板启动时执行）
 *        2._update_feedback() 每周期采集所有传感器反馈
 *        3._fsm_execute() 每周期执行控制逻辑、状态分支
 */
class arm_module_t final
    : public module_base_t<arm_module_t, arm_module_params_t>
{
    friend module_base_t<arm_module_t, arm_module_params_t>;
public:
    arm_module_t(const arm_module_t&) = delete;
    arm_module_t& operator=(const arm_module_t&) = delete;
    static arm_module_t* get_instance() { return instance(); }
    const arm_context_t& get_ctx() const;

private:
    arm_module_t();
    ~arm_module_t() override;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    void _axis_control();
    void _zero_force();

    fsm_t<arm_fsm_ctx_t> _arm_fsm;
    arm_fsm_ctx_t            _arm_fsm_ctx;
};

} // namespace pyro
#endif