#ifndef ARM_MODULE_PARAMS_H
#define ARM_MODULE_PARAMS_H

#include "pyro_typedef.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_module_base.h"

namespace pyro
{
// 声明
class Axis_control_t;
struct arm_context_t;
//================顶层机械臂模式================
enum class Arm_Ctr_Mode_t:uint8_t
{
    IDLE = 0,
    JOINT_DIRECT,
    CARTESIAN,
    AUTO_SEQUENCE,
    MAX
};

//================Tg1 子状态枚举（属于AUTO_SEQUENCE内部子状态）================
enum class Tg1State : uint8_t
{
    IDLE = 0,
    TRANSITION,
    RUNNING,
    PAUSED,
    FINISH,
    ERROR
};

// --------FSM前置声明--------
struct arm_fsm_ctx_t;
class arm_state_base_t;

// Tg1子状态前置声明
struct tg1_subctx_t;
class tg1_substate_base_t;

// 下发运动指令
struct arm_cmd_t : public cmd_base_t
{
    float axis_target_pos[7];
};

// 模块依赖
struct arm_deps_t
{
    struct motor_deps_t
    {
        dm_motor_drv_t* axis_motor[6]{};
        dm_motor_drv_t* end_motor{nullptr};
    };
    struct pid_deps_t
    {
        pid_t* axis_pos_pid[6]{};
        pid_t* axis_rot_pid[6]{};
        pid_t* end_pos_pid{nullptr};
        pid_t* end_rot_pid{nullptr};
    };
    motor_deps_t motor;
    pid_deps_t pid;
    float pos_range[6][2];
    float rot_range[6][2];
    float torque_range[6][2];
};

struct arm_data_ctx_t
{
    float axis_current_pos[7];
    float motor_current_pos[7];
    float motor_current_rot[7];
    float motor_current_torque[7];
};

//================ Tg1子状态机上下文（仅AUTO_SEQUENCE使用）================
struct tg1_subctx_t
{
    Tg1State current_state;
    uint32_t seq_step;        // 序列步数
    uint32_t seq_tick;        // 计时tick
    bool trigger_start;   // 启动触发信号
    bool trigger_pause;   // 暂停触发
    bool trigger_reset;   // 复位触发
};

//================顶层FSM上下文================
struct arm_fsm_ctx_t
{
    arm_cmd_t*          cmd{nullptr};
    arm_data_ctx_t*     data{nullptr};
    arm_context_t*      ctx{nullptr};
    arm_deps_t*         deps{nullptr};
    tg1_subctx_t*       tg1_ctx{nullptr}; // 指向子状态上下文
};

// 整机上下文 ModuleCtx
struct arm_context_t
{
    Arm_Ctr_Mode_t control_mode;
    arm_data_ctx_t data;
    Axis_control_t* axis_control[6]{};
    Axis_control_t* end_axis{nullptr};

    // 顶层FSM
    arm_fsm_ctx_t fsm_ctx;
    // Tg1子状态上下文（仅AUTO_SEQUENCE模式生效）
    tg1_subctx_t tg1_subctx;
};

// 模块参数包
struct arm_module_params_t
{
    using CmdType    = arm_cmd_t;
    using ModuleDeps = arm_deps_t;
    using ModuleCtx  = arm_context_t;
};

} // namespace pyro
#endif