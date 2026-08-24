#include "arm_control_execute.h"
#include "arm_module_params.h"
#include "pyro_typedef.h"
#include "pyro_algo_common.h"
#include <cstring>
#include <algorithm>
#include <cmath>

namespace pyro
{

namespace
{
// 机械臂模块栈大小、任务优先级、等待电机上电时间
constexpr uint32_t ARM_INIT_STACK = 1024U;
constexpr uint32_t ARM_LOOP_STACK = 512U;
constexpr TickType_t MOTOR_WAIT_TICKS = pdMS_TO_TICKS(1500U);

// 电机配置：tx_id, rx_id, can外设
struct MotorCfg_t
{
    uint8_t tx;
    uint8_t rx;
    bsp_can::which_can can;
};
constexpr std::array<MotorCfg_t,7> motor_cfg {{
    {0x2, 0x1, bsp_can::can1},
    {0x4, 0x3, bsp_can::can1},
    {0x6, 0x5, bsp_can::can3},
    {0x8, 0x7, bsp_can::can3},
    {0xa, 0x9, bsp_can::can2},
    {0xc, 0xb, bsp_can::can2},
    {0xe, 0xd, bsp_can::can2},
}};

// PID参数 Kp Ki Kd Kff out_max
struct PidCfg_t
{
    float kp;
    float ki;
    float kd;
    float kff;
    float outmax;
};
// 6轴：位置PID，旋转PID
constexpr std::array<std::pair<PidCfg_t,PidCfg_t>,6> motor_pid {{
    {{10,0.0,0.0,0.0,52}, {8.8,12.0,0.0,0.0,27}},
    {{20,0,0.0,20.0,150}, {20,0,0.0,20.0,150}},
    {{15,0.0,0.0,0.0,160}, {11,0.2,0.0,5.0,40}},
    {{15.7,0,0.0,6,200}, {1.0,0.2,0.001,3,7}},
    {{10.3,0.1,0.0,20,200}, {1.0,0.01,0.00,4,7}},
    {{9,0.0,0.0,0.0,200}, {0.8,0.1,0.0,1,7}},
}};
constexpr PidCfg_t end_pos_pid = {9,0.0,0.0,0,200};
constexpr PidCfg_t end_rot_pid = {0.8,0.0,0.0,0,7};

// 逻辑限位、offset
constexpr std::array<std::pair<float,float>,6> g_axis_logic_limit {{
    {-2.1599f, PI},
    {-0.3f, 1.0f},
    {-0.3f, 1.35f},
    {0.0f,0.0f}, // R4 disable_constraint
    {-0.5f*PI, 0.5f*PI},
    {-0.5f*PI, 0.5f*PI}
}};
constexpr std::array<float,6> g_axis_offset {{
    0.65045f,
    -2.3948f,
    -0.328f,
    2.34739f,
    2.46375f,
    2.74246f
}};
constexpr float end_low  = -PI;
constexpr float end_high = 0.0f;
constexpr float end_offset = 1.8587f;

// 硬件range
constexpr std::array<std::pair<float,float>,6> pos_range {{ {-PI,PI},{-PI,PI},{-PI,PI},{-PI,PI},{-PI,PI},{-PI,PI} }};
constexpr std::array<std::pair<float,float>,6> rot_range  {{ {-52,52},{-150,150},{-160,160},{-200,200},{-200,200},{-200,200} }};
constexpr std::array<std::pair<float,float>,6> tor_range {{ {-27,27},{-150,150},{-40,40},{-7,7},{-7,7},{-7,7} }};

} // namespace anonymous


// Axis_control_t 单轴控制器实现
Axis_control_t::Axis_control_t(motor_base_t *motor, pid_t *pos_pid, pid_t *rot_pid)
    : _motor(motor), _pos_pid(pos_pid), _rot_pid(rot_pid)
{
    // 增加空指针检测，调试阶段快速捕获bug
    assert(_motor != nullptr);
    assert(_pos_pid != nullptr);
    assert(_rot_pid != nullptr);

    _target_position = 0.0f;
    _target_rotate   = 0.0f;
    _axis_limit      = CONSTRAINT;
}

Axis_control_t::~Axis_control_t()
{
}

void Axis_control_t::set_target(float position)
{
    // 无限位关节角度循环归一化 [-PI, PI]
    if(_axis_limit == NO_CONSTRAINT)
    {
         position = loop_fp32_constrain(position, -PI, PI);
    }
    // 有限位直接钳位
    else
    {
        if(position < _lower_limit)
            position = _lower_limit;
        else if(position > _upper_limit)
            position = _upper_limit;
    }
    _target_position = position;
}

void Axis_control_t::update_feedback()
{
    _motor->update_feedback();
    _feedback_position = _motor->get_current_position() - _pos_offset;
    _feedback_rotate   = _motor->get_current_rotate();

    // 反馈角度多圈归一 [-PI, PI]
    _feedback_position = loop_fp32_constrain(_feedback_position, -PI, PI);
}

void Axis_control_t::pid_control()
{
    // 无限位关节误差跨圈修正，避免PID震荡
    if(_axis_limit == NO_CONSTRAINT)
    {
        float diff = _target_position - _feedback_position;
        if(diff > PI)
            _target_position -= 2 * PI;
        else if(diff < -PI)
            _target_position += 2 * PI;
    }
    float rot = _pos_pid->calculate(_target_position, _feedback_position);
    float torque = _rot_pid->calculate(rot, _feedback_rotate);
    _motor->send_torque(torque);
}

// 机械臂模块构造函数
arm_module_t::arm_module_t()
    : module_base_t("arm_ctrl", ARM_INIT_STACK, ARM_LOOP_STACK, task_base_t::priority_t::HIGH)
{
}

arm_module_t::~arm_module_t()
{
    // 析构，预留资源释放位置
}

const arm_context_t& arm_module_t::get_ctx() const
{
    return _ctx;
}

// 模块初始化
status_t arm_module_t::_init()
{
    auto& deps = _module_deps;
    auto& ctx  = _ctx;

    // 1. 创建底层电机驱动实例
    for(int i = 0; i < 6; i++)
    {
        const auto& cfg = motor_cfg[(size_t)i];
        deps.motor.axis_motor[i] = new dm_motor_drv_t(cfg.tx, cfg.rx, cfg.can);
    }
    {
        const auto& end_cfg = motor_cfg[6];
        deps.motor.end_motor = new dm_motor_drv_t(end_cfg.tx, end_cfg.rx, end_cfg.can);
    }

    // 2. 创建各轴位置/扭矩PID实例
    for(int i=0;i<6;i++)
    {
        const auto& [pos,rot] = motor_pid[(size_t)i];
        deps.pid.axis_pos_pid[i] = new pid_t(pos.kp, pos.ki, pos.kd, pos.kff, pos.outmax);
        deps.pid.axis_rot_pid[i] = new pid_t(rot.kp, rot.ki, rot.kd, rot.kff, rot.outmax);
    }
    deps.pid.end_pos_pid = new pid_t(end_pos_pid.kp, end_pos_pid.ki, end_pos_pid.kd, end_pos_pid.kff, end_pos_pid.outmax);
    deps.pid.end_rot_pid = new pid_t(end_rot_pid.kp, end_rot_pid.ki, end_rot_pid.kd, end_rot_pid.kff, end_rot_pid.outmax);

    // 3. 写入硬件限位参数至deps；移除memcpy，循环赋值，避免内存拷贝越界风险
    for(int i=0;i<6;i++)
    {
        deps.pos_range[i][0] = pos_range[(size_t)i].first;
        deps.pos_range[i][1] = pos_range[(size_t)i].second;

        deps.rot_range[i][0] = rot_range[(size_t)i].first;
        deps.rot_range[i][1] = rot_range[(size_t)i].second;

        deps.torque_range[i][0] = tor_range[(size_t)i].first;
        deps.torque_range[i][1] = tor_range[(size_t)i].second;
    }

    // 4. 底层电机配置硬件极限
    for(int i = 0; i < 6; i++)
    {
        deps.motor.axis_motor[i]->set_position_range(deps.pos_range[i][0], deps.pos_range[i][1]);
        deps.motor.axis_motor[i]->set_rotate_range(deps.rot_range[i][0], deps.rot_range[i][1]);
        deps.motor.axis_motor[i]->set_torque_range(deps.torque_range[i][0], deps.torque_range[i][1]);
    }
    deps.motor.end_motor->set_position_range(-PI, PI);
    deps.motor.end_motor->set_rotate_range(-200, 200);
    deps.motor.end_motor->set_torque_range(-7, 7);

    // 5. 创建单轴控制器，存入ModuleCtx指针数组
    for(int i = 0; i < 6; i++)
    {
        ctx.axis_control[i] = new Axis_control_t(deps.motor.axis_motor[i], deps.pid.axis_pos_pid[i], deps.pid.axis_rot_pid[i]);
    }
    ctx.end_axis = new Axis_control_t(deps.motor.end_motor, deps.pid.end_pos_pid, deps.pid.end_rot_pid);

    // 6. 各轴逻辑限位、安装偏移配置；数值完全和原版保持
    ctx.axis_control[0]->set_limit(-2.1599, PI);
    ctx.axis_control[1]->set_limit(-0.3, 1);
    ctx.axis_control[2]->set_limit(-0.3, 1.35);
    ctx.axis_control[3]->disable_constraint(); // R4无限位旋转关节
    ctx.axis_control[4]->set_limit(-0.5f*PI, 0.5f*PI);
    ctx.axis_control[5]->set_limit(-0.5f*PI, 0.5f*PI);
    ctx.end_axis->set_limit(end_low, end_high);

    ctx.axis_control[0]->set_feedback_pos_offset(g_axis_offset[0]);
    ctx.axis_control[1]->set_feedback_pos_offset(g_axis_offset[1]);
    ctx.axis_control[2]->set_feedback_pos_offset(g_axis_offset[2]);
    ctx.axis_control[3]->set_feedback_pos_offset(g_axis_offset[3]);
    ctx.axis_control[4]->set_feedback_pos_offset(g_axis_offset[4]);
    ctx.axis_control[5]->set_feedback_pos_offset(g_axis_offset[5]);
    ctx.end_axis->set_feedback_pos_offset(end_offset);

    // 延时等待电机上电稳定，使能所有驱动
    vTaskDelay(MOTOR_WAIT_TICKS);
    for(int i = 0; i < 6; i++)
    {
        deps.motor.axis_motor[i]->enable();
        vTaskDelay(1);
    }
    deps.motor.end_motor->enable();
    vTaskDelay(1);

    // 初始控制模式置空闲零力
    ctx.control_mode = Arm_Ctr_Mode_t::IDLE;
    return status_t::PYRO_OK;
}

// 模板周期反馈回调
void arm_module_t::_update_feedback()
{
    auto& ctx  = _ctx;
    auto& deps = _module_deps;
    auto& data = ctx.data;
    // 6个关节反馈更新
    for(int i = 0; i < 6; i++)
    {
        ctx.axis_control[i]->update_feedback();
        ctx.axis_control[i]->get_current_position(data.axis_current_pos[i]);
        data.motor_current_pos[i]    = deps.motor.axis_motor[i]->get_current_position();
        data.motor_current_rot[i]    = deps.motor.axis_motor[i]->get_current_rotate();
        data.motor_current_torque[i] = deps.motor.axis_motor[i]->get_current_torque();
    }
    // 夹爪轴反馈更新
    ctx.end_axis->update_feedback();
    ctx.end_axis->get_current_position(data.axis_current_pos[6]);
    data.motor_current_pos[6]    = deps.motor.end_motor->get_current_position();
    data.motor_current_rot[6]    = deps.motor.end_motor->get_current_rotate();
    data.motor_current_torque[6] = deps.motor.end_motor->get_current_torque();
}

// 执行所有轴pid_control调用
void arm_module_t::_axis_control()
{
    auto& ctx  = _ctx;
    for(int i = 0; i < 6; i++)
    {
        ctx.axis_control[i]->pid_control();
    }
    ctx.end_axis->pid_control();
}

// 零力模式
void arm_module_t::_zero_force()
{
    auto& deps = _module_deps;
    for(int i = 0; i < 6; i++)
    {
        deps.motor.axis_motor[i]->send_torque(0.0f);
    }
    deps.motor.end_motor->send_torque(0.0f);
}

/**
 * @brief 模板状态机回调，每1ms自动执行
 * @template-flow 模板内部先执行_update_command读取环形缓冲区_cmd，再进入本函数
 * @logic 解析指令目标，判断合法性，切换空闲/闭环模式，调用细分控制函数
 */
void arm_module_t::_fsm_execute()
{
    auto& ctx  = _ctx;
    // 把当前运行时数据绑定到FSM上下文
    _arm_fsm_ctx.cmd        = &_current_cmd;
    _arm_fsm_ctx.data       = &ctx.data;
    _arm_fsm_ctx.ctx        = &ctx;
    _arm_fsm_ctx.deps       = &_module_deps;
    _arm_fsm_ctx.tg1_ctx    = &ctx.tg1_subctx;
    // 交给FSM去跑状态机（由各个state::execute内部做业务逻辑）
    _arm_fsm.execute(&_arm_fsm_ctx);
}

} // namespace pyro