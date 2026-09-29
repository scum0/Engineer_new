#include "arm_module.h"
#include "pyro_algo_common.h"

namespace pyro
{
float axis_current_pos[6]{};
float gripper_target_pos = 0.0f;
float gripper_feedback_pos = 0.0f;

constexpr float gripper_open_pos = -0.4f;
constexpr float gripper_close_pos = -1.2f;

template <>
void module_base_t<arm_module_t, arm_module_params_t>::_update_command()
{
    scoped_mutex_t lock(_mutex);

    // 模块空闲，且队列有排队指令 → 取出新指令开始执行
    if (_ctx.is_cmd_empty && (_head != _tail))
    {
        _current_cmd = _cmd_buffer[_tail];
        _current_cmd.seg_cmd.tick_init();
        _ctx.is_cmd_empty = false;
    }

    // 如果正在执行轨迹：无条件跑，与队列是否为空无关
    if (!_ctx.is_cmd_empty)
    {
        _current_cmd.seg_cmd.output_val(_ctx.data.axis_target_pos.data());
        if (_current_cmd.seg_cmd.is_timeout())
        {
            _tail = (_tail + 1) % CMD_BUF_SIZE;
            _ctx.is_cmd_empty = true; // 当前段跑完，标记模块变为空闲
        }
    }
    // 队列空闲,队列为空且现在没有指令在执行
    _ctx.is_buffer_empty = (_head == _tail) && (_ctx.is_cmd_empty); 
}

arm_module_t::arm_module_t() :
        module_base_t<arm_module_t, arm_module_params_t>("arm", 1024, 512)
{
    _ctx = {};
}

status_t arm_module_t::_init()
{
    _ctx.axis = _module_deps.axis;
    _ctx.end_gripper = _module_deps.end_gripper;

    return PYRO_OK;
}

void arm_module_t::_update_feedback()
{
    for (int i=0 ; i<AXIS_NUM ; i++)
    {
        _ctx.axis[i]->_motor->update_feedback();
        _ctx.data.axis_feedback_pos[i] =
            _ctx.axis[i]->_motor->get_current_position() - _ctx.axis[i]->_pos_offset;
        _ctx.data.axis_feedback_rot[i] = _ctx.axis[i]->_motor->get_current_rotate();
        _ctx.data.axis_feedback_pos[i] = loop_fp32_constrain(_ctx.data.axis_feedback_pos[i], -PI, PI);
        axis_current_pos[i] = _ctx.data.axis_feedback_pos[i];
    }

    _ctx.end_gripper->_motor->update_feedback();
    _ctx.data.gripper_feedback_pos = _ctx.end_gripper->_motor->get_current_position()
                                                         - _ctx.end_gripper->_pos_offset;
    _ctx.data.gripper_feedback_rot = _ctx.end_gripper->_motor->get_current_rotate();
    gripper_feedback_pos = _ctx.data.gripper_feedback_pos;
}

void arm_module_t::_fsm_execute()
{
    if (_ctx.is_idle_state == false)
    {
        _pid_control();
    }
    else
    {
        // 发送控制帧才能读到反馈
        for (int i=0 ; i<AXIS_NUM ; i++)
            _ctx.axis[i]->_motor->send_torque(0.0f);

        _ctx.end_gripper->_motor->send_torque(0.0f);
    }
}

void arm_module_t::_pid_control()
{
    for (int i=0 ; i<AXIS_NUM ; i++)
    {
        // 没有限位的关节，目标位置与反馈位置差值大于 PI 时，进行环绕处理
        if(_ctx.axis[i]->_axis_limit == axis_limit_t::NO_CONSTRAINT)
        {
            float diff = _ctx.data.axis_target_pos[i] - _ctx.data.axis_feedback_pos[i];
            if(diff > PI)
                _ctx.data.axis_target_pos[i] -= 2 * PI;
            else if(diff < -PI)
                _ctx.data.axis_target_pos[i] += 2 * PI;
        }
        // 有限位直接钳位
        else
        {
            if(_ctx.data.axis_target_pos[i] < _ctx.axis[i]->_lower_limit)
                _ctx.data.axis_target_pos[i] = _ctx.axis[i]->_lower_limit;
            else if(_ctx.data.axis_target_pos[i] > _ctx.axis[i]->_upper_limit)
                _ctx.data.axis_target_pos[i] = _ctx.axis[i]->_upper_limit;
        }
        float target_rot = _ctx.axis[i]->_pos_pid->calculate(_ctx.data.axis_target_pos[i],_ctx.data.axis_feedback_pos[i]);
        float target_tor = _ctx.axis[i]->_rot_pid->calculate(target_rot,_ctx.data.axis_feedback_rot[i]);
        _ctx.axis[i]->_motor->send_torque(target_tor);
    }

    // 夹爪控制
    float target_rot = _ctx.end_gripper->_pos_pid->calculate(_ctx.data.gripper_target_pos,_ctx.data.gripper_feedback_pos);
    float target_tor = _ctx.end_gripper->_rot_pid->calculate(target_rot,_ctx.data.gripper_feedback_rot);
    _ctx.end_gripper->_motor->send_torque(target_tor);
}

void arm_module_t::_arm_disable()
{
    for (int i=0 ; i<AXIS_NUM ; i++)
        _ctx.axis[i]->_motor->disable();

    _ctx.end_gripper->_motor->disable();
}

void arm_module_t::_arm_enable()
{
    for (int i=0 ; i<AXIS_NUM ; i++)
        _ctx.axis[i]->_motor->enable();

    _ctx.end_gripper->_motor->enable();
}

void arm_module_t::_clear_pid()
{
    for (int i=0 ; i<AXIS_NUM ; i++)
    {
        _ctx.axis[i]->_pos_pid->clear();
        _ctx.axis[i]->_rot_pid->clear();
    }
    _ctx.end_gripper->_pos_pid->clear();
    _ctx.end_gripper->_rot_pid->clear();
}

void arm_module_t::_turn_gripper_state()
{
    _ctx.data.gripper_state = !_ctx.data.gripper_state;
    if(_ctx.data.gripper_state == true)
        _ctx.data.gripper_target_pos = gripper_open_pos;
    else
        _ctx.data.gripper_target_pos = gripper_close_pos;
}

position_t arm_module_t::_get_current_pos()
{
    return _ctx.data.axis_feedback_pos;
}

bool arm_module_t::_is_empty()
{
    return _ctx.is_buffer_empty;
}

arm_deps_t::axis_deps_t::axis_deps_t(dm_motor_drv_t *motor, pid_t *pos_pid, pid_t *rot_pid)
    : _motor(motor), _pos_pid(pos_pid), _rot_pid(rot_pid)
{
}

void arm_deps_t::axis_deps_t::set_limit(float lower_limit, float upper_limit)
{
    _axis_limit = axis_limit_t::CONSTRAINT;
    _lower_limit = lower_limit;
    _upper_limit = upper_limit;
}

} // namespace pyro