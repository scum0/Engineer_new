#include "arm_module.h"
#include "pyro_algo_common.h"

namespace pyro
{

template <>
void module_base_t<arm_module_t, arm_module_params_t>::_update_command()
{
    scoped_mutex_t lock(_mutex);
    if (_head != _tail)
    {
        _current_cmd = _cmd_buffer[_tail];

        if (_current_cmd.seg_cmd.is_initialized == false)
        {
            _current_cmd.seg_cmd.tick_init();
        }
        _ctx.is_cmd_empty = false;
        _current_cmd.seg_cmd.output_val(_ctx.data.target_pos.data());

        if (_current_cmd.seg_cmd.is_timeout() == true)
        {
            //这一段已经走完
            _tail = (_tail + 1) % CMD_BUF_SIZE;
        }
    }
    else
        _ctx.is_cmd_empty = true;
}

arm_module_t::arm_module_t() :
        module_base_t<arm_module_t, arm_module_params_t>("arm", 1024, 512)
{
    _ctx = {};
}

status_t arm_module_t::_init()
{
    _ctx.motor = _module_deps.motor;
    _ctx.pos_pid = _module_deps.pos_pid;
    _ctx.rot_pid = _module_deps.rot_pid;

    _ctx.axis = _module_deps.axis;

    return PYRO_OK;
}

void arm_module_t::_update_feedback()
{
    for (int i=0 ; i<JOINT_NUM ; i++)
    {
        _ctx.motor[i]->update_feedback();
        _ctx.data.feedback_pos[i] =
            _ctx.motor[i]->get_current_position() - _ctx.axis[i]->_pos_offset;
        _ctx.data.feedback_rot[i] = _ctx.motor[i]->get_current_rotate();
        _ctx.data.feedback_pos[i] = loop_fp32_constrain(_ctx.data.feedback_pos[i], -PI, PI);
    }
}

void arm_module_t::_fsm_execute()
{
    if (_current_cmd.mode == cmd_base_t::mode_t::ACTIVE || !_is_empty())
    {
        _pid_calculation();
        _send_motor_command();
    }
}

void arm_module_t::_pid_calculation()
{
    for (int i=0 ; i<JOINT_NUM ; i++)
    {
        if(_ctx.axis[i]->_axis_limit == axis_limit_t::NO_CONSTRAINT)
        {
            float diff = _ctx.data.target_pos[i] - _ctx.data.feedback_pos[i];
            if(diff > PI)
                _ctx.data.target_pos[i] -= 2 * PI;
            else if(diff < -PI)
                _ctx.data.target_pos[i] += 2 * PI;
        }
        // 有限位直接钳位
        else
        {
            if(_ctx.data.target_pos[i] < _ctx.axis[i]->_lower_limit)
                _ctx.data.target_pos[i] = _ctx.axis[i]->_lower_limit;
            else if(_ctx.data.target_pos[i] > _ctx.axis[i]->_upper_limit)
                _ctx.data.target_pos[i] = _ctx.axis[i]->_upper_limit;
        }
        float target_rot = _ctx.axis[i]->_pos_pid->calculate(_ctx.data.target_pos[i],_ctx.data.feedback_pos[i]);
        float target_tor = _ctx.axis[i]->_rot_pid->calculate(target_rot,_ctx.data.feedback_rot[i]);
        _ctx.data.out_torque[i] = target_tor;
    }
}

void arm_module_t::_send_motor_command()
{
    for (int i=0 ; i<JOINT_NUM ; i++)
        _ctx.motor[i]->send_torque(_ctx.data.out_torque[i]);
}

void arm_module_t::_arm_disable()
{
    _ctx.motor[0]->disable();
    _ctx.motor[1]->disable();
    _ctx.motor[2]->disable();
    _ctx.motor[3]->disable();
    _ctx.motor[4]->disable();
    _ctx.motor[5]->disable();
    _ctx.motor[6]->disable();
}

void arm_module_t::_arm_enable()
{
    _ctx.motor[0]->enable();
    _ctx.motor[1]->enable();
    _ctx.motor[2]->enable();
    _ctx.motor[3]->enable();
    _ctx.motor[4]->enable();
    _ctx.motor[5]->enable();
    _ctx.motor[6]->enable();
}

void arm_module_t::_clear_pid()
{
    for (int i=0 ; i<JOINT_NUM ; i++)
    {
        _ctx.pos_pid[i]->clear();
        _ctx.rot_pid[i]->clear();
    }
}

position_t arm_module_t::_get_current_pos()
{
    return _ctx.data.feedback_pos;
}

bool arm_module_t::_is_empty()
{
    return _ctx.is_cmd_empty;
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