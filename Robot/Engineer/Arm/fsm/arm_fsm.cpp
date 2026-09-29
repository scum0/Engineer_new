#include "arm_fsm.h"

using namespace pyro;


int fsm_ctx_t::segment_set_ptp(float duration, const position_t &start_position,
                              const position_t &end_position)
{
    if (duration <= 0.0f)
    {
        return -1;
    }

    const float inv_h2 = 1.0f / (duration * duration);
    const float inv_h3 = inv_h2 / duration;

    _cmd.duration = duration;
    for (int j = 0; j < AXIS_NUM; ++j)
    {
        const float delta = end_position[j] - start_position[j];
        _cmd.coeff[j][0] = start_position[j];
        _cmd.coeff[j][1] = 0.0f;
        _cmd.coeff[j][2] = 3.0f * delta * inv_h2;
        _cmd.coeff[j][3] = -2.0f * delta * inv_h3;
    }
    return 0;
}

void fsm_ctx_t::ptp_transition(float duration)
{
    position_t current_position = arm_module_ptr->_get_current_pos();
    segment_set_ptp(duration, current_position, _next_pos);
    set_command();
}

void fsm_ctx_t::set_command()
{
    _cmd_ctx.seg_cmd = _cmd;
    arm_module_ptr->set_command(_cmd_ctx);
}

void fsm_ctx_t::set_command(arm_cmd_t::segment_t* cmd)
{
    memcpy(&_cmd, cmd, sizeof(arm_cmd_t::segment_t));
    _cmd_ctx.seg_cmd = _cmd;
    arm_module_ptr->set_command(_cmd_ctx);
}

void fsm_ctx_t::set_command(float *pos)
{
    memcpy(_cmd.coeff[0].data(), pos, sizeof(position_t));
    _cmd_ctx.seg_cmd = _cmd;
    arm_module_ptr->set_command(_cmd_ctx);
}
