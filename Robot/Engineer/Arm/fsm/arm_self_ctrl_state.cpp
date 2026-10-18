#include "arm_fsm.h"
#include "pyro_image_drv.h"

#include <cstring>

using namespace pyro;

constexpr float transition_duration = 1.0f;

// 一阶低通滤波系数，用于平滑图传链路下发的自控数据
static constexpr float alpha = 0.05f;

// 该模块的目的为从图传链路 (image_drv_t) 获取自控数据
class self_control_command_t
{
public:
    self_control_command_t() = default;
    ~self_control_command_t() = default;

    void update();
    float *get_self_control_command();

private:
    position_t _origin_command{};
    position_t _filter_command{};
};

void self_control_command_t::update()
{
    const uint8_t *rx_data = image_drv_t::get_instance().get_controller_rx_data();
    memcpy(_origin_command.data(), rx_data, sizeof(position_t));

    for (int i = 0; i < AXIS_NUM; i++)
    {
        _filter_command[i] =
            _filter_command[i] * (1.0f - alpha) + _origin_command[i] * alpha;
    }
}

float *self_control_command_t::get_self_control_command()
{
    return _filter_command.data();
}

static self_control_command_t self_control_command;

void arm_top_fsm_t::self_ctrl_state_t::enter(owner *owner)
{
    owner->is_self_ctrl = true;
    owner->set_next_pos(self_control_command.get_self_control_command());
    owner->ptp_transition(transition_duration);
}

void arm_top_fsm_t::self_ctrl_state_t::execute(owner *owner)
{
    self_control_command.update();
    owner->set_command(self_control_command.get_self_control_command());
}

void arm_top_fsm_t::self_ctrl_state_t::exit(owner *owner)
{
    owner->is_self_ctrl = false;
}
