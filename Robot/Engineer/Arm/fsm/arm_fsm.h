#ifndef PYRO_ROBOT_ARM_FSM_H
#define PYRO_ROBOT_ARM_FSM_H

#include "arm_module.h"
#include "pyro_core_fsm.h"

// HFSM 状态机定义
// 主要功能为写命令

namespace pyro
{
extern arm_module_t *arm_module_ptr;

// 标签分发，不用实现
struct resets{};
struct normal{};

using position_t = std::array<float, AXIS_NUM>;
using axis_pos_t = std::array<float, 6>;
struct motion_param_t
{
    float duration{};
    position_t target_pos{};
};

inline constexpr position_t normal_position = {0.0f, 0.668f, 0.827f, 0.0f, 0.1241f, -0.094f};
inline constexpr position_t reset_position = {0.0f, 1.0905f, 1.523f, -0.018f, 0.2144f, 0.06f};

template<typename Tag>
struct motion_transition;

template<>
struct motion_transition<resets>
{
    static constexpr motion_param_t get()   {return{2.0f,reset_position};}
};

template<>
struct motion_transition<normal>
{
    static constexpr motion_param_t get()   {return{1.0f,normal_position};}
};


class fsm_ctx_t
{
public:
    template<typename TagT>
    void ptp_transition(TagT )
    {
        position_t current_position = arm_module_ptr->_get_current_pos();
        constexpr auto param = motion_transition<TagT>::get();

        segment_set_ptp(param.duration, current_position, param.target_pos);
        set_command();
    }

    void ptp_transition(float duration);

    void set_command();
    void set_command(arm_cmd_t::segment_t* cmd);
    void set_command(float* pos);

    void set_next_pos(float* pos){memcpy(_next_pos.data(), pos, sizeof(position_t));}

    bool is_self_ctrl = false;
    bool is_action = false;

private:
    arm_cmd_t _cmd_ctx{};
    arm_cmd_t::segment_t _cmd = arm_cmd_t::segment_t::make_default();
    position_t _next_pos{};

    int segment_set_ptp(float duration, const position_t &start_position,const position_t &end_position);

};

using owner = fsm_ctx_t;

struct arm_top_fsm_t : fsm_t<owner>
{
    struct idle_state_t : state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct normal_state_t : state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct self_ctrl_state_t : state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    idle_state_t      idle_state;
    normal_state_t    normal_state;
    self_ctrl_state_t self_ctrl_state;
};

}

#endif //PYRO_ROBOT_ARM_FSM_H