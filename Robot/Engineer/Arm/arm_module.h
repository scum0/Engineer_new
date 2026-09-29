#ifndef PYRO_ROBOT_ARM_MODULE_H
#define PYRO_ROBOT_ARM_MODULE_H

#include "pyro_module_base.h"
#include <array>
#include "pyro_dm_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

inline constexpr uint8_t AXIS_NUM = 6;
inline constexpr uint8_t COEFF_NUM = 4;

using position_t = std::array<float, AXIS_NUM>;

struct arm_cmd_t final : public cmd_base_t
{
    // 夹爪只有两种状态，由按键控制开合
    struct segment_t
    {
        float duration;                          //持续时长,单位:秒
        std::array<std::array<float,COEFF_NUM>,AXIS_NUM> coeff;  //[关节][a, b, c, d]

        float start_Tick;
        bool is_initialized;

        static segment_t make_default()
        {
            segment_t s{};
            s.is_initialized = false;
            s.duration = 0.0f;
            s.start_Tick = 0.0f;
            return s;
        }

        void tick_init()
        {
            start_Tick = dwt_drv_t::get_timeline_ms();
            is_initialized = true;
        }

        [[nodiscard]] bool is_timeout()
        {
            float current_Tick = dwt_drv_t::get_timeline_ms();
            return (current_Tick - start_Tick)/1000.0f >= duration;
        }

        void output_val(float* out_val) const
        {
            float dt1 = (dwt_drv_t::get_timeline_ms()-start_Tick)/1000.0f;
            dt1 = std::fmin(dt1,duration);
            float dt2 = dt1 * dt1;
            float dt3 = dt2 * dt1;
            for (size_t j = 0; j < AXIS_NUM; ++j)
            {
                const auto& row = coeff[j];
                out_val[j] = row[0] + row[1]*dt1 + row[2]*dt2 + row[3]*dt3;
            }
        }
    };
    segment_t seg_cmd{};
};

enum class axis_limit_t : uint8_t
{
    CONSTRAINT,
    NO_CONSTRAINT
};

struct arm_deps_t
{
    struct  axis_deps_t
    {
        axis_deps_t(dm_motor_drv_t *motor, pid_t *pos_pid, pid_t *rot_pid);
        ~axis_deps_t() = default;

        void enable_constraint(){_axis_limit = axis_limit_t::CONSTRAINT;}
        void disable_constraint(){_axis_limit = axis_limit_t::NO_CONSTRAINT;}
        void set_feedback_pos_offset(float offset){_pos_offset = offset;}
        void set_limit(float lower_limit, float upper_limit);

        dm_motor_drv_t*  _motor{nullptr};
        pid_t*  _pos_pid{nullptr};
        pid_t*  _rot_pid{nullptr};
        axis_limit_t _axis_limit;
        float _lower_limit;
        float _upper_limit;
        float _pos_offset;
    };

    std::array<dm_motor_drv_t*,AXIS_NUM> motor{nullptr};
    std::array<pid_t*,AXIS_NUM> pos_pid{nullptr};
    std::array<pid_t*,AXIS_NUM> rot_pid{nullptr};
    std::array<axis_deps_t*,AXIS_NUM> axis{nullptr};

    dm_motor_drv_t*  end_motor{nullptr};
    pid_t*  end_pos_pid{nullptr};
    pid_t*  end_rot_pid{nullptr};
    axis_deps_t* end_gripper{nullptr};
};

struct data_ctx_t
{
    std::array<float,AXIS_NUM> axis_feedback_pos;        // 含offset
    std::array<float,AXIS_NUM> axis_feedback_rot;
    std::array<float,AXIS_NUM> axis_target_pos;

    float gripper_feedback_pos;
    float gripper_feedback_rot;
    float gripper_target_pos;

    bool gripper_state = false;     //夹爪开为1，闭为0
};

struct arm_ctx_t
{
    std::array<arm_deps_t::axis_deps_t*,AXIS_NUM> axis{nullptr};
    arm_deps_t::axis_deps_t* end_gripper{nullptr};

    data_ctx_t data;
    arm_cmd_t *cmd{nullptr};

    bool is_cmd_empty = true;
    bool is_buffer_empty = true;
    bool is_idle_state= true;  
};

struct arm_module_params_t
{
    using CmdType    = arm_cmd_t;
    using ModuleDeps = arm_deps_t;
    using ModuleCtx  = arm_ctx_t;
};

class arm_module_t final
    : public module_base_t<arm_module_t, arm_module_params_t>
{
    friend class module_base_t;
public:
    void _arm_disable();
    void _arm_enable();
    void _clear_pid();
    void _turn_gripper_state();

    position_t _get_current_pos();
    [[nodiscard]] bool _is_empty();
    void _set_idle_state(bool is_idle){_ctx.is_idle_state = is_idle;}
private:
    arm_module_t();
    ~arm_module_t() override = default;

    // 基类接口实现
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    void _pid_control();
};

};


#endif //PYRO_ROBOT_ARM_MODULE_H
