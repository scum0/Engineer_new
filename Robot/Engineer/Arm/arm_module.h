#ifndef PYRO_ROBOT_ARM_MODULE_H
#define PYRO_ROBOT_ARM_MODULE_H

#include "pyro_module_base.h"
#include <array>
#include "pyro_dm_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

constexpr uint8_t JOINT_NUM = 7;
constexpr uint8_t COEFF_NUM = 4;

using position_t = std::array<float, JOINT_NUM>;

struct arm_cmd_t final : public cmd_base_t
{
    struct segment_t
    {
        float duration;                          //持续时长,单位:秒
        std::array<position_t,COEFF_NUM> coeff;            //[关节][a, b, c, d]

        float start_Tick;
        bool is_initialized;

        static segment_t make_default()
        {
            segment_t s{};
            s.coeff[0].fill(1.0f);
            s.coeff[1].fill(0.0f);
            s.coeff[2].fill(0.0f);
            s.coeff[3].fill(0.0f);
            s.is_initialized = false;
            s.duration = 0.0f;
            s.start_Tick = 0.0f;
            return s;
        }

        void tick_init()
        {
            start_Tick = dwt_drv_t::get_timeline_ms();
        }

        [[nodiscard]] bool is_timeout()
        {
            float current_Tick = dwt_drv_t::get_timeline_ms();
            return current_Tick - start_Tick >= duration;
        }

        void output_val(float* out_val) const
        {
            float dt1 = (dwt_drv_t::get_timeline_ms()-start_Tick)/1000.0f;
            dt1 = std::fmin(dt1,duration);
            float dt2 = dt1 * dt1;
            float dt3 = dt2 * dt1;
            for (size_t j = 0; j < JOINT_NUM; ++j)
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

    std::array<dm_motor_drv_t*,JOINT_NUM> motor{nullptr};
    std::array<pid_t*,JOINT_NUM> pos_pid{nullptr};
    std::array<pid_t*,JOINT_NUM> rot_pid{nullptr};
    std::array<axis_deps_t*,JOINT_NUM> axis{nullptr};};

struct data_ctx_t
{
    std::array<float,JOINT_NUM> feedback_pos;        // 含offset
    std::array<float,JOINT_NUM> feedback_rot;
    std::array<float,JOINT_NUM> target_pos;
    std::array<float,JOINT_NUM> out_torque;
};

struct arm_ctx_t
{
    std::array<dm_motor_drv_t*,JOINT_NUM> motor{nullptr};
    std::array<pid_t*,JOINT_NUM> pos_pid{nullptr};
    std::array<pid_t*,JOINT_NUM> rot_pid{nullptr};

    std::array<arm_deps_t::axis_deps_t*,JOINT_NUM> axis{nullptr};
    data_ctx_t data;
    arm_cmd_t *cmd{nullptr};

    bool is_cmd_empty = true;
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
    [[nodiscard]] const arm_ctx_t& get_ctx();
    void _arm_disable();
    void _arm_enable();
    void _clear_pid();
    position_t _get_current_pos();
    [[nodiscard]] bool _is_empty();
private:
    arm_module_t();
    ~arm_module_t() override = default;

    // 基类接口实现
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // 私有辅助方法
    void _pid_calculation();
    void _send_motor_command();
};

};


#endif //PYRO_ROBOT_ARM_MODULE_H
