#include "arm_fsm.h"
#include "arm_control_execute.h"
#include "arm_module_params.h"

namespace pyro
{
//================顶层状态实例================
state_arm_idle_t          state_arm_idle;
state_arm_joint_direct_t  state_arm_joint_direct;
state_arm_cartesian_t     state_arm_cartesian;
state_arm_auto_sequence_t state_arm_auto_sequence;
//================Tg1(路径生成器)子状态实例================
tg1_idle_t       tg1_idle;
tg1_transition_t tg1_transition;
tg1_running_t    tg1_running;
tg1_paused_t     tg1_paused;
tg1_finished_t   tg1_finished;
tg1_error_t      tg1_error;
// 子状态机管理器实例
static fsm_t<tg1_subctx_t> tg1_sub_fsm;
//==================== 顶层状态实现 ====================
void state_arm_idle_t::enter(arm_fsm_ctx_t *ctx)
{
    ctx->ctx->control_mode = Arm_Ctr_Mode_t::IDLE;
}
void state_arm_idle_t::execute(arm_fsm_ctx_t *ctx)
{
    auto& deps = *ctx->deps;
    for(int i = 0; i < 6; i++)
    {
        deps.motor.axis_motor[i]->send_torque(0.0f);
    }
    deps.motor.end_motor->send_torque(0.0f);
}
void state_arm_idle_t::exit(arm_fsm_ctx_t *ctx)
{
    
}
void state_arm_joint_direct_t::enter(arm_fsm_ctx_t *ctx)
{
    ctx->ctx->control_mode = Arm_Ctr_Mode_t::JOINT_DIRECT;
}
void state_arm_joint_direct_t::execute(arm_fsm_ctx_t *ctx)
{
    auto& ctx_arm = *ctx->ctx;
    auto& cmd = *ctx->cmd;
    bool target_valid = true;
    for(int i = 0; i < 6; i++)
    {
        if(fabs(cmd.axis_target_pos[i]) >= 64.0f) { target_valid = false; break; }
        ctx_arm.axis_control[i]->set_target(cmd.axis_target_pos[i]);
    }
    if(fabs(cmd.axis_target_pos[6]) >= 64.0f)
        target_valid = false;
    else
        ctx_arm.end_axis->set_target(cmd.axis_target_pos[6]);
    if(!target_valid)
    {
        // 状态内部请求切换
        this->request_switch(&state_arm_idle);
        return;
    }
    for(int i = 0; i < 6; i++) ctx_arm.axis_control[i]->pid_control();
    ctx_arm.end_axis->pid_control();
}
void state_arm_joint_direct_t::exit(arm_fsm_ctx_t *ctx)
{
    (void)ctx;
}
void state_arm_cartesian_t::enter(arm_fsm_ctx_t *ctx)
{
    ctx->ctx->control_mode = Arm_Ctr_Mode_t::CARTESIAN;
}
void state_arm_cartesian_t::execute(arm_fsm_ctx_t *ctx)
{
    (void)ctx;
}
void state_arm_cartesian_t::exit(arm_fsm_ctx_t *ctx)
{
    (void)ctx;
}
void state_arm_auto_sequence_t::enter(arm_fsm_ctx_t *ctx)
{
    ctx->ctx->control_mode = Arm_Ctr_Mode_t::AUTO_SEQUENCE;
    tg1_subctx_t* subctx = ctx->tg1_ctx;
    subctx->current_state = Tg1State::IDLE;
    subctx->seq_step = 0;
    subctx->seq_tick = 0;
    subctx->trigger_start = false;
    subctx->trigger_pause = false;
    subctx->trigger_reset = false;
    // fsm_t对象调用change_state
    tg1_sub_fsm.change_state(&tg1_idle);
    tg1_sub_fsm.enter(subctx);
}
void state_arm_auto_sequence_t::execute(arm_fsm_ctx_t *ctx)
{
    tg1_sub_fsm.execute(ctx->tg1_ctx);
}
void state_arm_auto_sequence_t::exit(arm_fsm_ctx_t *ctx)
{
    tg1_sub_fsm.exit(ctx->tg1_ctx);
    (void)ctx;
}
//==================== Tg1子状态实现 ====================
void tg1_idle_t::enter(tg1_subctx_t *ctx)
{
    ctx->current_state = Tg1State::IDLE;
}
void tg1_idle_t::execute(tg1_subctx_t *ctx)
{
    if(ctx->trigger_start)
    {
        ctx->trigger_start = false;
        this->request_switch(&tg1_transition);
    }
}
void tg1_idle_t::exit(tg1_subctx_t *ctx)
{
}
void tg1_transition_t::enter(tg1_subctx_t *ctx)
{
    ctx->current_state = Tg1State::TRANSITION;
    ctx->seq_tick = 0;
}
void tg1_transition_t::execute(tg1_subctx_t *ctx)
{
    ctx->seq_tick++;
    if(ctx->seq_tick >= 200)
    {
        ctx->seq_tick = 0;
        this->request_switch(&tg1_running);
    }
    if(ctx->trigger_reset)
    {
        ctx->trigger_reset = false;
        this->request_switch(&tg1_idle);
    }
}
void tg1_transition_t::exit(tg1_subctx_t *ctx)
{
    (void)ctx;
}
void tg1_running_t::enter(tg1_subctx_t *ctx)
{
    ctx->current_state = Tg1State::RUNNING;
}
void tg1_running_t::execute(tg1_subctx_t *ctx)
{
    ctx->seq_tick++;
    if(ctx->trigger_pause)
    {
        ctx->trigger_pause = false;
        this->request_switch(&tg1_paused);
    }
    if(ctx->trigger_reset)
    {
        ctx->trigger_reset = false;
        this->request_switch(&tg1_idle);
    }
    if(ctx->seq_step >= 10)
    {
        this->request_switch(&tg1_finished);
    }
}
void tg1_running_t::exit(tg1_subctx_t *ctx)
{
    (void)ctx;
}
void tg1_paused_t::enter(tg1_subctx_t *ctx)
{
    ctx->current_state = Tg1State::PAUSED;
}
void tg1_paused_t::execute(tg1_subctx_t *ctx)
{
    if(ctx->trigger_start)
    {
        ctx->trigger_start = false;
        this->request_switch(&tg1_running);
    }
    if(ctx->trigger_reset)
    {
        ctx->trigger_reset = false;
        this->request_switch(&tg1_idle);
    }
}
void tg1_paused_t::exit(tg1_subctx_t *ctx)
{
    (void)ctx;
}
void tg1_finished_t::enter(tg1_subctx_t *ctx)
{
    ctx->current_state = Tg1State::FINISH;
}
void tg1_finished_t::execute(tg1_subctx_t *ctx)
{
    if(ctx->trigger_reset)
    {
        ctx->trigger_reset = false;
        ctx->seq_step = 0;
        ctx->seq_tick = 0;
        this->request_switch(&tg1_idle);
    }
}
void tg1_finished_t::exit(tg1_subctx_t *ctx)
{
    (void)ctx;
}
void tg1_error_t::enter(tg1_subctx_t *ctx)
{
    ctx->current_state = Tg1State::ERROR;
}
void tg1_error_t::execute(tg1_subctx_t *ctx)
{
    if(ctx->trigger_reset)
    {
        ctx->trigger_reset = false;
        this->request_switch(&tg1_idle);
    }
}
void tg1_error_t::exit(tg1_subctx_t *ctx)
{
    (void)ctx;
}
} // namespace pyro