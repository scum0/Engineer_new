#ifndef __ARM_FSM_H
#define __ARM_FSM_H
#include "arm_module_params.h"
#include "pyro_core_fsm.h"

namespace pyro
{
//================顶层状态机================
struct arm_fsm_ctx_t;

// 基类：继承 state_t<上下文类型>
class arm_state_base_t : public state_t<arm_fsm_ctx_t>
{
public:
    virtual ~arm_state_base_t() = default;
};

class state_arm_idle_t : public arm_state_base_t
{
public:
    void enter(arm_fsm_ctx_t *ctx) override;
    void execute(arm_fsm_ctx_t *ctx) override;
    void exit(arm_fsm_ctx_t *ctx) override;
};

class state_arm_joint_direct_t : public arm_state_base_t
{
public:
    void enter(arm_fsm_ctx_t *ctx) override;
    void execute(arm_fsm_ctx_t *ctx) override;
    void exit(arm_fsm_ctx_t *ctx) override;
};

class state_arm_cartesian_t : public arm_state_base_t
{
public:
    void enter(arm_fsm_ctx_t *ctx) override;
    void execute(arm_fsm_ctx_t *ctx) override;
    void exit(arm_fsm_ctx_t *ctx) override;
};

class state_arm_auto_sequence_t : public arm_state_base_t
{
public:
    void enter(arm_fsm_ctx_t *ctx) override;
    void execute(arm_fsm_ctx_t *ctx) override;
    void exit(arm_fsm_ctx_t *ctx) override;
};

//================ Tg1(路径生成器)子状态机 =================
struct tg1_subctx_t;

class tg1_substate_base_t : public state_t<tg1_subctx_t>
{
public:
    virtual ~tg1_substate_base_t() = default;
};

class tg1_idle_t        : public tg1_substate_base_t {
public:
    void enter(tg1_subctx_t *ctx) override;
    void execute(tg1_subctx_t *ctx) override;
    void exit(tg1_subctx_t *ctx) override;
};
class tg1_transition_t  : public tg1_substate_base_t {
public:
    void enter(tg1_subctx_t *ctx) override;
    void execute(tg1_subctx_t *ctx) override;
    void exit(tg1_subctx_t *ctx) override;
};
class tg1_running_t     : public tg1_substate_base_t {
public:
    void enter(tg1_subctx_t *ctx) override;
    void execute(tg1_subctx_t *ctx) override;
    void exit(tg1_subctx_t *ctx) override;
};
class tg1_paused_t      : public tg1_substate_base_t {
public:
    void enter(tg1_subctx_t *ctx) override;
    void execute(tg1_subctx_t *ctx) override;
    void exit(tg1_subctx_t *ctx) override;
};
class tg1_finished_t    : public tg1_substate_base_t {
public:
    void enter(tg1_subctx_t *ctx) override;
    void execute(tg1_subctx_t *ctx) override;
    void exit(tg1_subctx_t *ctx) override;
};
class tg1_error_t       : public tg1_substate_base_t {
public:
    void enter(tg1_subctx_t *ctx) override;
    void execute(tg1_subctx_t *ctx) override;
    void exit(tg1_subctx_t *ctx) override;
};

// 顶层状态全局实例
extern state_arm_idle_t         state_arm_idle;
extern state_arm_joint_direct_t state_arm_joint_direct;
extern state_arm_cartesian_t    state_arm_cartesian;
extern state_arm_auto_sequence_t state_arm_auto_sequence;

// Tg1子状态全局实例
extern tg1_idle_t       tg1_idle;
extern tg1_transition_t tg1_transition;
extern tg1_running_t    tg1_running;
extern tg1_paused_t     tg1_paused;
extern tg1_finished_t   tg1_finished;
extern tg1_error_t      tg1_error;

} // namespace pyro
#endif // ARM_FSM_H