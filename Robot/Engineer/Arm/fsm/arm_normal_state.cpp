#include "arm_fsm.h"

using namespace pyro;
void arm_top_fsm_t::normal_state_t::enter(owner *owner)
{
    owner->ptp_transition(normal{});
}

void arm_top_fsm_t::normal_state_t::execute(owner *owner)
{

}

void arm_top_fsm_t::normal_state_t::exit(owner *owner)
{
    arm_module_ptr->_clear_pid();
}
