#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"


namespace pyro
{
void quad_booster_t::fsm_active_t::state_interim_t::enter(owner *owner)
{

}

void quad_booster_t::fsm_active_t::state_interim_t::execute(owner *owner)
{
    // @TODO: 循环判断摩擦轮转速，符合要求则进入ready状态
    request_switch(&owner->_state_active._ready_state);
    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_interim_t::exit(owner *owner)
{

}
}