//
// Created by pason on 2026/2/4.
//

#include "pyro_rud_chassis.h"

namespace pyro
{

void rud_chassis_t::fsm_active_t::state_turning_t::enter(rud_chassis_t *owner)
{
}

void rud_chassis_t::fsm_active_t::state_turning_t::execute(rud_chassis_t *owner)
{
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_states.modules[i].speed = 0;
    }

    if (abs(owner->_ctx.data.current_states.modules[0].angle -
            owner->_ctx.data.target_states.modules[0].angle) < 1 &&
        abs(owner->_ctx.data.current_states.modules[1].angle -
            owner->_ctx.data.target_states.modules[1].angle) < 1 &&
        abs(owner->_ctx.data.current_states.modules[2].angle -
            owner->_ctx.data.target_states.modules[2].angle) < 1 &&
        abs(owner->_ctx.data.current_states.modules[3].angle -
            owner->_ctx.data.target_states.modules[3].angle) < 1)
        owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::MOVING;

    _chassis_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void rud_chassis_t::fsm_active_t::state_turning_t::exit(rud_chassis_t *owner)
{
}

} // namespace pyro
