//
// Created by pason on 2026/2/3.
//
#include "pyro_rud_chassis.h"

namespace pyro
{
void rud_chassis_t::fsm_active_t::state_braking_t::enter(rud_chassis_t *owner)
{
}

void rud_chassis_t::fsm_active_t::state_braking_t::execute(rud_chassis_t *owner)
{
    owner->_ctx.data.target_states.modules[0].angle = PI / 4;
    owner->_ctx.data.target_states.modules[1].angle = -PI / 4;
    owner->_ctx.data.target_states.modules[2].angle = PI / 4;
    owner->_ctx.data.target_states.modules[3].angle = -PI / 4;

    _chassis_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);

    if (owner->_current_cmd.vx != 0 || owner->_current_cmd.vy != 0)
        owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::TURNING;
}

void rud_chassis_t::fsm_active_t::state_braking_t::exit(rud_chassis_t *owner)
{
}

} // namespace pyro
