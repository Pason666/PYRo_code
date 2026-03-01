//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

namespace pyro
{
void yaw_t::state_active_t::enter(owner *owner)
{
    owner->_ctx.yaw_config.motor.yaw->enable();
}

void yaw_t::state_active_t::execute(owner *owner)
{
    if (owner->_ctx.cmd->)
    _yaw_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void yaw_t::state_active_t::exit(owner *owner)
{
}


} // namespace pyro