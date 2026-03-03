//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

namespace pyro
{
void yaw_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.yaw_config.motor.yaw->enable();
}

void yaw_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.cmd->scanning)
        this->change_state(&_scanning_state);
    else
        this->change_state(&_manual_state);

}

void yaw_t::fsm_active_t::on_exit(owner *owner)
{
}


} // namespace pyro