//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

namespace pyro
{
void yaw_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_try_recover_motor();
}

void yaw_t::fsm_active_t::on_execute(owner *owner)
{
    this->change_state(&_manual_state);
}

void yaw_t::fsm_active_t::on_exit(owner *owner)
{
}


} // namespace pyro
