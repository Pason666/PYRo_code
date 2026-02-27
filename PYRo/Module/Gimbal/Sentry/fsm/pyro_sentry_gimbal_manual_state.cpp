#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{

}

void gimbal_t::fsm_active_t::state_manual_t::execute(
    owner *owner)
{
    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_manual_t::exit(
    owner *owner)
{
}
} // namespace pyro
