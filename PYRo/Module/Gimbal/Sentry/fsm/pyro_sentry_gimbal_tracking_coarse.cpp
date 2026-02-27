#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_coarse_t::enter(
    owner *owner)
{
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_coarse_t::execute(
    owner *owner)
{
    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_coarse_t::exit(
    owner *owner)
{
}
} // namespace pyro
