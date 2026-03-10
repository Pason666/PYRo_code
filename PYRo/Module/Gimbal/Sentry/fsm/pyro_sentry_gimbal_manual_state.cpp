#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{

}

void gimbal_t::fsm_active_t::state_manual_t::execute(
    owner *owner)
{
    // owner->_ctx.cmd->target_yaw_angle = 0.0f;
    owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->target_pitch_angle;
    owner->_ctx.data.target_yaw_rad = owner->_ctx.cmd->target_yaw_angle;
    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_manual_t::exit(
    owner *owner)
{
}
} // namespace pyro
