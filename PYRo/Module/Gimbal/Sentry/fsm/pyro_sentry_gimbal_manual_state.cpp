#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_rad;
}

void gimbal_t::fsm_active_t::state_manual_t::execute(
    owner *owner)
{
    if (owner->_ctx.cmd->is_aiming == true)
    {
        this->request_switch(&owner->_active_state._tracking_state);
    }

    owner->_ctx.data.target_yaw_rad = 0.0f;
    owner->_ctx.data.target_pitch_rad -= owner->_ctx.cmd->target_delta_pitch_rad;
    // owner->_ctx.data.target_yaw_rad -= owner->_ctx.cmd->target_delta_yaw_rad;
    // owner->_ctx.data.target_yaw_radps = owner->_ctx.cmd->test_yaw_radps;
    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_manual_t::exit(
    owner *owner)
{
}
} // namespace pyro
