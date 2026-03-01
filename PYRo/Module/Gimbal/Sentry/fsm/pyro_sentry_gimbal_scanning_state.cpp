#include "pyro_sentry_gimbal.h"

namespace pyro
{
enum
{
    DOWN  = 0,
    UP    = 1,
    Right = 0,
    Left  = 1,
};

bool pitch_direction = UP;
bool yaw_direction   = Left;

void gimbal_t::fsm_active_t::state_scanning_t::enter(owner *owner)
{
}

void gimbal_t::fsm_active_t::state_scanning_t::execute(owner *owner)
{
    if (pitch_direction)
    {
        owner->_ctx.data.target_pitch_rad += 0.0002f;
        if (owner->_ctx.data.target_pitch_rad >=
            owner->_ctx.gimbal_config.pitch_max_rad)
            pitch_direction = DOWN;
    }
    else
    {
        owner->_ctx.data.target_pitch_rad -= 0.0002f;
        if (owner->_ctx.data.target_pitch_rad <=
            owner->_ctx.gimbal_config.pitch_min_rad)
            pitch_direction = UP;
    }

    if (yaw_direction)
    {
        owner->_ctx.data.target_yaw_rad += 0.0004f;
        if (owner->_ctx.data.target_yaw_rad >=
            owner->_ctx.gimbal_config.yaw_max_rad)
            yaw_direction = Right;
    }
    else
    {
        owner->_ctx.data.target_yaw_rad -= 0.0004f;
        if (owner->_ctx.data.target_yaw_rad <=
            owner->_ctx.gimbal_config.yaw_min_rad)
            yaw_direction = Left;
    }

    _gimbal_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_scanning_t::exit(owner *owner)
{
}

} // namespace pyro