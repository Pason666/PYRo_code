#include "pyro_sentry_gimbal.h"

namespace pyro
{
enum
{
    UP    = 0,
    DOWN  = 1,
    Left  = 0,
    Right = 1,
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
        // owner->_ctx.data.target_pitch_rad -= 0.01f;
        // 加入判断pitch仰角是否到下限的判断
    }
    else
    {
        // owner->_ctx.data.target_pitch_rad += 0.01f;
        // 加入判断pitch仰角是否到上限的判断
    }

    if (yaw_direction)
    {
        // owner->_ctx.data.target_yaw_rad += 0.01f;
        // 加入判断yaw偏角是否到最右的判断
    }
    else
    {
        // owner->_ctx.data.target_yaw_rad -= 0.01f;
        // 加入判断yaw偏角是否到最左的判断
    }

    _gimbal_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_scanning_t::exit(owner *owner)
{
}

} // namespace pyro