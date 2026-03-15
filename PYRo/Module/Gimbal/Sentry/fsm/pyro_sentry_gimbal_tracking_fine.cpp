#include "pyro_sentry_gimbal.h"


float test_aim_pitch;
float test_pitch;
float test_target_pitch_rad;
float test_current_pitch_rad;

namespace pyro
{
void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::enter(
    owner *owner)
{
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::execute(
    owner *owner)
{
    if (owner->_ctx.cmd->is_aiming == false)
    {
        this->request_switch(&owner->_active_state._manual_state);
    }

    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_angles_b(&yaw, &pitch, &roll);
    pitch = pitch / 180 * PI;
    yaw   = yaw / 180 * PI;

    owner->_ctx.data.target_pitch_rad =
        owner->_ctx.data.current_pitch_rad -
        (pitch - owner->_ctx.cmd->aim_imu_pitch_rad);

    // owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_rad -
    //                                   (yaw - owner->_ctx.cmd->aim_imu_yaw_rad);

    owner->_ctx.data.target_yaw_rad = 0;

    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::exit(
    owner *owner)
{
}
} // namespace pyro
