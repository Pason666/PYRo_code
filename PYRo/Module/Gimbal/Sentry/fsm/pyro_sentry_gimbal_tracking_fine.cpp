#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::enter(
    owner *owner)
{
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::execute(
    owner *owner)
{
    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_angles_b(&yaw, &pitch, &roll);
    pitch           = pitch / 180 * PI;

    const float delta_imu = pitch - owner->_ctx.cmd->aim_imu_pitch_rad;
    owner->_ctx.data.target_pitch_rad =
        owner->_ctx.data.current_pitch_rad + delta_imu;

    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::exit(
    owner *owner)
{
}
} // namespace pyro
