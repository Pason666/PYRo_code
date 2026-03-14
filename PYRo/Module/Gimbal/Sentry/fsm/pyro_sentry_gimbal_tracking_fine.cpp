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
    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_angles_b(&yaw, &pitch, &roll);
    pitch           = pitch / 180 * PI;

    test_aim_pitch = owner->_ctx.cmd->aim_imu_pitch_rad;
    test_pitch = pitch;


    const float delta_pitch_imu = pitch - owner->_ctx.cmd->aim_imu_pitch_rad;
    owner->_ctx.data.target_pitch_rad =
        owner->_ctx.data.current_pitch_rad + delta_pitch_imu;

    test_target_pitch_rad = owner->_ctx.data.target_pitch_rad;
    test_current_pitch_rad = owner->_ctx.data.current_pitch_rad;

    const float delta_yaw_imu = yaw - owner->_ctx.cmd->aim_imu_yaw_rad;
    owner->_ctx.data.target_yaw_rad =
        owner->_ctx.data.current_yaw_rad + delta_yaw_imu * 0.2f;

    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::exit(
    owner *owner)
{
}
} // namespace pyro
