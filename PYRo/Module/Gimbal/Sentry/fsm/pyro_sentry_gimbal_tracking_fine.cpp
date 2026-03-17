#include "pyro_sentry_gimbal.h"

float aim_range_max, aim_range_min, aim_yaw, aim_pitch, aim_yaw_aftercalc;

namespace pyro
{
void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::enter(
    owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.yaw->enable();
    owner->_ctx.gimbal_config.motor.pitch->enable();
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::execute(
    owner *owner)
{
    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);

    if (owner->_ctx.cmd->is_aiming == false)
    {
        this->request_switch(&owner->_active_state._manual_state);
    }

    owner->_ctx.data.aim_imu_max_yaw =
        yaw + (owner->_ctx.gimbal_config.yaw_max_rad -
               owner->_ctx.data.current_yaw_rad);
    owner->_ctx.data.aim_imu_min_yaw =
        yaw - (owner->_ctx.data.current_yaw_rad -
               owner->_ctx.gimbal_config.yaw_min_rad);

    aim_range_max = owner->_ctx.data.aim_imu_max_yaw;
    aim_range_min = owner->_ctx.data.aim_imu_min_yaw;

    owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->aim_imu_pitch_rad;
    owner->_ctx.data.target_yaw_rad   = owner->_ctx.cmd->aim_imu_yaw_rad;

    
    aim_pitch   = owner->_ctx.cmd->aim_imu_pitch_rad;
    aim_yaw = owner->_ctx.cmd->aim_imu_yaw_rad;

    if (owner->_ctx.data.target_pitch_rad >
        owner->_ctx.gimbal_config.pitch_max_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_max_rad;
    if (owner->_ctx.data.target_pitch_rad <
        owner->_ctx.gimbal_config.pitch_min_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_min_rad;

    if (owner->_ctx.data.target_yaw_rad > owner->_ctx.data.aim_imu_max_yaw)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.aim_imu_max_yaw;
    if (owner->_ctx.data.target_yaw_rad < owner->_ctx.data.aim_imu_min_yaw)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.aim_imu_min_yaw;

    float yaw_error = owner->_ctx.cmd->aim_imu_yaw_rad - yaw;
    while (yaw_error > PI)
        yaw_error -= 2 * PI;
    while (yaw_error < -PI)
        yaw_error += 2 * PI;
    owner->_ctx.data.target_yaw_rad += yaw_error;

    aim_yaw_aftercalc = owner->_ctx.data.target_yaw_rad;

    // owner->_ctx.data.target_yaw_rad = 0;

    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::fsm_tracking_t::state_turning_fine_t::exit(
    owner *owner)
{
}
} // namespace pyro
