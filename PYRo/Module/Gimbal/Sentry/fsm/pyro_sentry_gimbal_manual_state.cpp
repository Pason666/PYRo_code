#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.yaw->enable();
    owner->_ctx.gimbal_config.motor.pitch->enable();
    owner->_ctx.data.target_pitch_rad = pitch;
}

void gimbal_t::fsm_active_t::state_manual_t::execute(owner *owner)
{
    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);
    if (owner->_ctx.data.target_pitch_rad >
        owner->_ctx.gimbal_config.pitch_max_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_max_rad;
    if (owner->_ctx.data.target_pitch_rad <
        owner->_ctx.gimbal_config.pitch_min_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_min_rad;
    if (owner->_ctx.data.target_yaw_rad > owner->_ctx.gimbal_config.yaw_max_rad)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.gimbal_config.yaw_max_rad;
    if (owner->_ctx.data.target_yaw_rad < owner->_ctx.gimbal_config.yaw_min_rad)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.gimbal_config.yaw_min_rad;

    owner->_ctx.data.target_yaw_rad = yaw;
    owner->_ctx.data.target_pitch_rad -=
        owner->_ctx.cmd->target_delta_pitch_rad;
    // owner->_ctx.data.target_yaw_rad -= owner->_ctx.cmd->target_delta_yaw_rad;
    // owner->_ctx.data.target_yaw_radps = owner->_ctx.cmd->test_yaw_radps;
    _gimbal_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
}
} // namespace pyro
