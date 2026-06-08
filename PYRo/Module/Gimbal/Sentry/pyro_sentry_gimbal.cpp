#include "pyro_sentry_gimbal.h"
float yaw, pitch, roll;

float t_aim_pitch{};

namespace pyro
{

float aim_yaw, my_pitchps, my_pitch, my_torque, my_pitch_mec;
float test_current_pos;

float gravity_offset = 0.95
;

gimbal_t::gimbal_t()
    : module_base_t("sentry_gimbal", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

status_t gimbal_t::_init()
{
    _ctx.gimbal_config = _module_deps;
    return PYRO_OK;
}

void gimbal_t::_update_feedback()
{
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);

    my_pitch = pitch;
    
    _ctx.gimbal_config.motor.pitch->update_feedback();
    _ctx.gimbal_config.motor.yaw->update_feedback();

    // 1. 获取当前角度和角速度
    _ctx.data.current_pitch_rad =
        _ctx.gimbal_config.motor.pitch->get_current_position();

    _ctx.data.current_pitch_rad = wrap_pi(_ctx.data.current_pitch_rad);

    my_pitch_mec = _ctx.data.current_pitch_rad;
    test_current_pos = _ctx.data.current_yaw_rad;

    _ctx.data.current_pitch_radps =
        _ctx.gimbal_config.motor.pitch->get_current_rotate();

    _ctx.data.current_yaw_rad =
        _ctx.gimbal_config.motor.yaw->get_current_position() -
        _ctx.gimbal_config.yaw_offset;
    _ctx.data.current_yaw_rad = wrap_pi(_ctx.data.current_yaw_rad);

    _ctx.data.current_yaw_radps =
        _ctx.gimbal_config.motor.yaw->get_current_rotate();

        my_pitchps = _ctx.data.current_pitch_radps;
}

void gimbal_t::_gimbal_mec_control(gimbal_context_t *ctx)
{
    ctx->data.target_pitch_radps =
        ctx->gimbal_config.pid.pitch_pos_pid->calculate(
            ctx->data.target_pitch_rad, ctx->data.current_pitch_rad);

    // pitch轴速度环
    ctx->data.out_pitch_torque =
        ctx->gimbal_config.pid.pitch_spd_pid->calculate(
            ctx->data.target_pitch_radps, ctx->data.current_pitch_radps) 
        - gravity_offset * cos(pitch); // 重力补偿

    // yaw轴位置环
    ctx->data.target_yaw_radps = ctx->gimbal_config.pid.yaw_pos_pid->calculate(
        ctx->data.target_yaw_rad, ctx->data.current_yaw_rad);

    // yaw轴速度环
    ctx->data.out_yaw_torque = ctx->gimbal_config.pid.yaw_spd_pid->calculate(
        ctx->data.target_yaw_radps, ctx->data.current_yaw_radps);
}

void gimbal_t::_gimbal_imu_control(gimbal_context_t *ctx)
{
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);

    t_aim_pitch = ctx->data.target_pitch_rad;

    // pitch轴位置环
    ctx->data.target_pitch_radps =
        -ctx->gimbal_config.pid.pitch_pos_pid->calculate(
            ctx->data.target_pitch_rad, pitch);

    // pitch轴速度环
    ctx->data.out_pitch_torque =
        ctx->gimbal_config.pid.pitch_spd_pid->calculate(
            ctx->data.target_pitch_radps, ctx->data.current_pitch_radps) 
            - gravity_offset * cos(pitch); // 重力补偿

    // yaw轴位置环
    ctx->data.target_yaw_radps = ctx->gimbal_config.pid.yaw_pos_pid->calculate(
        ctx->data.target_yaw_rad, yaw);

    // yaw轴速度环
    ctx->data.out_yaw_torque = ctx->gimbal_config.pid.yaw_spd_pid->calculate(
        ctx->data.target_yaw_radps, ctx->data.current_yaw_radps);
    // ctx->data.out_yaw_torque = 0;
}

void gimbal_t::_send_motor_command(gimbal_context_t *ctx)
{
    my_torque = ctx->data.out_pitch_torque;
    ctx->gimbal_config.motor.pitch->send_torque(ctx->data.out_pitch_torque);
    ctx->gimbal_config.motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void gimbal_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_passive_state);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

} // namespace pyro