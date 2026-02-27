//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

float cangle     = 0.0f;
float tangle     = 0.0f;
float imu_cangle = 0.0f;
float cyaw       = 0.0f;

namespace pyro
{

float yaw{}, pitch{}, roll{};
int yaw_rotation_loops = 0;
float wrap_pi(float angle)
{
    while (angle > PI)
        angle -= 2 * PI;
    while (angle < -PI)
        angle += 2 * PI;
    return angle;
}

yaw_t::yaw_t() : module_base_t("yaw", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

float calculate_yaw_error(float target, float current, int &loops)
{
    float target_wrap = wrap_pi(target);
    int target_loops  = static_cast<int>((target - target_wrap) / (2 * PI));

    // 2. 计算当前实际角度（回绕角度 + 累计圈数×2π）
    float current_continuous = current + loops * 2 * PI;
    // 3. 计算目标实际角度
    float target_continuous  = target_wrap + target_loops * 2 * PI;

    // 4. 修正累计圈数（处理跨±π的跳变）
    float delta              = target_continuous - current_continuous;
    if (delta > PI)
    {
        loops += 1; // 逆时针跨π，圈数+1
    }
    else if (delta < -PI)
    {
        loops -= 1; // 顺时针跨π，圈数-1
    }

    current_continuous = current + loops * 2 * PI;
    return target_continuous - current_continuous;
}

float yaw_t::get_yaw_error() const
{
    float world_yaw_error =
        wrap_pi(_ctx.data.gimbal_world_yaw - _ctx.data.chassis_world_yaw);
    if (abs(world_yaw_error) < 0.005f)
        return 0.0f;
    else
        return world_yaw_error;
}

status_t yaw_t::_init()
{
    if (_module_deps.motor.yaw == nullptr) {
        // 电机指针未初始化，返回错误码
        return PYRO_ERROR;
    }
    if (_module_deps.pid.yaw_pos_pid == nullptr || _module_deps.pid.yaw_spd_pid == nullptr) {
        // PID 指针未初始化，返回错误
        return PYRO_ERROR;
    }
    _ctx.yaw_config        = _module_deps;
    return PYRO_OK;
}

void yaw_t::_update_feedback()
{
    ins_drv_t *ins = ins_drv_t::get_instance();
    _ctx.yaw_config.motor.yaw->update_feedback();

    // yaw轴当前角度（电机角度， -PI ~ PI）
    _ctx.data.current_yaw_angle =
        wrap_pi(_ctx.yaw_config.motor.yaw->get_current_position() -
                _ctx.yaw_config.yaw_offset);
    cyaw = _ctx.data.current_yaw_angle;

    // 这里需要获取底盘imu数据减去大yaw的机械角度得到yaw轴的imu角度
    ins->get_angles_n(&yaw, &pitch, &roll);
    _ctx.data.chassis_world_yaw = yaw / 180 * PI;
    _ctx.data.gimbal_world_yaw =
        wrap_pi(_ctx.data.chassis_world_yaw - _ctx.data.current_yaw_angle);



    _ctx.data.current_yaw_imu_angle =
        wrap_pi(yaw - _ctx.data.current_yaw_angle);

    // yaw电机当前角速度
    _ctx.data.current_yaw_radps =
        _ctx.yaw_config.motor.yaw->get_current_rotate();
}

void yaw_t::_yaw_control(yaw_ctx_t *ctx)
{
    ctx->data.target_yaw_imu_angle = ctx->cmd->target_yaw_imu_angle;

    cangle                         = ctx->data.current_yaw_angle;
    tangle                         = ctx->data.target_yaw_imu_angle;
    imu_cangle                     = ctx->data.gimbal_world_yaw;

    float world_yaw_error =
        calculate_yaw_error(ctx->data.target_yaw_imu_angle,
                            ctx->data.gimbal_world_yaw, yaw_rotation_loops);

    float yaw_pos_output =
        ctx->yaw_config.pid.yaw_pos_pid->calculate(0, world_yaw_error);

    ctx->data.out_yaw_torque = ctx->yaw_config.pid.yaw_spd_pid->calculate(
        yaw_pos_output, ctx->data.current_yaw_radps);
}

void yaw_t::_send_motor_command(yaw_ctx_t *ctx)
{
    ctx->yaw_config.motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void yaw_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

} // namespace pyro
