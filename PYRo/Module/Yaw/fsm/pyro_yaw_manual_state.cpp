#include "pyro_yaw.h"

namespace pyro
{
float test_target_yaw_rad;
float test_current_yaw_rad;
float test_yaw_torque;
float test_target_yaw_imu_angle;
float test_out_yaw_radps;

int yaw_rotation_loops = 0;

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

void yaw_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
    // owner->_ctx.cmd->target_yaw_imu_angle = owner->_ctx.cmd->current_yaw_imu_rad;
    // yaw_rotation_loops = 0;
}

void yaw_t::fsm_active_t::state_manual_t::execute(owner *owner)
{
    owner->_ctx.data.world_yaw_error = calculate_yaw_error(
        owner->_ctx.cmd->target_yaw_imu_angle,
        owner->_ctx.cmd->current_yaw_imu_rad, yaw_rotation_loops);

    owner->_ctx.data.out_yaw_radps =
        owner->_ctx.yaw_config.pid.yaw_pos_pid->calculate(
            0, owner->_ctx.data.world_yaw_error);
    owner->_ctx.data.out_yaw_torque =
        owner->_ctx.yaw_config.pid.yaw_spd_pid->calculate(
            owner->_ctx.data.out_yaw_radps, owner->_ctx.data.current_yaw_radps);

    _send_motor_command(&owner->_ctx);
}

void yaw_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
}

} // namespace pyro