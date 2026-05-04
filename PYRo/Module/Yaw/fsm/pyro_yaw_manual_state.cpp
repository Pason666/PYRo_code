#include "pyro_yaw.h"

float test_error;
float test_radps_target;
float test_a, test_b;

namespace pyro
{
void yaw_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
}

void yaw_t::fsm_active_t::state_manual_t::execute(owner *owner)
{
    auto *motor = owner->_ctx.yaw_config.motor.yaw;
    static uint32_t last_retry_tick = 0;
    const uint32_t now = xTaskGetTickCount();

    if (dm_motor_drv_t::ok !=
        owner->_ctx.yaw_config.motor.yaw->get_error_code())
    {
        owner->_ctx.yaw_config.motor.yaw->clear_error();
        owner->_ctx.yaw_config.motor.yaw->enable();
    }
    else
    {
        float target_yaw = owner->_ctx.cmd->target_yaw_imu_angle;
        float current = owner->_ctx.cmd->current_yaw_imu_rad;
        float current_radps = owner->_ctx.cmd->current_yaw_imu_radps;

        while (target_yaw > PI)  target_yaw -= 2 * PI;
        while (target_yaw < -PI) target_yaw += 2 * PI;

        float error = target_yaw - current;
        while (error > PI)  error -= 2 * PI;
        while (error < -PI) error += 2 * PI;

        // 用归一化后的 error 做 PID: calculate(0, -error) 等价于 calculate(target, current)
        // 但避免了 target = current + error 越界 [-PI, PI] 的问题
        owner->_ctx.data.out_yaw_radps =
            owner->_ctx.yaw_config.pid.yaw_pos_pid->calculate(0, error);

        test_error = error;
        test_radps_target = owner->_ctx.data.out_yaw_radps;

        owner->_ctx.data.out_yaw_torque =
            owner->_ctx.yaw_config.pid.yaw_spd_pid->calculate(
                owner->_ctx.data.out_yaw_radps, current_radps);

        _send_motor_command(&owner->_ctx);
    }
}

void yaw_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
}

} // namespace pyro