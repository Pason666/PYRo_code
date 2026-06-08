//
// Created by pason on 2026/2/3.
//

#include "pyro_rud_chassis.h"

namespace pyro
{

float last_angle[4]{};
bool is_stick_zeroed        = false;
// bool are_all_servos_reached = false;
bool are_all_wheel_stopped  = false;

void rud_chassis_t::fsm_active_t::state_moving_t::enter(rud_chassis_t *owner)
{
    is_stick_zeroed        = false;
    // are_all_servos_reached = false;
    are_all_wheel_stopped  = false;
}

void rud_chassis_t::fsm_active_t::state_moving_t::execute(rud_chassis_t *owner)
{
    bool stick_is_zero = (owner->_current_cmd.vx == 0 && owner->_current_cmd.vy == 0);

    // 判断电机速度
    for (int i = 0; i < 4; i++)
    {
        if (fabs(owner->_ctx.data.current_states.modules[i].speed) < 0.005f)
            are_all_wheel_stopped = true;
        else
            are_all_wheel_stopped = false;
    }

    // // 判断舵机是否到达目标角度
    // for (int i = 0; i < 4; i++)
    // {
    //     if (calculate_angle_diff(
    //             owner->_ctx.data.current_states.modules[i].angle,
    //             owner->_ctx.data.target_states.modules[i].angle) > 0.f)
    //         are_all_servos_reached = false;
    //     else
    //         are_all_servos_reached = true;
    // }

    if (stick_is_zero)
    {
        is_stick_zeroed = true; // 拨杆已经回零

        // 拨杆回零但是速度未归零 -> 保持舵机角度让电机减速
        if (!are_all_wheel_stopped)
        {
            // 保持当前舵机角度
            for (int i = 0; i < 4; i++)
            {
                owner->_ctx.data.target_states.modules[i].angle = last_angle[i];
                owner->_ctx.data.target_states.modules[i].speed = 0;
            }
            _chassis_control(&owner->_ctx);
            _send_motor_command(&owner->_ctx);
            return;
        }

        else
        {
            owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::BRAKING;
            return;
        }
    }
    else
    {
        is_stick_zeroed = false;
    }

    // // 拨杆未回零时 执行正常逻辑
    // if (are_all_servos_reached)
    // {
    _chassis_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
    // }
    // else
    // {
    //     // 舵机未到位时轮子停转
    //     float temp_speed[4];
    //     for (int i = 0; i < 4; i++)
    //     {
    //         temp_speed[i] = owner->_ctx.data.target_states.modules[i].speed;
    //         owner->_ctx.data.target_states.modules[i].speed = 0;
    //     }
    //     _chassis_control(&owner->_ctx);
    //     _send_motor_command(&owner->_ctx);
    //
    //     for (int i = 0; i < 4; i++)
    //     {
    //         owner->_ctx.data.target_states.modules[i].speed = temp_speed[i];
    //     }
    // }

    // 更新舵机角度记录
    for (int i = 0; i < 4; i++)
        last_angle[i] = owner->_ctx.data.target_states.modules[i].angle;
    //
    //
    //
    //
    // if (owner->_cmd->vx == 0 && owner->_cmd->vy == 0)
    // {
    //     owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::BRAKING;
    //     // 直接设置制动角度，避免先锁last_angle
    //     owner->_ctx.data.target_states.modules[0].angle = PI / 4;
    //     owner->_ctx.data.target_states.modules[1].angle = -PI / 4;
    //     owner->_ctx.data.target_states.modules[2].angle = PI / 4;
    //     owner->_ctx.data.target_states.modules[3].angle = -PI / 4;
    //     // 立即发送制动指令
    //     _chassis_control(&owner->_ctx);
    //     _send_motor_command(&owner->_ctx);
    //     return; // 直接返回，不再执行后续moving逻辑
    // }
    //
    // if (abs(owner->_ctx.data.current_states.modules[0].speed) < 0.005f &&
    //     abs(owner->_ctx.data.current_states.modules[1].speed) < 0.005f &&
    //     abs(owner->_ctx.data.current_states.modules[2].speed) < 0.005f &&
    //     abs(owner->_ctx.data.current_states.modules[3].speed) < 0.005f)
    // {
    //     owner->_ctx.drive_mode = rud_chassis_t::drive_mode_t::BRAKING;
    //     owner->_ctx.data.target_states.modules[0].angle = PI / 4;
    //     owner->_ctx.data.target_states.modules[1].angle = -PI / 4;
    //     owner->_ctx.data.target_states.modules[2].angle = PI / 4;
    //     owner->_ctx.data.target_states.modules[3].angle = -PI / 4;
    // }
    //
    //
    // // for (int i = 0; i < 4; i++)
    // // {
    // //     if (owner->_ctx.data.target_states.modules[i].angle == 0)
    // //         owner->_ctx.data.target_states.modules[i].angle =
    // last_angle[i];
    // //     last_angle[i] = owner->_ctx.data.target_states.modules[i].angle;
    // // }
    //
    // _chassis_control(&owner->_ctx);
    //
    // _send_motor_command(&owner->_ctx);
}

void rud_chassis_t::fsm_active_t::state_moving_t::exit(rud_chassis_t *owner)
{
}


} // namespace pyro