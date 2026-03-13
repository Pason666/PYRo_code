#include "pyro_rud_chassis.h"

float test_power_limit{};
namespace pyro
{
/**********************************************************************/
float test_cspeed[4]{};
float tspeed[4]{};
float test_ctorque[4]{};
float test_yaw_error{};
float predict[4];
/**********************************************************************/
// static float _mps_to_rpm(const float mps, const float radius)
// {
//     // v = w * r  -> w = v / r
//     // RPM = w * 60 / 2pi
//     if (radius < 1e-4f)
//         return 0.0f;
//     return (mps / radius) * 9.5492966f;
// }
//
// static float _radps_to_rpm(const float radps)
// {
//     // RPM = (w * 60) / (2 * pi)
//     return radps * 9.5492966f;
// }

rud_chassis_t::rud_chassis_t()
    : module_base_t("rudder", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

status_t rud_chassis_t::_init()
{
    _kinematics               = new rudder_kin_t(0.36f, 0.36f);
    _ctx.rud_config           = _module_deps;
    _ctx.hardware.power_meter = new powermeter_drv_t(0x212, can_hub_t::can2);
    _ctx.power.data           = new powermeter_data();
    return PYRO_OK;
}

void rud_chassis_t::_update_feedback()
{
    _ctx.rud_config.motor.rudder[0]->update_feedback();
    _ctx.rud_config.motor.rudder[1]->update_feedback();
    _ctx.rud_config.motor.rudder[2]->update_feedback();
    _ctx.rud_config.motor.rudder[3]->update_feedback();
    _ctx.rud_config.motor.wheel[0]->update_feedback();
    _ctx.rud_config.motor.wheel[1]->update_feedback();
    _ctx.rud_config.motor.wheel[2]->update_feedback();
    _ctx.rud_config.motor.wheel[3]->update_feedback();

    // 1. 四个舵机的角度和角速度
    // 舵机当前角度（-PI ~ PI）
    _ctx.data.current_states.modules[rudder_kin_t::FL].angle =
        _ctx.rud_config.motor.rudder[0]->get_current_position() -
        _ctx.rud_config.rud_pos_moving_offset[0];
    _ctx.data.current_states.modules[rudder_kin_t::FR].angle =
        _ctx.rud_config.motor.rudder[1]->get_current_position() -
        _ctx.rud_config.rud_pos_moving_offset[1];
    _ctx.data.current_states.modules[rudder_kin_t::BL].angle =
        _ctx.rud_config.motor.rudder[2]->get_current_position() -
        _ctx.rud_config.rud_pos_moving_offset[2];
    _ctx.data.current_states.modules[rudder_kin_t::BR].angle =
        _ctx.rud_config.motor.rudder[3]->get_current_position() -
        _ctx.rud_config.rud_pos_moving_offset[3];
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.data.current_states.modules[i].angle > PI)
            _ctx.data.current_states.modules[i].angle -= 2 * PI;
        else if (_ctx.data.current_states.modules[i].angle < -PI)
            _ctx.data.current_states.modules[i].angle += 2 * PI;
    }
    // 舵机当前角速度
    _ctx.data.current_rud_radps[0] =
        _ctx.rud_config.motor.rudder[0]->get_current_rotate();
    _ctx.data.current_rud_radps[1] =
        _ctx.rud_config.motor.rudder[1]->get_current_rotate();
    _ctx.data.current_rud_radps[2] =
        _ctx.rud_config.motor.rudder[2]->get_current_rotate();
    _ctx.data.current_rud_radps[3] =
        _ctx.rud_config.motor.rudder[3]->get_current_rotate();

    // 2. 四个轮子的 RPM
    _ctx.data.current_states.modules[rudder_kin_t::FL].speed =
        _ctx.rud_config.motor.wheel[0]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;

    _ctx.data.current_states.modules[rudder_kin_t::FR].speed =
        _ctx.rud_config.motor.wheel[1]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;

    _ctx.data.current_states.modules[rudder_kin_t::BL].speed =
        _ctx.rud_config.motor.wheel[2]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;

    _ctx.data.current_states.modules[rudder_kin_t::BR].speed =
        _ctx.rud_config.motor.wheel[3]->get_current_rotate() *
        dji_m3508_motor_drv_t::reciprocal_reduction_ratio * RUD_RADIUS;
}

void rud_chassis_t::_kinematics_solve()
{
    if (_ctx.cmd->mode == rud_cmd_t::mode_t::PASSIVE)
    {
        _ctx.cmd->vx        = 0.0f;
        _ctx.cmd->vy        = 0.0f;
        _ctx.cmd->wz        = 0.0f;
        _ctx.cmd->yaw_error = 0.0f;
    }
    else if (_ctx.cmd->mode == rud_cmd_t::mode_t::ACTIVE)
    {
        test_yaw_error = _ctx.cmd->yaw_error;
        if (_ctx.cmd->follow_yaw == true)
        {
            _ctx.cmd->wz = _ctx.rud_config.pid.follow_yaw_pid->calculate(
                0, _ctx.cmd->yaw_error);
        }
        else if (_ctx.cmd->follow_yaw == false)
        {
            const float vx = _ctx.cmd->vx;
            const float vy = _ctx.cmd->vy;
            _ctx.cmd->vx =
                vx * cosf(_ctx.cmd->yaw_error) - vy * sinf(_ctx.cmd->yaw_error);
            _ctx.cmd->vy =
                vx * sinf(_ctx.cmd->yaw_error) + vy * cosf(_ctx.cmd->yaw_error);
            _ctx.cmd->wz = 0.0f;
        }
    }
    _ctx.data.target_states = _kinematics->solve(
        _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz, _ctx.data.current_states);
}

void rud_chassis_t::_chassis_control(rud_ctx_t *ctx)
{
    for (int i = 0; i < 4; i++)
    {
        // 舵机位置环

        const float rud_pos_output =
            ctx->rud_config.pid.rud_pos_pid[i]->calculate(
                ctx->data.target_states.modules[i].angle,
                ctx->data.current_states.modules[i].angle);

        // 舵机速度环
        ctx->data.out_rud_torque[i] =
            ctx->rud_config.pid.rud_spd_pid[i]->calculate(
                rud_pos_output, ctx->data.current_rud_radps[i]);

        // 轮子速度环
        ctx->data.out_wheel_torque[i] =
            ctx->rud_config.pid.wheel_pid[i]->calculate(
                ctx->data.target_states.modules[i].speed,
                ctx->data.current_states.modules[i].speed);
        test_cspeed[i] = ctx->data.current_states.modules[i].speed;
        tspeed[i] = ctx->data.target_states.modules[i].speed;
    }

#if POWER_CONTROL_USE
    std::vector<power_control_drv_t::motor_data_t> motor_data(POWERCONTROL_NUM);

    power_control_drv_t &power_controller = power_control_drv_t::get_instance();
    for (int i = 0; i < POWERCONTROL_NUM; i++)
    {
        motor_data.at(i).gyro       = ctx->data.current_states.modules[i].speed;
        motor_data.at(i).torque_cmd = ctx->data.out_wheel_torque[i];
        motor_data.at(i).power_predict = power_controller.motor_power_predict(
            i, motor_data.at(i).torque_cmd, motor_data.at(i).gyro);
    }

    const float power_limit = referee_drv_t::get_instance()
                            ->get_data()
                            .robot_status.chassis_power_limit;
    test_power_limit = referee_drv_t::get_instance()
                            ->get_data()
                            .robot_status.chassis_power_limit;

    // 不平均分配
    // float custom_ratios[POWERCONTROL_NUM] = {0.1f, 0.1f, 0.1f, 0.1f};
    // power_controller.calculate_restricted_torques(
    //     motor_data.data(), POWERCONTROL_NUM, POWER_LIMIT, custom_ratios);

    // 平均分配
    power_controller.calculate_restricted_torques(
        motor_data.data(), POWERCONTROL_NUM, power_limit);
    for (int i = 0; i < POWERCONTROL_NUM; i++)
    {
        ctx->data.out_wheel_torque[i] = motor_data.at(i).restricted_torque;
        predict[i]                    = motor_data.at(i).power_predict;
    }


#endif
}

void rud_chassis_t::_send_motor_command(rud_ctx_t *ctx)
{
    // 发送舵机扭矩命令
    for (int i = 0; i < 4; i++)
    {
        ctx->rud_config.motor.rudder[i]->send_torque(
            ctx->data.out_rud_torque[i]);
    }

    // 发送轮子扭矩命令
    for (int i = 0; i < 4; i++)
    {
        test_ctorque[i] = ctx->data.out_wheel_torque[i];
        ctx->rud_config.motor.wheel[i]->send_torque(
            ctx->data.out_wheel_torque[i]);
    }
}

void rud_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

} // namespace pyro