#include "pyro_quad_booster.h"
#include "pyro_algo_common.h"
#include "pyro_dm_motor_drv.h"

#include <cmath>

namespace pyro
{

quad_booster_t::quad_booster_t() : module_base_t("quad_booster")
{
    _ctx = {};
}

status_t quad_booster_t::_init()
{
    // 1. 摩擦轮电机初始化
    _ctx.motor.fric_wheels[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can2);
    _ctx.motor.fric_wheels[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can2);
    _ctx.motor.fric_wheels[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);
    _ctx.motor.fric_wheels[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can2);

    // 摩擦轮 PID
    _ctx.pid.fric_pid[0] = new pid_t(6.40f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    _ctx.pid.fric_pid[1] = new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    _ctx.pid.fric_pid[2] = new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    _ctx.pid.fric_pid[3] = new pid_t(6.4f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);

    // 2. 拨弹电机初始化
    _ctx.motor.trigger_wheel =
        new dm_motor_drv_t(0x20, 0x10, can_hub_t::can2);

    // 拨弹 PID
    _ctx.pid.trigger_pos_pid =
        new pid_t(10.2f, 0.03f, 0.005f, 1.0f, 10.0f, 200, 100, 4);
    _ctx.pid.trigger_spd_pid =
        new pid_t(3.6f, 0.02f, 0.005f, 2.0f, 20.0f, 200, 100, 4);

    return PYRO_OK;
}

void quad_booster_t::_update_feedback()
{
    // 1. 摩擦轮反馈
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_mps[i] =
            _ctx.motor.fric_wheels[i]->get_current_rotate() * FRIC1_RADIUS;
    }

    // 2. 拨弹反馈
    _ctx.motor.trigger_wheel->update_feedback();

    // --- A. 速度反馈 ---
    _ctx.data.current_trig_radps =
        _ctx.motor.trigger_wheel->get_current_rotate();

    // --- B. 扭矩反馈 ---
    _ctx.data.current_trig_torque =
        _ctx.motor.trigger_wheel->get_current_torque();

    // --- C. 角度反馈 (-PI ~ PI) ---
    _ctx.data.current_trig_rad = _ctx.motor.trigger_wheel->get_current_position();
}

void quad_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

void quad_booster_t::_fric_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void quad_booster_t::_trigger_position_control()
{
    const float error = _ctx.data.target_trig_rad - _ctx.data.current_trig_rad;
    // 处理过零点问题，选择最短路径
    if (error > PI)
    {
        _ctx.data.target_trig_rad -= 2.0f * PI;
    }
    else if (error < -PI)
    {
        _ctx.data.target_trig_rad += 2.0f * PI;
    }

    // 拨弹 PID 计算
    _ctx.data.target_trig_radps = _ctx.pid.trigger_pos_pid->calculate(
        _ctx.data.target_trig_rad, _ctx.data.current_trig_rad);

    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps);
}

void quad_booster_t::_trigger_speed_control()
{
    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps);
}

void quad_booster_t::_send_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i]);
    }
}

void quad_booster_t::_send_trigger_command() const
{
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

} // namespace pyro