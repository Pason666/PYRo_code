#include "pyro_rud_chassis.h"
#include <cmath>
#include <vector>

namespace pyro
{
namespace
{
constexpr float ODOM_MIN_SPEED_MPS  = 0.05f;
constexpr float ODOM_MAX_SPEED_MPS  = 2.0f;
constexpr float ODOM_STOP_TOLERANCE = 0.01f;
constexpr float ODOM_DECEL_DISTANCE = 0.20f;
constexpr float ODOM_FALLBACK_DT_S  = 0.001f;
constexpr float ODOM_MAX_DT_S       = 0.02f;
} // namespace

supercap_drv_t::cap_feedback_t test_cap_feedback{};
float test_chassis_power_cap{};
float test_cap_power_cap{};
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

status_t rud_chassis_t::move_distance(move_direction_t direction,
                                      float distance_m, float speed_mps)
{
    if (distance_m <= 0.0f || speed_mps <= 0.0f)
    {
        return PYRO_PARAM_ERROR;
    }

    float direction_vx = 0.0f;
    float direction_vy = 0.0f;
    _direction_to_vector(direction, direction_vx, direction_vy);
    if (direction_vx == 0.0f && direction_vy == 0.0f)
    {
        return PYRO_PARAM_ERROR;
    }

    if (speed_mps < ODOM_MIN_SPEED_MPS)
    {
        speed_mps = ODOM_MIN_SPEED_MPS;
    }
    else if (speed_mps > ODOM_MAX_SPEED_MPS)
    {
        speed_mps = ODOM_MAX_SPEED_MPS;
    }

    scoped_mutex_t lock(get_mutex());
    _ctx.odom.active             = true;
    _ctx.odom.direction          = direction;
    _ctx.odom.direction_vx       = direction_vx;
    _ctx.odom.direction_vy       = direction_vy;
    _ctx.odom.target_distance    = distance_m;
    _ctx.odom.travelled_distance = 0.0f;
    _ctx.odom.speed_mps          = speed_mps;
    _ctx.odom.last_tick          = xTaskGetTickCount();

    _current_cmd.mode        = cmd_base_t::mode_t::ACTIVE;
    _current_cmd.vx          = 0.0f;
    _current_cmd.vy          = 0.0f;
    _current_cmd.wz          = 0.0f;
    _current_cmd.follow_yaw  = false;
    _current_cmd.is_nav_mode = false;
    _current_cmd.timestamp   = _ctx.odom.last_tick;

    return PYRO_OK;
}

void rud_chassis_t::stop_distance_move()
{
    scoped_mutex_t lock(get_mutex());
    _ctx.odom.active = false;
    _current_cmd.vx  = 0.0f;
    _current_cmd.vy  = 0.0f;
    _current_cmd.wz  = 0.0f;
}

bool rud_chassis_t::is_distance_move_active() const
{
    return _ctx.odom.active;
}

float rud_chassis_t::get_distance_move_remaining() const
{
    const float remaining =
        _ctx.odom.target_distance - _ctx.odom.travelled_distance;
    return remaining > 0.0f ? remaining : 0.0f;
}

void rud_chassis_t::get_body_velocity(float &vx, float &vy, float yaw)
{
    scoped_mutex_t lock(get_mutex());
    float body_vx = 0.0f;
    float body_vy = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        const float angle = _ctx.data.current_states.modules[i].angle;
        const float speed = _ctx.data.current_states.modules[i].speed;
        body_vx += speed * sinf(angle);
        body_vy += speed * cosf(angle);
    }
    body_vx *= 0.25f;
    body_vy *= 0.25f;

    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    vx = body_vx * cy - body_vy * sy;
    vy = -(body_vx * sy + body_vy * cy);
}

void rud_chassis_t::_direction_to_vector(move_direction_t direction, float &vx,
                                         float &vy)
{
    vx = 0.0f;
    vy = 0.0f;
    switch (direction)
    {
    case move_direction_t::FORWARD:
        vy = 1.0f;
        break;
    case move_direction_t::BACKWARD:
        vy = -1.0f;
        break;
    case move_direction_t::LEFT:
        vx = -1.0f;
        break;
    case move_direction_t::RIGHT:
        vx = 1.0f;
        break;
    default:
        break;
    }
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

    // 3. 更新 cap_tx 数据
    _ctx.supercap_cmd.power_referee     = 0;
    _ctx.supercap_cmd.power_limit_referee =
        referee_drv_t::get_instance()->get_data().robot_status.chassis_power_limit;
    _ctx.supercap_cmd.power_buffer_limit_referee = 60.0f;
    _ctx.supercap_cmd.power_buffer_referee =
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy;
    _ctx.supercap_cmd.use_cap           = 1;
    _ctx.supercap_cmd.kill_chassis_user = 0;
    _ctx.supercap_cmd.speed_up_user_now = 0;

    // 4. 更新 cap_rx 数据
    _ctx.cap_feedback = supercap_drv_t::get_instance()->get_feedback();
    test_cap_feedback = _ctx.cap_feedback;
    test_chassis_power_cap = test_cap_feedback.chassis_power_cap / 100.0f;
    test_cap_power_cap = test_cap_feedback.cap_power_cap / 100.0f - 250;
}

void rud_chassis_t::_update_odometry_distance()
{
    if (!_ctx.odom.active)
    {
        return;
    }

    const TickType_t now_tick = xTaskGetTickCount();
    TickType_t delta_tick     = now_tick - _ctx.odom.last_tick;
    _ctx.odom.last_tick       = now_tick;

    float dt_s                = ODOM_FALLBACK_DT_S;
    if (delta_tick > 0)
    {
        dt_s = static_cast<float>(delta_tick) *
               static_cast<float>(portTICK_PERIOD_MS) * 0.001f;
        if (dt_s > ODOM_MAX_DT_S)
        {
            dt_s = ODOM_FALLBACK_DT_S;
        }
    }

    float vx = 0.0f;
    float vy = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        const float angle = _ctx.data.current_states.modules[i].angle;
        const float speed = _ctx.data.current_states.modules[i].speed;
        vx += speed * sinf(angle);
        vy += speed * cosf(angle);
    }
    vx *= 0.25f;
    vy *= 0.25f;

    const float distance_delta =
        (vx * _ctx.odom.direction_vx + vy * _ctx.odom.direction_vy) * dt_s;
    if (distance_delta > 0.0f)
    {
        _ctx.odom.travelled_distance += distance_delta;
    }
}

void rud_chassis_t::_apply_odometry_control(float &vx, float &vy, float &wz)
{
    if (!_ctx.odom.active)
    {
        return;
    }

    _update_odometry_distance();

    const float remaining =
        _ctx.odom.target_distance - _ctx.odom.travelled_distance;
    if (remaining <= ODOM_STOP_TOLERANCE)
    {
        _ctx.odom.active = false;
        vx               = 0.0f;
        vy               = 0.0f;
        wz               = 0.0f;
        return;
    }

    float speed_mps = _ctx.odom.speed_mps;
    if (remaining < ODOM_DECEL_DISTANCE)
    {
        speed_mps *= remaining / ODOM_DECEL_DISTANCE;
        if (speed_mps < ODOM_MIN_SPEED_MPS)
        {
            speed_mps = ODOM_MIN_SPEED_MPS;
        }
    }

    vx = _ctx.odom.direction_vx * speed_mps;
    vy = _ctx.odom.direction_vy * speed_mps;
    wz = 0.0f;
}

void rud_chassis_t::_kinematics_solve()
{
    float vx = _ctx.cmd->vx;
    float vy = _ctx.cmd->vy;
    float wz = _ctx.cmd->wz;

    if (_ctx.cmd->mode == rud_cmd_t::mode_t::PASSIVE)
    {
        vx                  = 0.0f;
        vy                  = 0.0f;
        wz                  = 0.0f;
        _ctx.cmd->yaw_error = 0.0f;
        _ctx.odom.active    = false;
    }
    else if (_ctx.cmd->mode == rud_cmd_t::mode_t::ACTIVE)
    {
        test_yaw_error = _ctx.cmd->yaw_error;
        if (_ctx.cmd->follow_yaw == true)
        {
            // if (abs(_ctx.cmd->yaw_error) < 0.05f)
            // {
            //     _ctx.cmd->yaw_error = 0;
            // }
            wz = _ctx.rud_config.pid.follow_yaw_pid->calculate(
                0, _ctx.cmd->yaw_error);
        }
        else if (_ctx.cmd->follow_yaw == false)
        {
            const float raw_vx = vx;
            const float raw_vy = vy;
            vx =
                raw_vx * cosf(_ctx.cmd->yaw_error) - raw_vy * sinf(_ctx.cmd->yaw_error);
            vy =
                raw_vx * sinf(_ctx.cmd->yaw_error) + raw_vy * cosf(_ctx.cmd->yaw_error);
        }

        _apply_odometry_control(vx, vy, wz);
    }
    _ctx.data.target_states = _kinematics->solve(
        vx, vy, wz, _ctx.data.current_states);
}

void rud_chassis_t::_chassis_control(rud_ctx_t *ctx)
{
    // if(ctx->cmd->is_nav_mode)
    // {
    //     if(ctx->cmd->follow_yaw)
    //     {
    //         ctx->data.target_states.modules[0].angle = 1.5708f;
    //         ctx->data.target_states.modules[1].angle = 1.5708f;
    //         ctx->data.target_states.modules[2].angle = 1.5708f;
    //         ctx->data.target_states.modules[3].angle = 1.5708f;
    //     }
    // }

    for (int i = 0; i < 4; i++)
    {
        // 舵位置环

        const float rud_pos_output =
            ctx->rud_config.pid.rud_pos_pid[i]->calculate(
                ctx->data.target_states.modules[i].angle,
                ctx->data.current_states.modules[i].angle);

        // 舵速度环
        ctx->data.out_rud_torque[i] =
            ctx->rud_config.pid.rud_spd_pid[i]->calculate(
                rud_pos_output, ctx->data.current_rud_radps[i]);

        // 轮子速度环
        ctx->data.out_wheel_torque[i] =
            ctx->rud_config.pid.wheel_pid[i]->calculate(
                ctx->data.target_states.modules[i].speed,
                ctx->data.current_states.modules[i].speed);
        test_cspeed[i] = ctx->data.current_states.modules[i].speed;
        tspeed[i]      = ctx->data.target_states.modules[i].speed;
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
    float power_limit = referee_drv_t::get_instance()
                            ->get_data()
                            .robot_status.chassis_power_limit;

    if (ctx->cap_feedback.vot_cap >= 1000)
    {
        // 平均分配
        power_controller.calculate_restricted_torques(
            motor_data.data(), POWERCONTROL_NUM, power_limit + 100);
    }
    else
    {
        // 平均分配
        power_controller.calculate_restricted_torques(
        motor_data.data(), POWERCONTROL_NUM, power_limit);
    }

    // 不平均分配
    // float custom_ratios[POWERCONTROL_NUM] = {0.1f, 0.1f, 0.1f, 0.1f};
    // power_controller.calculate_restricted_torques(
    //     motor_data.data(), POWERCONTROL_NUM, POWER_LIMIT, custom_ratios);

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

void rud_chassis_t::_send_supercap_command() const
{
    supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd); // NOLINT
}

void rud_chassis_t::_decide_cap()
{
    static bool _last_status = false;
    static uint32_t _timer   = 0;
    static bool _delay_done  = false;

    bool current_status      = referee_drv_t::get_instance()
                              ->get_data()
                              .robot_status.power_management_chassis_output;

    if (current_status)
    {
        // --- 情况 A：Chassis 有输出 ---
        if (!_last_status)
        {
            // 刚切到有输出状态：重置计时器和延迟标志
            _timer      = 0;
            _delay_done = false;
        }

        if (!_delay_done)
        {
            // 1. 处理 1000 tick 的初始延迟
            if (++_timer >= 1000)
            {
                _delay_done               = true;
                _timer                    = 0; // 重置用于后续的 10 tick 周期

                // 达到 1000 tick 时立即发送第一次开启指令
                _ctx.supercap_cmd.use_cap = 1;
                _send_supercap_command();
            }
        }
        else
        {
            // 2. 延迟结束后，以 10 tick 为周期发送
            if (++_timer >= 10)
            {
                _timer                    = 0;
                _ctx.supercap_cmd.use_cap = 1;
                _send_supercap_command();
            }
        }
    }
    else
    {
        // --- 情况 B：Chassis 无输出 ---
        if (_last_status)
        {
            // 刚切换到无输出状态：发送一次 use_cap = 0
            _ctx.supercap_cmd.use_cap = 0;
            _send_supercap_command();

            // 重置状态位，防止重复发送
            _delay_done = false;
            _timer      = 0;
        }
    }

    // 更新旧状态
    _last_status = current_status;
}

void rud_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _decide_cap();

    _main_fsm.execute(this);
}

} // namespace pyro
