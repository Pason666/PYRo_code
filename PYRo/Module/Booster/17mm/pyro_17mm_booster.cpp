#include "pyro_17mm_booster.h"
#include <cmath>
#include "pyro_core_def.h"

float test_fric1_radps;
float test_fric2_radps;
float test_target_radps;
float test_ttr;
float test_ctr;
float err;

extern pyro::booster_cmd_t *booster_cmd_ptr;
namespace pyro
{

// ================== 物理与数学常量 ==================
namespace
{
constexpr float PI_DIV_4                     = PI / 4.0f;

// 速度与角度设定
constexpr float SHOOT_BULLET_MUZZLE_VELOCITY = 25; // 枪口初速度
constexpr float FRICTION_WHEEL_RADIUS        = 0.03f;
constexpr float SHOOT_FIRE_RADPS =
    (-SHOOT_BULLET_MUZZLE_VELOCITY / FRICTION_WHEEL_RADIUS); // 摩擦轮角速度

// constexpr float TRIGGER_UNJAM_RADPS      = 6.0f;   // 解堵速度
constexpr float TRIGGER_CONTINUOUS_RADPS = 7.85f; // 连续发射速度（拨弹盘速度）

// 堵转判定
// constexpr float TRIGGER_BLOCK_RAD        = 0.2f; // 堵转判定弧度阈值
// constexpr float TRIGGER_BLOCK_RADPS      = 5.0f; // 堵转判定速度阈值
// constexpr uint16_t TRIGGER_BLOCK_TIME    = 300;  // 堵转判定时间阈值

constexpr float TRIGGER_GEAR_RATIO       = 36.0f; // M2006拨弹电机减速比
} // namespace

// ================== 基础接口实现 ==================
shoot_17mm_control_t::shoot_17mm_control_t()
    : module_base_t("booster", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

status_t shoot_17mm_control_t::_init()
{
    _ctx.booster_cfg = _module_deps;
    return PYRO_OK;
}

void shoot_17mm_control_t::_update_feedback()
{
    _ctx.booster_cfg.motor.fric[0]->update_feedback();
    _ctx.booster_cfg.motor.fric[1]->update_feedback();
    _ctx.booster_cfg.motor.trigger->update_feedback();

    _ctx.data.current_fric_radps[0] =
        _ctx.booster_cfg.motor.fric[0]->get_current_rotate();
    _ctx.data.current_fric_radps[1] =
        _ctx.booster_cfg.motor.fric[1]->get_current_rotate();

    float current_rotor_rad =
        _ctx.booster_cfg.motor.trigger->get_current_position();

    if (_ctx.data.is_first_update)
    {
        // 第一次开机，初始化基准角度
        _ctx.data.last_rotor_rad   = current_rotor_rad;
        _ctx.data.current_trig_rad = current_rotor_rad / TRIGGER_GEAR_RATIO;
        _ctx.data.is_first_update  = false;
    }
    else
    {
        // 1. 计算转子转动的极短增量
        float delta_rad = current_rotor_rad - _ctx.data.last_rotor_rad;

        // 2. 处理跨圈突变边界 (0 <-> 2PI 或 -PI <-> PI)
        if (delta_rad > PI)
        {
            delta_rad -= 2.0f * PI;
        }
        else if (delta_rad < -PI)
        {
            delta_rad += 2.0f * PI;
        }

        // 3. 将物理增量除以减速比，累加到连续的拨弹盘世界角度中
        _ctx.data.current_trig_rad += delta_rad / TRIGGER_GEAR_RATIO;

        // _ctx.data.current_trig_rad = wrap_pi(_ctx.data.current_trig_rad);

        // 更新历史值
        _ctx.data.last_rotor_rad = current_rotor_rad;
    }

    _ctx.data.current_trig_radps =
        _ctx.booster_cfg.motor.trigger->get_current_rotate() /
        TRIGGER_GEAR_RATIO;
}

void shoot_17mm_control_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if (!_ctx.cmd->is_fric_on)
        _main_fsm.change_state(&_state_stop);
    _main_fsm.execute(this);
    // _ctx.data.fric_pid_active = false;
    _fric_control(this);
    _trig_control(this);
    _send_motor_command(&_ctx);
}

void shoot_17mm_control_t::_fric_control(shoot_17mm_control_t *ctx)
{
    for (int i = 0; i < 2; i++)
    {
        if (ctx->_ctx.data.fric_pid_active)
        {
            ctx->_ctx.data.out_fric_torque[i] =
                ctx->_ctx.booster_cfg.pid.fric_pid[i]->calculate(
                    ctx->_ctx.data.target_fric_radps[i],
                    ctx->_ctx.data.current_fric_radps[i]);
        }
        else
            ctx->_ctx.data.out_fric_torque[i] = 0.0f;
    }
}

void shoot_17mm_control_t::_trig_control(shoot_17mm_control_t *ctx)
{
    if (!ctx->_ctx.data.trig_pid_active)
    {
        ctx->_ctx.data.out_trig_torque = 0.0f;
    }

    if (ctx->_ctx.data.trig_mode == data_ctx_t::trig_mode_e::POSITION)
    {
        // 1. 位置环：根据目标角度算出目标角速度
        ctx->_ctx.data.target_trig_radps =
            ctx->_ctx.booster_cfg.pid.trig_pos_pid->calculate(
                ctx->_ctx.data.target_trig_rad,
                ctx->_ctx.data.current_trig_rad);
    }
    // 2. 速度环：根据目标角速度算出扭矩 (无论是直接给定的还是位置环算出的)
    ctx->_ctx.data.out_trig_torque =
        ctx->_ctx.booster_cfg.pid.trig_spd_pid->calculate(
            ctx->_ctx.data.target_trig_radps,
            ctx->_ctx.data.current_trig_radps);

    if (!ctx->_ctx.data.trig_output_enable)
    {
        ctx->_ctx.data.out_trig_torque = 0.0f;
    }
}

void shoot_17mm_control_t::_send_motor_command(booster_ctx_t *ctx)
{
    ctx->booster_cfg.motor.fric[0]->send_torque(ctx->data.out_fric_torque[0]);
    ctx->booster_cfg.motor.fric[1]->send_torque(ctx->data.out_fric_torque[1]);
    if (ctx->data.fire_flag ==  true)
    {
        ctx->booster_cfg.motor.trigger->send_torque(ctx->data.out_trig_torque);
    }
}
// ================== FSM 状态实现 ==================

// 1. 停止状态 (SHOOT_STOP)
void shoot_17mm_control_t::state_stop_t::enter(owner *ctx)
{
    ctx->_ctx.data.target_fric_radps[0] = 0;
    ctx->_ctx.data.target_fric_radps[1] = 0;
    ctx->_ctx.data.fric_pid_active      = true;

    ctx->_ctx.data.trig_mode            = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_radps    = 0;
    ctx->_ctx.data.trig_pid_active      = true;
    ctx->_ctx.data.trig_output_enable   = false;
}
void shoot_17mm_control_t::state_stop_t::execute(owner *ctx)
{
    if (ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_ready_fric);
    }

    if (std::abs(ctx->_ctx.data.current_fric_radps[0]) < 10 &&
        std::abs(ctx->_ctx.data.current_fric_radps[1]) < 10)
        ctx->_ctx.data.fric_pid_active = false;

    if (std::abs(ctx->_ctx.data.current_trig_radps) < 0.01f)
        ctx->_ctx.data.trig_pid_active = false;
}
void shoot_17mm_control_t::state_stop_t::exit(owner *ctx)
{
    // 强制重置模式和目标，防止旧数据残留
    ctx->_ctx.data.trig_mode         = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_radps = 0;
    // 这一行很重要：把位置目标也同步为当前位置，防止位置环有历史遗留误差
    // ctx->_ctx.data.target_trig_rad   = ctx->_ctx.data.current_trig_rad;
}

// 2. 摩擦轮启动状态 (SHOOT_READY_FRIC)
void shoot_17mm_control_t::state_ready_fric_t::enter(owner *ctx)
{
    ctx->_ctx.data.target_fric_radps[0] = SHOOT_FIRE_RADPS;
    ctx->_ctx.data.target_fric_radps[1] = -SHOOT_FIRE_RADPS;
    ctx->_ctx.data.fric_pid_active      = true;
    ctx->_ctx.data.trig_output_enable   = false;
}
void shoot_17mm_control_t::state_ready_fric_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    test_fric1_radps  = ctx->_ctx.data.current_fric_radps[0];
    test_fric2_radps  = ctx->_ctx.data.current_fric_radps[1];
    test_target_radps = SHOOT_FIRE_RADPS;

    // 检查摩擦轮速度是否达标 (误差小于 1.0 rad/s)
    if (std::abs(ctx->_ctx.data.current_fric_radps[0] - SHOOT_FIRE_RADPS) <
            150 &&
        std::abs(ctx->_ctx.data.current_fric_radps[1] - (-SHOOT_FIRE_RADPS)) <
            150)
    {
        this->request_switch(&ctx->_state_ready_shoot);
    }
}
void shoot_17mm_control_t::state_ready_fric_t::exit(owner *ctx)
{
}

// 3. 发射就绪状态 (SHOOT_READY_SHOOT)
void shoot_17mm_control_t::state_ready_shoot_t::enter(owner *ctx)
{
    ctx->_ctx.data.fric_pid_active    = true;
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.current_trig_rad;
    ctx->_ctx.data.target_trig_radps  = 0;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.trig_output_enable = false;
}
void shoot_17mm_control_t::state_ready_shoot_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }
    if (ctx->_ctx.cmd->single_shoot)
    {
        // if (ctx->_ctx.data.is_calibrated)
        // {
        this->request_switch(&ctx->_state_single_bullet);
        // }
        // else
        // {
        //     this->request_switch(&ctx->_state_single_bullet);
        // }
    }
    else if (ctx->_ctx.cmd->continue_shoot)
    {
        // ctx->_ctx.data.is_calibrated = false;
        this->request_switch(&ctx->_state_continue_bullet);
    }
}
void shoot_17mm_control_t::state_ready_shoot_t::exit(owner *ctx)
{
}

// 4. 反转预热状态 (SHOOT_REVERSE)
// void shoot_17mm_control_t::state_reverse_t::enter(owner *ctx)
// {
//     ctx->_ctx.data.fric_pid_active = true;
//
//     ctx->_ctx.data.block_time      = 0;
//     ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::SPEED; // 切速度模式
//     ctx->_ctx.data.trig_pid_active    = true;
//     // ctx->_ctx.data.target_trig_radps  = TRIGGER_UNJAM_RADPS;
//     ctx->_ctx.data.trig_output_enable = true;
// }
// void shoot_17mm_control_t::state_reverse_t::execute(owner *ctx)
// {
//     if (std::abs(ctx->_ctx.data.current_trig_radps - TRIGGER_UNJAM_RADPS) >
//         TRIGGER_BLOCK_RADPS)
//     {
//         ctx->_ctx.data.block_time++;
//         if (ctx->_ctx.data.block_time > TRIGGER_BLOCK_TIME * 3)
//         {
//             this->request_switch(&ctx->_state_cali);
//         }
//     }
//     else
//     {
//         ctx->_ctx.data.block_time = 0;
//     }
// }
// void shoot_17mm_control_t::state_reverse_t::exit(owner *ctx)
// {
// }

// // 5. 校准状态 (SHOOT_CALI)
// void shoot_17mm_control_t::state_cali_t::enter(owner *ctx)
// {
//     ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::POSITION; // 切位置模式
//     ctx->_ctx.data.trig_pid_active    = true;
//     ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.current_trig_rad + 0.33f;
//     ctx->_ctx.data.trig_output_enable = true;
// }
// void shoot_17mm_control_t::state_cali_t::execute(owner *ctx)
// {
//     float err = std::abs(ctx->_ctx.data.current_trig_rad -
//                          ctx->_ctx.data.target_trig_rad);
//     if (err < (TRIGGER_BLOCK_RAD * 0.1f))
//     {
//         ctx->_ctx.data.is_calibrated = true;
//         this->request_switch(&ctx->_state_single_bullet);
//     }
// }
// void shoot_17mm_control_t::state_cali_t::exit(owner *ctx)
// {
// }

// 6. 单发状态 (SHOOT_SINGLE_BULLET)
void shoot_17mm_control_t::state_single_bullet_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode  = data_ctx_t::trig_mode_e::POSITION; // 切位置模式
    ctx->_ctx.data.trig_pid_active = true;
    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.current_trig_rad + PI_DIV_4;

    ctx->_ctx.data.trig_output_enable = true;
}

void shoot_17mm_control_t::state_single_bullet_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }
     err = std::abs(ctx->_ctx.data.current_trig_rad -
                         ctx->_ctx.data.target_trig_rad);
    test_ctr = ctx->_ctx.data.current_trig_rad;
    test_ttr = ctx->_ctx.data.target_trig_rad;
    if (abs(err) < 0.05f)
    {
        this->request_switch(&ctx->_state_done);
    }
}

void shoot_17mm_control_t::state_single_bullet_t::exit(owner *ctx)
{
    // ctx->_ctx.cmd->single_shoot = false;
    booster_cmd_ptr->single_shoot = false;
}

// 7. 连发状态 (SHOOT_CONTINUE_BULLET)
void shoot_17mm_control_t::state_continue_bullet_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode  = data_ctx_t::trig_mode_e::SPEED; // 切速度模式
    ctx->_ctx.data.target_trig_radps  = TRIGGER_CONTINUOUS_RADPS;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.trig_output_enable = true;
}

void shoot_17mm_control_t::state_continue_bullet_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->continue_shoot)
    {
        this->request_switch(&ctx->_state_done);
        return;
    }
    // 移除堵弹检测逻辑
}

void shoot_17mm_control_t::state_continue_bullet_t::exit(owner *ctx)
{
}

// 8. 射击完成状态 (SHOOT_DONE)
void shoot_17mm_control_t::state_done_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.target_trig_radps  = 0;
    ctx->_ctx.data.trig_output_enable = false;
}
void shoot_17mm_control_t::state_done_t::execute(owner *ctx)
{
    this->request_switch(&ctx->_state_ready_shoot);
}
void shoot_17mm_control_t::state_done_t::exit(owner *ctx)
{
}

// // 9. 堵弹调整状态 (SHOOT_ADJUST)
// void shoot_17mm_control_t::state_adjust_t::enter(owner *ctx)
// {
//     ctx->_ctx.data.block_time = 0;
//     ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::SPEED; // 退弹用速度控制
//     ctx->_ctx.data.trig_pid_active    = true;
//     ctx->_ctx.data.trig_output_enable = true;
// }
// void shoot_17mm_control_t::state_adjust_t::execute(owner *ctx)
// {
//     ctx->_ctx.data.target_trig_radps = TRIGGER_UNJAM_RADPS;
//     if (std::abs(ctx->_ctx.data.current_trig_radps - TRIGGER_UNJAM_RADPS) >
//         TRIGGER_BLOCK_RADPS)
//     {
//         ctx->_ctx.data.block_time++;
//         if (ctx->_ctx.data.block_time > TRIGGER_BLOCK_TIME)
//         {
//             ctx->_ctx.data.target_trig_rad =
//                 ctx->_ctx.data.current_trig_rad - (PI_DIV_4 / 16.0f);
//             this->request_switch(&ctx->_state_ready_shoot);
//         }
//     }
//     else
//     {
//         ctx->_ctx.data.block_time = 0;
//     }
// }
// void shoot_17mm_control_t::state_adjust_t::exit(owner *ctx)
// {
// }

} // namespace pyro
