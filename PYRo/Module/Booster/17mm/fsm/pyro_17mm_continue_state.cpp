#include "pyro_17mm_booster.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pyro_sentry_gimbal.h"
// 7. 连发状态 (SHOOT_CONTINUE_BULLET)
// 速度环全速连发, 进入时清除校准标志, 堵转时进入校准

float test_fric_target_speed[2]{};

namespace pyro
{
pid_t *bullet_speed_pid = new pid_t(5.0f, 0.0f, 0.0f, 5.0f, 8.0f);

void shoot_17mm_control_t::state_continue_bullet_t::enter(owner *ctx)
{
    // --- 连发不需要校准, 进入时清除校准标志 ---
    ctx->_ctx.data.is_calibrated      = false;
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::SPEED; // 切速度模式
    ctx->_ctx.data.target_trig_radps  = TRIGGER_CONTINUOUS_RADPS;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.block_start_tick   = 0; // 清零堵转计时
    ctx->_ctx.data.current_state      = data_ctx_t::state_e::CONTINUE_BULLET;
}

void shoot_17mm_control_t::state_continue_bullet_t::execute(owner *ctx)
{
    // --- 紧急退出 ---
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 停止条件 ---
    if (!ctx->_ctx.cmd->continue_shoot)
    {
        this->request_switch(&ctx->_state_done);
        return;
    }

    // --- 弹速闭环 --- 
    if(last_bullet_speed != bullet_speed)
    {
        ctx->_ctx.data.fric_radps_error = bullet_speed_pid->calculate(22.7f, bullet_speed);
        ctx->_ctx.data.target_fric_radps[0] -= ctx->_ctx.data.fric_radps_error;
        ctx->_ctx.data.target_fric_radps[1] += ctx->_ctx.data.fric_radps_error;
        if(abs(ctx->_ctx.data.target_fric_radps[0]) > lin_v_to_radps(TARGET_BULLET_SPEED) * 1.3f)
        {
            ctx->_ctx.data.target_fric_radps[0] = -lin_v_to_radps(TARGET_BULLET_SPEED) * 1.3f;
        }
        if(abs(ctx->_ctx.data.target_fric_radps[1]) > lin_v_to_radps(TARGET_BULLET_SPEED) * 1.3f)
        {
            ctx->_ctx.data.target_fric_radps[1] = lin_v_to_radps(TARGET_BULLET_SPEED) * 1.3f;
        }
    }

    test_fric_target_speed[0] = ctx->_ctx.data.target_fric_radps[0];
    test_fric_target_speed[1] = ctx->_ctx.data.target_fric_radps[1];

    // --- 堵转检测: 目标速度大但实际极低 ---
    // 模板: speedErr > 50.0f && vel < 10.0f (转子角速度)
    // 换算到拨弹盘: speedErr > 50/36 ≈ 1.4 rad/s, vel < 10/36 ≈ 0.28 rad/s
    float speed_err = std::abs(ctx->_ctx.data.target_trig_radps) - std::abs(ctx->_ctx.data.current_trig_radps);
    if (speed_err > 1.4f && std::abs(ctx->_ctx.data.current_trig_radps) < 0.3f)
    {
        if (ctx->_ctx.data.block_start_tick == 0)
        {
            ctx->_ctx.data.block_start_tick = xTaskGetTickCount();
        }
        else if (xTaskGetTickCount() - ctx->_ctx.data.block_start_tick >= pdMS_TO_TICKS(2000))
        {
            // 堵转超时 2000ms, 记录来源状态并进入校准
            ctx->_ctx.data.jam_source_state = data_ctx_t::state_e::CONTINUE_BULLET;
            this->request_switch(&ctx->_state_cali_reverse);
            return;
        }
    }
    else
    {
        ctx->_ctx.data.block_start_tick = 0;
    }
}

void shoot_17mm_control_t::state_continue_bullet_t::exit(owner *ctx)
{
    // --- 清除弹速闭环影响 ---
    ctx->_ctx.data.target_fric_radps[0] = -lin_v_to_radps(TARGET_BULLET_SPEED);
    ctx->_ctx.data.target_fric_radps[1] = lin_v_to_radps(TARGET_BULLET_SPEED);
    ctx->_ctx.data.fric_radps_error = 0.0f;

    // --- 清空速度环积分 ---
    ctx->_ctx.booster_cfg.pid.trig_spd_pid->clear();

    // --- 对齐到前方最近的槽位, 为位置环锁位做准备 ---
    // 将当前角度向上取整到最近的 "一发" 位置
    float ecd_per_bullet_rad = PI / 4; // 一发对应的角度增量
    float current_relative_rad = ctx->_ctx.data.current_trig_rad - ctx->_ctx.data.trigger_offset;
    int32_t bullet_count = (int32_t)(current_relative_rad / ecd_per_bullet_rad) + 1;
    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.trigger_offset + bullet_count * ecd_per_bullet_rad;

    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::POSITION;
}


} // namespace pyro