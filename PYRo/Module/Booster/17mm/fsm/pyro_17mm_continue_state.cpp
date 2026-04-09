#include "pyro_17mm_booster.h"
// 7. 连发状态 (SHOOT_CONTINUE_BULLET)

extern pyro::booster_cmd_t *booster_cmd_ptr;

namespace pyro
{

void shoot_17mm_control_t::state_continue_bullet_t::enter(owner *ctx)
{
    ctx->fire_ctrl.isCalibrated   = false;
    ctx->fire_ctrl.blockStartTick = 0;
    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::SPEED; // 切速度模式
    ctx->_ctx.data.trig_pid_active = true;
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
    }

    // --- 热控器动态调节安全射频 ---
    ctx->_ctx.data.target_trig_radps =
        ctx->fire_ctrl.heatController.getSafeBurstRpm(TRIGGER_SPEED, 36.0f);

    // --- 堵转检测 ---
    float err = abs(ctx->_ctx.data.target_trig_radps) -
                abs(ctx->_ctx.data.current_trig_radps);
    if (err > 50.0f && abs(ctx->_ctx.data.current_trig_radps) < 10.0f)
    {
        if (ctx->fire_ctrl.blockStartTick == 0)
            ctx->fire_ctrl.blockStartTick = xTaskGetTickCount();
        else if (xTaskGetTickCount() - ctx->fire_ctrl.blockStartTick > pdMS_TO_TICKS(2000))
        {
            ctx->fire_ctrl.jamSourceState = fire_state::BurstFire;
            request_switch(&instance()->_state_cali_reverse);
            return;
        }
    }
    else
        ctx->fire_ctrl.blockStartTick = 0;
}

void shoot_17mm_control_t::state_continue_bullet_t::exit(owner *ctx)
{
    ctx->_ctx.booster_cfg.pid.trig_spd_pid->clear();
}


} // namespace pyro