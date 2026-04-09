//
// Created by pason on 2026/4/9.
//
#include "pyro_17mm_booster.h"
namespace pyro
{

void shoot_17mm_control_t::state_safe_burst::enter(owner *ctx)
{
    ctx->fire_ctrl.isCalibrated = false;
    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::POSITION; // 切位置模式
    ctx->fire_ctrl.blockStartTick  = 0;

    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.current_trig_rad + PI / 4;
}

void shoot_17mm_control_t::state_safe_burst::execute(owner *ctx)
{
    // --- 紧急退出 ---
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 续杯逻辑 ---
    if (ctx->_ctx.data.target_trig_rad - ctx->_ctx.data.current_trig_rad <
        PI / 8)
    {
        if (ctx->fire_ctrl.heatController.canShootSingle())
        {
            ctx->_ctx.data.target_trig_rad += PI / 4;
        }
        else
        {
            request_switch(&instance()->_state_ready_shoot);
            return;
        }
    }

    // --- 堵转检测 ---
    if (abs(ctx->_ctx.data.target_trig_rad - ctx->_ctx.data.current_trig_rad) >
            PI / 16.0f &&
        abs(ctx->_ctx.data.current_trig_radps) < TRIGGER_SINGLE_JAM_SPEED)
    {
        if (ctx->fire_ctrl.blockStartTick == 0)
            ctx->fire_ctrl.blockStartTick = xTaskGetTickCount();
        else if (xTaskGetTickCount() - ctx->fire_ctrl.blockStartTick >
                 pdMS_TO_TICKS(2000))
        {
            ctx->fire_ctrl.jamSourceState = fire_state::SingleFire;
            request_switch(&instance()->_state_cali_reverse);
            return;
        }
    }
    else
    {
        ctx->fire_ctrl.blockStartTick = 0;
    }
}

void shoot_17mm_control_t::state_safe_burst::exit(owner *ctx)
{

}

} // namespace pyro
