//
// Created by pason on 2026/4/8.
//
#include "pyro_17mm_booster.h"

namespace pyro
{

void decide_next_behavior(const shoot_17mm_control_t *ctx)
{
    switch (ctx->fire_ctrl.jamSourceState)
    {
        case shoot_17mm_control_t::fire_state::SingleFire:
            // 单发堵转 → 回到 Ready 等待下次指令
            ctx->fire_ctrl.targetStateAfterCali =
                shoot_17mm_control_t::fire_state::CaliForward;
            break;

        case shoot_17mm_control_t::fire_state::BurstFire:
            // 连发堵转 → 根据热量恢复
            if (ctx->fire_ctrl.heatController.isApproachingHeatLimit())
            {
                if (ctx->fire_ctrl.heatController.canShootSingle())
                {
                    ctx->fire_ctrl.targetStateAfterCali =
                        shoot_17mm_control_t::fire_state::SafeBurst;
                }
                else
                {
                    ctx->fire_ctrl.targetStateAfterCali =
                        shoot_17mm_control_t::fire_state::CaliForward;
                }
            }
            else
            {
                ctx->fire_ctrl.targetStateAfterCali =
                    shoot_17mm_control_t::fire_state::BurstFire;
            }
            break;

        case shoot_17mm_control_t::fire_state::SafeBurst:
            if (ctx->fire_ctrl.heatController.canShootSingle())
            {
                ctx->fire_ctrl.targetStateAfterCali =
                    shoot_17mm_control_t::fire_state::SafeBurst;
            }
            else
            {
                ctx->fire_ctrl.targetStateAfterCali =
                    shoot_17mm_control_t::fire_state::CaliForward;
            }
            break;

        default:
            // 首次校准 → 回到就绪
            ctx->fire_ctrl.targetStateAfterCali =
                shoot_17mm_control_t::fire_state::CaliForward;
            break;
    }
}

void shoot_17mm_control_t::state_cali_reverse::enter(owner *ctx)
{
    ctx->fire_ctrl.blockStartTick    = 0;
    ctx->_ctx.data.trig_mode         = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_radps = TRIGGER_UNJAM_RADPS;
    ctx->fire_ctrl.jamSourceState    = fire_state::CaliReverse;
}

void shoot_17mm_control_t::state_cali_reverse::execute(owner *ctx)
{
    // --- 紧急退出 ---
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 堵转检测 ---
    if (abs(ctx->_ctx.data.current_trig_radps - TRIGGER_UNJAM_RADPS) >
        TRIGGER_UNJAM_RADPS * 0.5f)
    {
        if (ctx->fire_ctrl.blockStartTick == 0)
            ctx->fire_ctrl.blockStartTick = xTaskGetTickCount();
        else if (xTaskGetTickCount() - ctx->fire_ctrl.blockStartTick >
                 pdMS_TO_TICKS(700))
        {
            decide_next_behavior(ctx);
            request_switch(&instance()->_state_cali_forward);
        }
    }
    else
    {
        ctx->fire_ctrl.blockStartTick = 0;
    }
}

void shoot_17mm_control_t::state_cali_reverse::exit(owner *ctx)
{
    ctx->_ctx.booster_cfg.pid.trig_spd_pid->clear();

    ctx->fire_ctrl.triggerOffset = ctx->_ctx.data.current_trig_rad;
}

} // namespace pyro