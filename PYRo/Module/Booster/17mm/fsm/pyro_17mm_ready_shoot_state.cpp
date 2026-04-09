#include "pyro_17mm_booster.h"
// 3. 发射就绪状态 (SHOOT_READY_SHOOT)
namespace pyro
{

void shoot_17mm_control_t::state_ready_shoot_t::enter(owner *ctx)
{
    ctx->_ctx.data.fric_pid_active = true;
    ctx->_ctx.data.target_fric_radps[0] =
        -ctx->_ctx.booster_cfg.target_fric_speed;
    ctx->_ctx.data.target_fric_radps[1] =
        ctx->_ctx.booster_cfg.target_fric_speed;

    ctx->_ctx.data.trig_mode       = data_ctx_t::trig_mode_e::POSITION;
    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.current_trig_rad;
    ctx->_ctx.data.trig_pid_active = true;
}

void shoot_17mm_control_t::state_ready_shoot_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 单发指令 ---
    if (ctx->_ctx.cmd->single_shoot)
    {
        if (!ctx->fire_ctrl.isCalibrated)
        {
            // 未校准 → 先进入校准流程
            request_switch(&instance()->_state_cali_reverse);

        }
        else if (ctx->fire_ctrl.heatController.canShootSingle())
        {
            // 已校准 + 热量允许 → 进入单发
            request_switch(&instance()->_state_single_bullet);
        }
        return;
    }


    if (ctx->_ctx.cmd->continue_shoot)
    {
        this->request_switch(&ctx->_state_continue_bullet);
    }
}

void shoot_17mm_control_t::state_ready_shoot_t::exit(owner *ctx)
{
}

} // namespace pyro