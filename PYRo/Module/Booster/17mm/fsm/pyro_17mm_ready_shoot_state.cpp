#include "pyro_17mm_booster.h"
// 3. 发射就绪状态 (SHOOT_READY_SHOOT)
namespace pyro
{

void shoot_17mm_control_t::state_ready_shoot_t::enter(owner *ctx)
{
    ctx->_ctx.data.fric_pid_active    = true;
    // --- 位置环锁位, 保持拨弹盘有力 ---
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::POSITION;
    ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.current_trig_rad;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.current_state      = data_ctx_t::state_e::READY_SHOOT;
}

void shoot_17mm_control_t::state_ready_shoot_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 单发触发: 未校准先进入校准 ---
    if (ctx->_ctx.cmd->single_shoot)
    {
        if (!ctx->_ctx.data.is_calibrated)
        {
            // 记录来源状态, 校准完成后进入单发
            ctx->_ctx.data.jam_source_state = data_ctx_t::state_e::READY_SHOOT;
            this->request_switch(&ctx->_state_cali_reverse);
        }
        else
        {
            this->request_switch(&ctx->_state_single_bullet);
        }
        return;
    }

    // --- 连发触发: 不需要校准, 直接进入 ---
    if (ctx->_ctx.cmd->continue_shoot)
    {
        this->request_switch(&ctx->_state_continue_bullet);
    }
}

void shoot_17mm_control_t::state_ready_shoot_t::exit(owner *ctx)
{
}

}