//
// Created by pason on 2026/4/9.
//
#include "pyro_17mm_booster.h"

void pyro::shoot_17mm_control_t::state_cali_forward::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::POSITION;
    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.current_trig_rad + 0.34f;
}

void pyro::shoot_17mm_control_t::state_cali_forward::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    if (abs(ctx->_ctx.data.current_trig_rad - ctx->_ctx.data.target_trig_rad) < 0.001f)
    {
        this->request_switch(&ctx->_state_ready_shoot);
    }
}

void pyro::shoot_17mm_control_t::state_cali_forward::exit(owner *ctx)
{

}