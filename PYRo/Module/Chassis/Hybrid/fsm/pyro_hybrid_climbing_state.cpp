#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::climbing_state_t::enter(owner *owner)
{
    // 进入爬坡模式，重新初始化所有驱动机构的 PID
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }
}

void hybrid_chassis_t::fsm_active_t::climbing_state_t::execute(owner *owner)
{
    // 1. 轮腿 VMC 控制 (爬坡时维持车身不后倾翻车，甚至可根据 delta_pitch 压车头)
    owner->_leg_control();

    // 2. 麦轮速度环控制 (提供前轮牵引力)
    owner->_mecanum_control();

    // 3. 履带速度环控制 (履带正式介入，提供主要越障/爬坡推进力)
    owner->_track_control();

    // 4. 统一发送所有电机指令
    owner->_send_motor_command();
}

void hybrid_chassis_t::fsm_active_t::climbing_state_t::exit(owner *owner)
{
    // 退出爬坡模式时的清理工作（当前可留空）
}

} // namespace pyro