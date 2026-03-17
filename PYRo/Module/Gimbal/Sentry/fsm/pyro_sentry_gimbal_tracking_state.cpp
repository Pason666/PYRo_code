#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::fsm_tracking_t::on_enter(gimbal_t *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
}

void gimbal_t::fsm_active_t::fsm_tracking_t::on_execute(gimbal_t *owner)
{
     this->change_state(&_turning_fine_state);
}

void gimbal_t::fsm_active_t::fsm_tracking_t::on_exit(gimbal_t *owner)
{
}

}