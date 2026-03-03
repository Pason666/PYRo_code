#include "pyro_wl_chassis.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::state_test_t::enter(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_test_t::execute(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_wheel_drv[i]->send_torque(0);
    }
    for(uint8_t i = 0; i < 4; i++)
    {
        owner->_motor_drv[i]->send_torque(0);
    }
}
void wl_chassis_t::fsm_active_t::state_test_t::exit(wl_chassis_t *owner)
{
}

}