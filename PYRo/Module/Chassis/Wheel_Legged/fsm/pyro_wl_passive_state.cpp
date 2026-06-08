/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 19:51:12
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-26 19:58:54
 * @Description: Wheel-legged chassis passive state implementation
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
namespace pyro
{
    
/**
 * @description: 
   When wheel-legged chassis enter passive state, it will call this function.
   In this function, all the motors of the chassis will be disabled.
 * @param {wl_chassis_t} *owner
   Pointer to the wheel-legged chassis instance.
 * @return {*}
 */
void wl_chassis_t::state_passive_t::enter(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        owner->_motor_drv[i]->disable();
    }
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_wheel_drv[i]->disable();
    }
}

void wl_chassis_t::state_passive_t::execute(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_wheel_drv[i]->send_torque(0);
    }
}

void wl_chassis_t::state_passive_t::exit(wl_chassis_t *owner)
{
}

}