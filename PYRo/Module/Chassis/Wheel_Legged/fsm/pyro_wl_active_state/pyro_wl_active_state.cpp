/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:03:11
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-28 13:14:30
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{ 
    for(uint8_t i = 0; i < 4; i++)
    {
        if(dm_motor_drv_t::ok != owner->_motor_drv[i]->get_error_code())
        {
            owner->_motor_drv[i]->clear_error();
        }
        owner->_motor_drv[i]->enable();
    }
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_wheel_drv[i]->enable();
    }
}

void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *owner)
{
    if(owner->_cmd->active_mode == 1)
    {
        this->change_state(&_state_test);
    }
    if(owner->_cmd->active_mode == 0)
    {
        this->change_state(&_state_normal);
    }
     
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *owner)
{
}
}
