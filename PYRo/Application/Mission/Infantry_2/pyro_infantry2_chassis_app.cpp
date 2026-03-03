/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:18:33
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-03-01 15:36:59
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_rc_hub.h"


using namespace pyro;
wl_chassis_t *infantry2_chassis_ptr = nullptr;
wl_cmd_t     *infantry2_chassis_cmd_ptr = nullptr;
dr16_drv_t::dr16_ctrl_t const *infantry2_rc_ctrl_ptr = nullptr;
extern wl_chassis_cfg_t infantry2_chassis_cfg;
extern "C"
{
void infantry2_chassis_rc2cmd(void const *rc_ctrl)
{
    pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(
         pyro::rc_hub_t::DR16)->get_lock());
   static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    if(pyro::dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
    {
        infantry2_chassis_cmd_ptr->l_angle = 0;
        infantry2_chassis_cmd_ptr->r_angle = 0;
        infantry2_chassis_cmd_ptr->l_leg = 0;
        infantry2_chassis_cmd_ptr->r_leg = 0;
        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        return;
    }
    infantry2_chassis_cmd_ptr->l_leg = (p_ctrl->rc.ch_ly + 1.0f) / 14.0f + 0.15f;
    infantry2_chassis_cmd_ptr->r_leg = (p_ctrl->rc.ch_ry + 1.0f) / 14.0f + 0.15f;

    infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    if(p_ctrl->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        infantry2_chassis_cmd_ptr->active_mode = 1;
    }
    else
    {
        infantry2_chassis_cmd_ptr->active_mode = 0;
    }

}
void infantry2_chassis_main_tread(void *argument)
{infantry2_chassis_ptr->start();
    while(1)
    {
        infantry2_chassis_rc2cmd(infantry2_rc_ctrl_ptr);
        infantry2_chassis_ptr->set_command(*infantry2_chassis_cmd_ptr);
        vTaskDelay(1);
    }
}

status_t infantry2_chassis_init(void *argument)
{
    infantry2_chassis_cmd_ptr = new wl_cmd_t();
    infantry2_chassis_ptr = wl_chassis_t::instance();
    infantry2_rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
    BaseType_t ret = 
        xTaskCreate(infantry2_chassis_main_tread, 
            "Infantry2 Chassis", 512, 
            nullptr, 1, nullptr);
    CHECK_OS_RET(ret);
    infantry2_chassis_ptr->configure(infantry2_chassis_cfg);
    return status_t::PYRO_OK;
}

}
