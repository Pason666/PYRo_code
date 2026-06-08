#include "pyro_core_config.h"
#if BOARD_ID == GIMBAL_ID

#include "pyro_module_base.h"
#include "pyro_17mm_booster.h"
#include "pyro_com_canrx.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_uart_drv.h"
#include "pyro_uart_message.h"
#include "pyro_uart_comm.h"
#include "pyro_crc.h"
#include "pyro_sentry_gimbal.h"
#include "pyro_17mm_config.h"

#include <algorithm>
#include <math.h>  // 方差计算需要

using namespace pyro;

float test_bullet_speed = 0.0f;

shoot_17mm_control_t *booster_ptr     = nullptr;
booster_cmd_t *booster_cmd_ptr        = nullptr;
booster_cfg_t *booster_cfg_ptr        = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ptr = nullptr;

uint16_t down_time{};

vt03_drv_t::vt03_ctrl_t dddddddata;

void booster_config(booster_cfg_t &cfg)
{
    cfg.motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can1);
    cfg.motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can1);
    cfg.motor.trigger = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can1);

    // cfg.motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can2);
    // cfg.motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can2);
    // cfg.motor.trigger = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);

    // 原有PID不变
    cfg.pid.fric_pid[0]      = new pid_t(0.5f, 0.0f, 0.0f, 0.8f, 20.0f);
    cfg.pid.fric_pid[1]      = new pid_t(0.5f, 0.0f, 0.0f, 0.8f, 20.0f);
    cfg.pid.trig_pos_pid     = new pid_t(30.0f, 0.5f, 0.0f, 3.0f, 20.0f);
    cfg.pid.trig_spd_pid     = new pid_t(4.0f, 0.02f, 0.0f, 5.0f, 10.0f);
}

extern "C"
{
    void booster_rc2cmd(void const *rc_ctrl)
    {
        auto *dr16_driver = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
        read_scope_lock lockdr16(rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl = static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        auto *vt03_driver = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03);
        read_scope_lock lockvt03(vt03_driver->get_lock());
        const auto *rc_data = 
        static_cast<vt03_drv_t::vt03_ctrl_t const*>(vt03_driver->read());

        dddddddata = *rc_data;

        if(vt03_driver->check_online())
        {
            static float last_fn_l_time = 0.0f;
            static float last_fn_r_time = 0.0f;
            if (vt03_drv_t::gear_state_t::GEAR_RIGHT == rc_data->rc.gear.state||
                vt03_drv_t::gear_state_t::GEAR_MID == rc_data->rc.gear.state)
            {
                if(rc_data->rc.fn_l.ctrl == pyro::vt03_drv_t::key_ctrl_t::KEY_PRESSED&&
                   rc_data->rc.fn_l.change_time > last_fn_l_time)
                {
                    booster_cmd_ptr->is_fric_on = !booster_cmd_ptr->is_fric_on;
                    last_fn_l_time = rc_data->rc.fn_l.change_time;
                }
                if(booster_cmd_ptr->is_fric_on)
                {
                    if(rc_data->rc.trigger.ctrl == pyro::vt03_drv_t::key_ctrl_t::KEY_PRESSED&&
                       rc_data->rc.trigger.change_time > last_fn_r_time)
                    {
                        booster_cmd_ptr->single_shoot = true;
                        booster_cmd_ptr->continue_shoot = false;
                        last_fn_r_time = rc_data->rc.trigger.change_time;
                    }
                    else if(rc_data->rc.trigger.ctrl == pyro::vt03_drv_t::key_ctrl_t::KEY_HOLD)
                    { 
                        down_time++;
                        if (down_time > 600)
                        {
                            booster_cmd_ptr->continue_shoot = true;
                            booster_cmd_ptr->single_shoot   = false;
                        }
                    }
                    else
                    {
                        booster_cmd_ptr->single_shoot   = false;
                        booster_cmd_ptr->continue_shoot = false;
                        down_time                       = 0;
                    }
                    

                }
                
                
            }
            else
            {
                booster_cmd_ptr->is_fric_on     = false;
                booster_cmd_ptr->continue_shoot = false;
                booster_cmd_ptr->single_shoot   = false;
                down_time                       = 0;
            }
            

        }
        else if(dr16_driver->check_online())
        {
            if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state || dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
            {
                if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_l.state ||
                    dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_l.state)
                {
                    booster_cmd_ptr->is_fric_on = true;

                    if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_l.state || auto_fire)
                    {
                        down_time++;
                        if (down_time > 800 || auto_fire)
                        {
                            booster_cmd_ptr->continue_shoot = true;
                            booster_cmd_ptr->single_shoot   = false;
                        }
                    }
                    else
                    {
                        booster_cmd_ptr->continue_shoot = false;
                        down_time                       = 0;
                    }

                    static float sl_using_time = 0;
                    if (dr16_drv_t::sw_ctrl_t::SW_MID_TO_DOWN == p_ctrl->rc.s_l.ctrl &&
                        p_ctrl->rc.s_l.change_time != sl_using_time)
                    {
                        sl_using_time                 = p_ctrl->rc.s_l.change_time;
                        booster_cmd_ptr->single_shoot = true;
                        booster_cmd_ptr->continue_shoot = false;
                    }
                }
                else
                {
                    booster_cmd_ptr->is_fric_on = false;
                }
            }
            else
            {
                booster_cmd_ptr->is_fric_on     = false;
                booster_cmd_ptr->continue_shoot = false;
                booster_cmd_ptr->single_shoot   = false;
                down_time                       = 0;
            }
        }
        
    }

    void chassis2booster()
    {
        booster_cmd_ptr->current_bullet_mps = bullet_speed;
        test_bullet_speed = bullet_speed;
        booster_cmd_ptr->power_heat         = power_heat;
        booster_cmd_ptr->heat_limit         = heat_limit;
        booster_cmd_ptr->cooling_rate       = cooling_rate;
    }

    // ===================== 核心：方差+均值闭环（彻底消除弹速跳动）=====================
    // void speed_control(void)
    // {
    //     static float target_lin_v = INIT_FRIC_LIN_V;
    //     float real_bullet_speed = bullet_speed;

    //     // 1. 无效弹速直接保持，不调节
    //     if (real_bullet_speed < VALID_BULLET_SPEED_MIN)
    //     {
    //         booster_cfg_ptr->target_fric_speed = lin_v_to_radps(target_lin_v);
    //         return;
    //     }

    //     // 2. 弹速存入滑动窗口（统计最近10帧）
    //     bullet_buffer[buffer_index] = real_bullet_speed;
    //     buffer_index = (buffer_index + 1) % WINDOW_SIZE;

    //     // 3. 计算窗口内弹速的【平均值】+【方差】
    //     float sum = 0, mean = 0, variance = 0;
    //     // 求和算均值
    //     for (uint8_t i = 0; i < WINDOW_SIZE; i++) sum += bullet_buffer[i];
    //     mean = sum / WINDOW_SIZE;
    //     // 求方差（判断数据稳定性）
    //     for (uint8_t i = 0; i < WINDOW_SIZE; i++) variance += powf(bullet_buffer[i] - mean, 2);
    //     variance /= WINDOW_SIZE;

    //     // 4. 核心逻辑：方差过大=数据乱跳，直接不调节（根治抖动！）
    //     if (variance > MAX_VARIANCE)
    //     {
    //         booster_cfg_ptr->target_fric_speed = lin_v_to_radps(target_lin_v);
    //         return;
    //     }

    //     // 5. 数据稳定后，用【均值】计算误差（无视单帧跳动）
    //     float speed_error = TARGET_BULLET_SPEED - mean;

    //     // 6. 死区：小误差不调节，进一步防抖
    //     if (fabsf(speed_error) < 0.1f)
    //     {
    //         booster_cfg_ptr->target_fric_speed = lin_v_to_radps(target_lin_v);
    //         return;
    //     }

    //     // 7. 极小量PID调节
    //     float adjust = booster_cfg_ptr->pid.bullet_speed_pid->calculate(0.0f, speed_error);
    //     adjust = std::clamp(adjust, -MAX_ADJUST, MAX_ADJUST);

    //     // 8. 严格锁死摩擦轮速度 23.0~24.0
    //     target_lin_v += adjust;
    //     target_lin_v = std::clamp(target_lin_v, MIN_FRIC_LIN_V, MAX_FRIC_LIN_V);

    //     // 9. 输出rad/s给电机
    //     booster_cfg_ptr->target_fric_speed = lin_v_to_radps(target_lin_v);
    // }

    void booster_thread(void *argument)
    {
        while (true)
        {
            booster_rc2cmd(rc_ptr);
            chassis2booster();
            booster_ptr->set_command(*booster_cmd_ptr);
            // speed_control();
            vTaskDelay(1);
        }
    }

    status_t sentry_booster_init(void *argument)
    {
        can_rx_drv_t::subscribe(can_hub_t::which_can::can3, 0x102);
        can_rx_drv_t::subscribe(can_hub_t::which_can::can3, 0x105);
        booster_cmd_ptr = new booster_cmd_t();
        booster_cfg_ptr = new booster_cfg_t();

        booster_ptr     = shoot_17mm_control_t::instance();
        booster_config(*booster_cfg_ptr);
        booster_ptr->configure(*booster_cfg_ptr);
        booster_ptr->start();

        rc_ptr = static_cast<dr16_drv_t::dr16_ctrl_t const *>(
            rc_hub_t::get_instance(rc_hub_t::DR16)->read());
        xTaskCreate(booster_thread, "booster_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
        return PYRO_OK;
    }
}

#endif