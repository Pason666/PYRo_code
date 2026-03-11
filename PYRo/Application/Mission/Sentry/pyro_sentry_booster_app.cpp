#include "pyro_core_config.h"
#if BOARD_ID == GIMBAL_ID

#include "pyro_module_base.h"
#include "pyro_17mm_booster.h"
#include "pyro_com_canrx.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_uart_drv.h"
using namespace pyro;

shoot_17mm_control_t *booster_ptr     = nullptr;
booster_cmd_t *booster_cmd_ptr        = nullptr;
booster_cfg_t *booster_cfg_ptr        = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ptr = nullptr;

float rc_timestamp{};
uint8_t down_time{};

void booster_config(booster_cfg_t &cfg)
{
    cfg.motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                                  can_hub_t::can2); // 右摩擦轮
    cfg.motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                  can_hub_t::can2); // 左摩擦轮
    cfg.motor.trigger =
        new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can2);

    cfg.pid.fric_pid[0]  = new pid_t(0.22f, 0.01f, 0.0f, 0.8f, 20.0f);
    cfg.pid.fric_pid[1]  = new pid_t(0.22f, 0.01f, 0.0f, 0.8f, 20.0f);
    cfg.pid.trig_pos_pid = new pid_t(1000.0f, 0.0f, 0.0f, 100.0f, 1000.0f);
    cfg.pid.trig_spd_pid = new pid_t(0.05f, 0.02f, 0.0f, 5.0f, 20.0f);

    // cfg.pid.trig_pos_pid = new pid_t(8.0f, 0.0f, 0.00f, 10, 100.0f);
    // cfg.pid.trig_spd_pid = new pid_t(0.01f, 0.02f, 0.00f, 5.00f, 10.0f);
}

extern "C"
{
    void booster_rc2cmd(void const *rc_ctrl)
    {
        read_scope_lock lock(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state)
        {
            if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_l.state ||
                dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_l.state)
            {
                booster_cmd_ptr->is_fric_on = true;
            }
            else
                booster_cmd_ptr->is_fric_on = false;

            // 情况 A：拨杆保持在下方 (SW_DOWN) -> 连发模式
            if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_l.state)
            {
                down_time++;
                if (down_time > 200)
                {
                    booster_cmd_ptr->continue_shoot = true;
                    booster_cmd_ptr->single_shoot = false;
                    // 注意：连发模式下，不要触发单发，防止逻辑冲突
                }
            }
            else
            {
                // 拨杆不在下方，关闭连发
                booster_cmd_ptr->continue_shoot = false;
                down_time                       = 0;
            }
            // 情况 B：检测到边沿信号 (MID -> DOWN) -> 触发一次单发
            static float sl_using_time = 0;
            if (dr16_drv_t::sw_ctrl_t::SW_MID_TO_DOWN == p_ctrl->rc.s_l.ctrl &&
                p_ctrl->rc.s_l.change_time != sl_using_time)
            {
                sl_using_time                 = p_ctrl->rc.s_l.change_time;
                booster_cmd_ptr->single_shoot = true;
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

    void chassis2booster_rx()
    {
        std::array<uint8_t, 8> raw_data{};
        can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x102, raw_data);

        booster_cmd_ptr->current_bullet_mps = raw_data[0] + raw_data[1] / 100.0f;
        booster_cmd_ptr->ammo_count = static_cast<int16_t>(raw_data[2] << 8 | raw_data[3]);
    }

    void booster_thread(void *argument)
    {
        while (true)
        {
            booster_rc2cmd(rc_ptr);
            booster_ptr->set_command(*booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    status_t sentry_booster_init(void *argument)
    {
        booster_cmd_ptr = new booster_cmd_t();
        booster_cfg_ptr = new booster_cfg_t();

        booster_ptr     = shoot_17mm_control_t::instance();
        booster_config(*booster_cfg_ptr);
        booster_ptr->configure(*booster_cfg_ptr);
        booster_ptr->start();

        rc_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(booster_thread, "booster_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
        return PYRO_OK;
    }
}

#endif
