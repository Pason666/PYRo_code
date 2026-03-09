#include "pyro_com_cantx.h"
#include "pyro_core_config.h"
#if BOARD_ID == GIMBAL_ID

#include "pyro_module_base.h"
#include "pyro_sentry_gimbal.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_cantx.h"
#include "pyro_uart_comm.h"
#include "pyro_crc.h"
#include "pyro_uart_message.h"

using namespace pyro;

extern status_t sentry_booster_init(void *argument);

gimbal_t *gimbal_ptr                       = nullptr;
gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
gimbal_cfg_t *gimbal_cfg_ptr               = nullptr;
uart_comm_t *comm                          = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

__attribute__((section(".dma_heap"))) nav2mcu_msg_t nav2mcu_msg;
__attribute__((section(".dma_heap"))) mcu2nav_msg_t mcu2nav_msg;

void gimbal_config(gimbal_cfg_t &gimbal_cfg)
{
    gimbal_cfg.motor.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_1, can_hub_t::can1);
    gimbal_cfg.motor.pitch = new dm_motor_drv_t(0x01, 0x00, can_hub_t::can1);
    gimbal_cfg.motor.pitch->set_position_range(-PI, PI);
    gimbal_cfg.motor.pitch->set_rotate_range(-20, 20);
    gimbal_cfg.motor.pitch->set_torque_range(-10, 10);

    gimbal_cfg.pitch_max_rad = 0.13f;
    gimbal_cfg.pitch_min_rad = -0.17f;
    gimbal_cfg.yaw_max_rad   = 0.70f;
    gimbal_cfg.yaw_min_rad   = -0.70f;

    gimbal_cfg.pid.pitch_pos_pid =
        new pid_t(50.0f, 0.05f, 0.09f, 0.5f, 10.0f, 15, 150, 4);
    gimbal_cfg.pid.pitch_spd_pid =
        new pid_t(0.35f, 0.0f, 0.010f, 0.1f, 3.0f, 15, 150, 4);
    gimbal_cfg.pid.yaw_pos_pid =
        new pid_t(25.0f, 1.0f, 0.09f, 5, 10.0f, 15, 150, 4);
    gimbal_cfg.pid.yaw_spd_pid =
        new pid_t(0.35f, 0.0f, 0.0f, 0.1f, 6, 15, 150, 4);

    // gimbal_cfg.pid.yaw_pos_pid =
    //     new pid_t(0.5f, 0, 0.09f, 5, 10.0f);
    // gimbal_cfg.pid.yaw_spd_pid =
    //     new pid_t(0.35f, 0.0f, 0.010f, 0.1f, 3);

    gimbal_cfg.pitch_offset = 0.27f;
    // gimbal_cfg.pitch_offset = 0.024f;
    gimbal_cfg.yaw_offset   = 2.05022361f;
}

extern "C"
{
    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        read_scope_lock lock(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        if (dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
        {
            gimbal_cmd_ptr->mode               = gimbal_cmd_t::mode_t::PASSIVE;
            gimbal_cmd_ptr->target_pitch_angle = 0.0f;
            gimbal_cmd_ptr->target_yaw_angle   = 0.0f;
        }
        else if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state)
        {
            gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
            gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::MANUAL;
            gimbal_cmd_ptr->target_pitch_angle -= p_ctrl->rc.ch_ry * 0.001f;
            if (gimbal_cmd_ptr->target_pitch_angle >
                gimbal_cfg_ptr->pitch_max_rad)
                gimbal_cmd_ptr->target_pitch_angle =
                    gimbal_cfg_ptr->pitch_max_rad;
            if (gimbal_cmd_ptr->target_pitch_angle <
                gimbal_cfg_ptr->pitch_min_rad)
                gimbal_cmd_ptr->target_pitch_angle =
                    gimbal_cfg_ptr->pitch_min_rad;
            gimbal_cmd_ptr->target_yaw_angle -= p_ctrl->rc.ch_rx * 0.005f;
            if (gimbal_cmd_ptr->target_yaw_angle > gimbal_cfg_ptr->yaw_max_rad)
                gimbal_cmd_ptr->target_yaw_angle = gimbal_cfg_ptr->yaw_max_rad;
            if (gimbal_cmd_ptr->target_yaw_angle < gimbal_cfg_ptr->yaw_min_rad)
                gimbal_cmd_ptr->target_yaw_angle = gimbal_cfg_ptr->yaw_min_rad;
        }
        else if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
        {
            gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
            gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::SCANNING;
        }
    }

    void chassis_rc2cmd(void const *rc_ctrl)
    {
        read_scope_lock lock(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        static int8_t vx        = 0;
        static int8_t vy        = 0;
        static int8_t wz        = 0;
        static int8_t delta_yaw = 0;
        static bool active      = false;
        static bool follow_yaw  = false;
        static bool scanning    = false;

        can_tx_drv_t::clear(0x101);

        if (dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
        {
            vx         = 0;
            vy         = 0;
            wz         = 0;
            delta_yaw  = 0;
            follow_yaw = false;
            active     = false;
            scanning   = false;
            memset(&nav2mcu_msg, 0, sizeof(nav2mcu_msg));
        }
        else if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state)
        {
            vx         = static_cast<int8_t>(p_ctrl->rc.ch_lx * 127);
            vy         = static_cast<int8_t>(p_ctrl->rc.ch_ly * 127);
            wz         = 0;
            delta_yaw  = static_cast<int8_t>(p_ctrl->rc.ch_rx * 127);
            follow_yaw = true;
            active     = true;
            scanning   = false;
            memset(&nav2mcu_msg, 0, sizeof(nav2mcu_msg));
        }
        else if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
        {
            vx         = static_cast<int8_t>(nav2mcu_msg.data.vx * 127);
            vy         = static_cast<int8_t>(nav2mcu_msg.data.vy * 127);
            wz         = 0;
            delta_yaw  = static_cast<int8_t>(nav2mcu_msg.data.wz * 127);
            follow_yaw = true;
            active     = true;
            scanning   = false;
        }

        can_tx_drv_t::add_data(0x101, 8, vx);
        can_tx_drv_t::add_data(0x101, 8, vy);
        can_tx_drv_t::add_data(0x101, 8, wz);
        can_tx_drv_t::add_data(0x101, 8, delta_yaw);
        can_tx_drv_t::add_data(0x101, 1, static_cast<uint8_t>(follow_yaw));
        can_tx_drv_t::add_data(0x101, 1, static_cast<uint8_t>(active));
        can_tx_drv_t::add_data(0x101, 1, static_cast<uint8_t>(scanning));
        can_tx_drv_t::send(0x101, can_hub_t::get_instance()->hub_get_can_obj(
                                      can_hub_t::which_can::can3));
    }

    void sentry_gimbal_thread(void *argument)
    {
        while (true)
        {
            chassis_rc2cmd(rc_ctrl_ptr);
            gimbal_rc2cmd(rc_ctrl_ptr);
            gimbal_ptr->set_command(*gimbal_cmd_ptr);
            comm->read(nav2mcu_msg);
            comm->write(mcu2nav_msg);
            vTaskDelay(1);
        }
    }

    status_t sentry_gimbal_init(void *argument)
    {
        gimbal_cmd_ptr = new gimbal_cmd_t();
        gimbal_cfg_ptr = new gimbal_cfg_t();
        comm           = new uart_comm_t(uart_drv_t::which_uart::uart10, 0x01);

        nav2mcu_msg.header.sof = 0xA5;
        mcu2nav_msg.header.sof = 0xA5;

        mcu2nav_msg.data.enemy_color = 300;
        mcu2nav_msg.data.stop_record = 400;

        comm->register_msg_type(
            sizeof(nav2mcu_msg),
            reinterpret_cast<const uint8_t *>(&nav2mcu_msg.header),
            sizeof(nav2mcu_msg.header));
        comm->register_msg_type(
            sizeof(mcu2nav_msg),
            reinterpret_cast<const uint8_t *>(&mcu2nav_msg.header),
            sizeof(mcu2nav_msg.header));
        append_crc16_check_sum(reinterpret_cast<uint8_t *>(&mcu2nav_msg),
                               sizeof(mcu2nav_msg));

        gimbal_ptr = gimbal_t::instance();
        gimbal_config(*gimbal_cfg_ptr);
        gimbal_ptr->configure(*gimbal_cfg_ptr);
        gimbal_ptr->start();

        rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(sentry_gimbal_thread, "sentry_gimbal_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        // vTaskDelete(nullptr);
        return PYRO_OK;
    }
}

#endif