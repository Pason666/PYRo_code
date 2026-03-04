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

using namespace pyro;

gimbal_t *gimbal_ptr                       = nullptr;
gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
gimbal_cfg_t *gimbal_cfg_ptr               = nullptr;
// uart_comm_t *comm                          = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

struct FramHeader
{
    uint8_t sof;
} __attribute__((packed));

FramHeader *framHeader = new FramHeader;

struct InputData2
{
    float vx;
    float vy;
    float vz;
    float wz;
    uint8_t stuck;
} __attribute__((packed));

InputData2 inputData2;

struct FramTailer
{
    uint16_t crc16;
} __attribute__((packed));

FramTailer framTailer;

struct InputData
{
    float curr_yaw;
    float curr_pitch;
    float self_v_magnitude;
    float self_v_angle;
    uint8_t shoot_delay;
    uint8_t state       : 5;
    uint8_t stop_record : 1;
    uint8_t enemy_color : 1;
} __attribute__((packed));

InputData inputData;

struct Enter
{
    char a;
} __attribute__((packed));

Enter enter_data;

struct send2pc
{
    FramHeader header;
    InputData data;
    FramTailer tailer;
    Enter enter;
} __attribute__((packed));

struct send2nav
{
    FramHeader header;
    InputData2 data;
    FramTailer tailer;
} __attribute__((packed));

__attribute__((section(".dma_heap"))) send2pc send2pc_packet;
__attribute__((section(".dma_heap"))) send2nav send2nav_packet;

// struct SensorPacket
// {
//     uint8_t header;     // 0xAA
//     uint32_t timestamp; // 时间戳
//     float temperature;  // 温度
//     float humidity;     // 湿度
// };
//
// SensorPacket tx_packet = {0xAA, 1000, 25.5f, 60.0f};
// SensorPacket rx_packet;

void gimbal_config(gimbal_cfg_t &gimbal_cfg)
{
    gimbal_cfg.motor.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_1, can_hub_t::can1);
    gimbal_cfg.motor.pitch = new dm_motor_drv_t(0x01, 0x00, can_hub_t::can2);
    gimbal_cfg.motor.pitch->set_position_range(-PI, PI);
    gimbal_cfg.motor.pitch->set_rotate_range(-20, 20);
    gimbal_cfg.motor.pitch->set_torque_range(-10, 10);

    gimbal_cfg.pitch_max_rad = 0.13f;
    gimbal_cfg.pitch_min_rad = -0.17f;
    gimbal_cfg.yaw_max_rad   = 0.75f;
    gimbal_cfg.yaw_min_rad   = -0.75f;

    gimbal_cfg.pid.pitch_pos_pid =
        new pid_t(50.0f, 0.05f, 0.09f, 0.5f, 10.0f, 15, 150, 4);
    gimbal_cfg.pid.pitch_spd_pid =
        new pid_t(0.35f, 0.0f, 0.010f, 0.1f, 3.0f, 15, 150, 4);
    gimbal_cfg.pid.yaw_pos_pid =
        new pid_t(30.0f, 0.05f, 0.09f, 0.5f, 10.0f, 15, 150, 4);
    gimbal_cfg.pid.yaw_spd_pid =
        new pid_t(0.35f, 0.0f, 0.010f, 0.1f, 3.0f, 15, 150, 4);

    gimbal_cfg.pitch_offset = 0.27f;
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
        }
        else if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state)
        {
            vx         = static_cast<int8_t>(p_ctrl->rc.ch_lx * 127);
            vy         = static_cast<int8_t>(p_ctrl->rc.ch_ly * 127);
            wz         = 0;
            delta_yaw  = static_cast<int8_t>(p_ctrl->rc.ch_rx * 127);
            // follow_yaw = true;
            follow_yaw = false;
            active     = true;
            scanning   = false;
        }
        else if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
        {
            vx         = static_cast<int8_t>(p_ctrl->rc.ch_lx * 127);
            vy         = static_cast<int8_t>(p_ctrl->rc.ch_ly * 127);
            wz         = 2;
            delta_yaw  = static_cast<int8_t>(p_ctrl->rc.ch_rx * 127);
            follow_yaw = false;
            active     = true;
            scanning   = true;
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

            // comm->write(send2pc_packet);
            // if (comm->read(send2pc_packet))
            // {
            //     if (send2pc_packet.header.sof == 0xA5)
            //     {
            //         // ... 执行成功解析逻辑 ...
            //     }
            // }
            // comm->write(send2nav_packet);
            // comm->read(send2nav_packet);

            vTaskDelay(5);
        }
    }

    status_t sentry_gimbal_init(void *argument)
    {
        gimbal_cmd_ptr = new gimbal_cmd_t();
        gimbal_cfg_ptr = new gimbal_cfg_t();
        // comm           = new uart_comm_t(uart_drv_t::which_uart::uart10, 0x01);
        // // uint8_t header = 0xAA;
        // comm->register_msg_type(sizeof(send2pc_packet),
        //                         &send2pc_packet.header.sof,
        //                         sizeof(send2pc_packet));
        // comm->register_msg_type(sizeof(send2nav_packet),
        //                         &send2nav_packet.header.sof,
        //                         sizeof(send2nav_packet));

        framHeader->sof            = 0xA5;
        send2pc_packet.header      = *framHeader;
        send2nav_packet.header     = *framHeader;

        inputData.curr_yaw         = 0.0f;
        inputData.curr_pitch       = 1.0f;
        inputData.self_v_magnitude = 2.0f;
        inputData.self_v_angle     = 3.0f;
        inputData.shoot_delay      = 4.0f;
        inputData.state            = 5;
        inputData.stop_record      = 0;
        inputData.enemy_color      = 1;

        inputData2.vx              = 1.0f;

        enter_data.a               = '\n';

        send2pc_packet.data        = inputData;
        send2nav_packet.data       = inputData2;

        append_crc16_check_sum(reinterpret_cast<uint8_t *>(&send2pc_packet),
                               sizeof(send2pc_packet));
        append_crc16_check_sum(reinterpret_cast<uint8_t *>(&send2nav_packet),
                               sizeof(send2nav_packet));

        send2pc_packet.enter = enter_data;

        gimbal_ptr           = gimbal_t::instance();
        gimbal_config(*gimbal_cfg_ptr);
        gimbal_ptr->configure(*gimbal_cfg_ptr);
        gimbal_ptr->start();

        rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(sentry_gimbal_thread, "sentry_gimbal_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
        return PYRO_OK;
    }
}

#endif