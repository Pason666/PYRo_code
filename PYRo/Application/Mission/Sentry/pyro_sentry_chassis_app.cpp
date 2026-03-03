#include "pyro_core_config.h"
#if BOARD_ID == CHASSIS_ID

#include "pyro_module_base.h"
#include "pyro_rud_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_canrx.h"
#include "pyro_yaw.h"
#include "pyro_uart_drv.h"
using namespace pyro;

rud_chassis_t *rud_chassis_ptr             = nullptr;
yaw_t *yaw_ptr                             = nullptr;
rud_cmd_t *rud_cmd_ptr                     = nullptr;
yaw_cmd_t *yaw_cmd_ptr                     = nullptr;
rud_cfg_t *rud_cfg_ptr                     = nullptr;
yaw_cfg_t *yaw_cfg_ptr                     = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
dr16_drv_t::dr16_ctrl_t dr16_data;

void chassis_config(rud_cfg_t &rud_cfg)
{
    rud_cfg.motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                    can_hub_t::can2); // FL Rudder
    rud_cfg.motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                    can_hub_t::can2); // BL Rudder
    rud_cfg.motor.rudder[2] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                    can_hub_t::can1); // BR Rudder
    rud_cfg.motor.rudder[3] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                    can_hub_t::can1); // FR Rudder

    rud_cfg.motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can2); // FL Wheel
    rud_cfg.motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can2); // BL Wheel
    rud_cfg.motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BR Wheel
    rud_cfg.motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // FR Wheel

    rud_cfg.pid.wheel_pid[0]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    rud_cfg.pid.wheel_pid[1]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    rud_cfg.pid.wheel_pid[2]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);
    rud_cfg.pid.wheel_pid[3]   = new pid_t(20.0f, 0.1f, 0.00f, 1.00f, 20.0f);

    rud_cfg.pid.rud_pos_pid[0] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    rud_cfg.pid.rud_pos_pid[1] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    rud_cfg.pid.rud_pos_pid[2] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    rud_cfg.pid.rud_pos_pid[3] = new pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);

    rud_cfg.pid.rud_spd_pid[0] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    rud_cfg.pid.rud_spd_pid[1] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    rud_cfg.pid.rud_spd_pid[2] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    rud_cfg.pid.rud_spd_pid[3] = new pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

    rud_cfg.pid.follow_yaw_pid = new pid_t(3.0f, 0.01f, 0.02f, 0.1f, 4.0f);

    rud_cfg.rud_pos_moving_offset[0] = 1.01472831f;
    rud_cfg.rud_pos_moving_offset[1] = -0.29145637f;
    rud_cfg.rud_pos_moving_offset[2] = -1.87299052f;
    rud_cfg.rud_pos_moving_offset[3] = -1.04003897f;

    power_control_drv_t &power_controller =
        power_control_drv_t::get_instance(4);
    power_control_drv_t::motor_coefficient_t coef1;
    coef1.k1 = 0;
    coef1.k2 = 0;
    coef1.k3 = 0;
    coef1.k4 = 0;
    power_controller.set_motor_coefficient(1, coef1);

    power_control_drv_t::motor_coefficient_t coef2;
    coef2.k1 = 0;
    coef2.k2 = 0;
    coef2.k3 = 0;
    coef2.k4 = 0;
    power_controller.set_motor_coefficient(2, coef2);

    power_control_drv_t::motor_coefficient_t coef3;
    coef3.k1 = 0;
    coef3.k2 = 0;
    coef3.k3 = 0;
    coef3.k4 = 0;
    power_controller.set_motor_coefficient(3, coef3);

    power_control_drv_t::motor_coefficient_t coef4;
    coef4.k1 = 0;
    coef4.k2 = 0;
    coef4.k3 = 0;
    coef4.k4 = 0;
    power_controller.set_motor_coefficient(4, coef4);
}

void yaw_config(yaw_cfg_t &yaw_cfg)
{
    yaw_cfg.motor.yaw = new dm_motor_drv_t(0x01, 0x02, can_hub_t::can2);
    yaw_cfg.motor.yaw->set_position_range(-PI, PI);
    yaw_cfg.motor.yaw->set_rotate_range(-20, 20);
    yaw_cfg.motor.yaw->set_torque_range(-10, 10);

    yaw_cfg.pid.yaw_pos_pid =
        new pid_t(50.0f, 0.0f, 0.09f, 0.5f, 10.0f, 15, 150, 4);
    yaw_cfg.pid.yaw_spd_pid =
        new pid_t(0.35f, 0.0f, 0.010f, 0.1f, 3.0f, 15, 150, 4);

    yaw_cfg.yaw_offset = -2.40028524f;
}

extern "C"
{
    void chassis_rxcmd(void const *rc_ctrl)
    {
        std::array<uint8_t, 8> raw_data{};
        can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x101, raw_data);
        if (static_cast<bool>(static_cast<int8_t>(raw_data[4] >> 1)) & 0x01)
        {
            rud_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
            yaw_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
        }
        else
        {
            rud_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
            yaw_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
        }

        rud_cmd_ptr->follow_yaw =
            static_cast<bool>(raw_data[4] & 0x01);
        rud_cmd_ptr->vx =
            2 * static_cast<float>(static_cast<int8_t>(raw_data[0])) / 127.0f;
        rud_cmd_ptr->vy =
            2 * static_cast<float>(static_cast<int8_t>(raw_data[1])) / 127.0f;
        rud_cmd_ptr->wz = static_cast<float>(static_cast<int8_t>(raw_data[2]));

        yaw_cmd_ptr->target_yaw_imu_angle -=
            static_cast<float>(static_cast<int8_t>(raw_data[3])) / 127.0f *
            0.003f;
        yaw_cmd_ptr->scanning =
            static_cast<bool>(static_cast<int8_t>(raw_data[4] >> 2)) & 0x01;
        rud_cmd_ptr->yaw_error = yaw_ptr->get_yaw_error();
    }

    void chassis_pc2cmd()
    {

    }
    void sentry_chassis_thread(void *argument)
    {
        while (true)
        {
            // chassis_rxcmd(rc_ctrl_ptr);
            chassis_rxcmd(rc_ctrl_ptr);
            rud_chassis_ptr->set_command(*rud_cmd_ptr);
            yaw_ptr->set_command(*yaw_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void sentry_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can3, 0x101);
        rud_cmd_ptr     = new rud_cmd_t();
        rud_cfg_ptr     = new rud_cfg_t();
        yaw_cmd_ptr     = new yaw_cmd_t();
        yaw_cfg_ptr     = new yaw_cfg_t();

        // can_rx_drv_t::subscribe(can_hub_t::which_can::can2, 0x101);
        rud_chassis_ptr = rud_chassis_t::instance();
        chassis_config(*rud_cfg_ptr);
        rud_chassis_ptr->configure(*rud_cfg_ptr);
        yaw_ptr = yaw_t::instance();
        yaw_config(*yaw_cfg_ptr);
        yaw_ptr->configure(*yaw_cfg_ptr);
        rud_chassis_ptr->start();
        yaw_ptr->start();
        xTaskCreate(sentry_chassis_thread, "sentry_chassis_thread", 512,
                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}

#endif