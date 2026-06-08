#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_canrx.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

using namespace pyro;

static pyro::hybrid_chassis_t *hybrid_chassis_ptr       = nullptr;
static pyro::hybrid_cmd_t *hybrid_cmd_ptr               = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
static pyro::hybrid_deps_t *hybrid_deps_ptr             = nullptr;

extern "C"
{
    void chassis_rxcmd(void const *rc_ctrl)
    {
        std::array<uint8_t, 8> raw_data{};
        pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can2, 0x101,
                                     raw_data);
        hybrid_cmd_ptr->vx =
            3 * static_cast<float>(static_cast<int8_t>(raw_data[0])) / 127.0f;
        hybrid_cmd_ptr->vy =
            3 * static_cast<float>(static_cast<int8_t>(raw_data[1])) / 127.0f;
        hybrid_cmd_ptr->wz =
            3 * static_cast<float>(static_cast<int8_t>(raw_data[2])) / 127.0f;
        hybrid_cmd_ptr->mode =
            static_cast<pyro::cmd_base_t::mode_t>(raw_data[3]);
    }

    void hero_chassis_thread(void *argument)
    {
        while (true)
        {
            chassis_rxcmd(rc_ctrl_ptr);
            hybrid_chassis_ptr->set_command(*hybrid_cmd_ptr);
            vTaskDelay(1);
        }
    }

    static void deps_init()
    {
        hybrid_deps_ptr = new pyro::hybrid_deps_t();
        hybrid_deps_ptr->motor_deps.mecanum[0] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                            pyro::can_hub_t::can1); // FL Wheel
        hybrid_deps_ptr->motor_deps.mecanum[1] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                            pyro::can_hub_t::can1); // FR Wheel
        hybrid_deps_ptr->motor_deps.mecanum[2] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                            pyro::can_hub_t::can1); // RL Wheel
        hybrid_deps_ptr->motor_deps.mecanum[3] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                            pyro::can_hub_t::can1); // RR Wheel
        hybrid_deps_ptr->motor_deps.track[0] =
            new pyro::dm_motor_drv_t(0x31, 0x41, pyro::can_hub_t::can2);
        hybrid_deps_ptr->motor_deps.track[1] =
            new pyro::dm_motor_drv_t(0x32, 0x42, pyro::can_hub_t::can2);
        hybrid_deps_ptr->motor_deps.leg[0] =
            new pyro::dm_motor_drv_t(0x11, 0x21, pyro::can_hub_t::can3);
        hybrid_deps_ptr->motor_deps.leg[1] =
            new pyro::dm_motor_drv_t(0x12, 0x22, pyro::can_hub_t::can3);

        // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
            ->set_position_range(-PI, PI);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
            ->set_position_range(-PI, PI);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
            ->set_rotate_range(-20.94f, 20.94f);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
            ->set_rotate_range(-20.94f, 20.94f);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
            ->set_torque_range(-11.0f, 11.0f);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
            ->set_torque_range(-11.0f, 11.0f);

        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
            ->set_position_range(-PI, PI);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
            ->set_position_range(-PI, PI);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
            ->set_rotate_range(-5.655f, 5.655f);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
            ->set_rotate_range(-5.655f, 5.655f);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
            ->set_torque_range(-27.0f, 27.0f);
        static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
            ->set_torque_range(-27.0f, 27.0f);
        // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

        hybrid_deps_ptr->pid_deps.mecanum_pid[0] =
            new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 200, 100, 4);
        hybrid_deps_ptr->pid_deps.mecanum_pid[1] =
            new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 200, 100, 4);
        hybrid_deps_ptr->pid_deps.mecanum_pid[2] =
            new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 200, 100, 4);
        hybrid_deps_ptr->pid_deps.mecanum_pid[3] =
            new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 200, 100, 4);

        hybrid_deps_ptr->pid_deps.follow_yaw_pid =
            new pid_t(0.5f, 0.0001f, 0.00002f, 0.5f, 11.0f, 200, 100, 4);

        hybrid_deps_ptr->pid_deps.track_pid[0] =
            new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 200, 100, 4);
        hybrid_deps_ptr->pid_deps.track_pid[1] =
            new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 200, 100, 4);

        hybrid_deps_ptr->pid_deps.pitch_pid =
            new pid_t(10.0f, 0.001f, 0.0002f, 0.1f, 4.0f, 200, 100, 4);
        hybrid_deps_ptr->pid_deps.roll_pid =
            new pid_t(10.0f, 0.001f, 0.0002f, 0.1f, 3.0f, 200, 100, 4);
    }

    void hero_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can2, 0x101);
        hybrid_cmd_ptr     = new pyro::hybrid_cmd_t();
        hybrid_chassis_ptr = pyro::hybrid_chassis_t::instance();
        deps_init();
        hybrid_chassis_ptr->configure(*hybrid_deps_ptr);
        hybrid_chassis_ptr->start();
        xTaskCreate(hero_chassis_thread, "start_hero_chassis_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}