#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_cantx.h"
#include "pyro_quad_booster.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"

using namespace pyro;

static pyro::quad_booster_t *quad_booster_ptr           = nullptr;
static pyro::quad_booster_cmd_t *quad_booster_cmd_ptr   = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
static pyro::quad_deps_t *quad_deps_ptr                 = nullptr;
extern "C"
{
    void booster_rc2cmd(void const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
        if (pyro::dr16_drv_t::sw_state_t::SW_MID != p_ctrl->rc.s_r.state)
        {
            quad_booster_cmd_ptr->mode      = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on   = false;
            quad_booster_cmd_ptr->fric1_mps = 0.0f;
            quad_booster_cmd_ptr->fric2_mps = 0.0f;
            quad_booster_cmd_ptr->fire_enable = false;
            return;
        }
        quad_booster_cmd_ptr->mode      = pyro::cmd_base_t::mode_t::ACTIVE;
        quad_booster_cmd_ptr->fric1_mps = 0.0f; // 可调节
        quad_booster_cmd_ptr->fric2_mps = 0.0f;
        // 摩擦轮控制
        static float sl_using_time      = 0;
        if (pyro::dr16_drv_t::sw_ctrl_t::SW_UP_TO_MID == p_ctrl->rc.s_l.ctrl &&
            p_ctrl->rc.s_l.change_time != sl_using_time)
        {
            sl_using_time                 = p_ctrl->rc.s_l.change_time;
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
        }
        if (pyro::dr16_drv_t::sw_ctrl_t::SW_DOWN_TO_MID ==
                p_ctrl->rc.s_l.ctrl &&
            p_ctrl->rc.s_l.change_time != sl_using_time)
        {
            sl_using_time                     = p_ctrl->rc.s_l.change_time;
            quad_booster_cmd_ptr->fire_enable = true;
        }
        else
        {
            quad_booster_cmd_ptr->fire_enable = false;
        }
        // 开火控制 (单发）
    }


    void hero_booster_thread(void *argument)
    {
        while (true)
        {
            booster_rc2cmd(rc_ctrl_ptr);
            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    static void deps_init()
    {
        quad_deps_ptr = new pyro::quad_deps_t();
        quad_deps_ptr->motor_deps.fric_wheels[0] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                            pyro::can_hub_t::can1); // Fric 1
        quad_deps_ptr->motor_deps.fric_wheels[1] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                            pyro::can_hub_t::can1); // Fric 2
        quad_deps_ptr->motor_deps.fric_wheels[2] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                            pyro::can_hub_t::can1); // Fric 3
        quad_deps_ptr->motor_deps.fric_wheels[3] =
            new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                            pyro::can_hub_t::can1); // Fric 4
        quad_deps_ptr->motor_deps.trigger_wheel =
            new pyro::dm_motor_drv_t(0x20, 0x10, pyro::can_hub_t::can2);

        quad_deps_ptr->pid_deps.fric_pid[0] =
            new pid_t(6.40f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
        quad_deps_ptr->pid_deps.fric_pid[1] =
            new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
        quad_deps_ptr->pid_deps.fric_pid[2] =
            new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
        quad_deps_ptr->pid_deps.fric_pid[3] =
            new pid_t(6.4f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);

        quad_deps_ptr->pid_deps.trigger_pos_pid =
            new pid_t(15.2f, 0.03f, 0.005f, 1.0f, 10.0f, 200, 100, 4);
        quad_deps_ptr->pid_deps.trigger_spd_pid =
            new pid_t(0.5f, 0.02f, 0.005f, 2.0f, 20.0f, 200, 100, 4);
    }

    void hero_booster_init(void *argument)
    {
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();
        deps_init();
        quad_booster_ptr->configure(*quad_deps_ptr);
        rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        quad_booster_ptr->start();
        xTaskCreate(hero_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}