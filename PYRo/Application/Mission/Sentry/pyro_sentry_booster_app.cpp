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

using namespace pyro;

shoot_17mm_control_t *booster_ptr     = nullptr;
booster_cmd_t *booster_cmd_ptr        = nullptr;
booster_cfg_t *booster_cfg_ptr        = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ptr = nullptr;

uint16_t down_time{};
float filtered_speed_error          = 0.0f;
bool first_ball_received            = false;
static constexpr float FILTER_ALPHA = 0.12f;

void booster_config(booster_cfg_t &cfg)
{
    // 上车版
    cfg.motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                  can_hub_t::can2); // 右摩擦轮
    cfg.motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                                  can_hub_t::can2); // 左摩擦轮
    cfg.motor.trigger =
        new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);

    // 小发射测试版
    // cfg.motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
    //                                               can_hub_t::can2); // 右摩擦轮
    // cfg.motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
    //                                               can_hub_t::can2); // 左摩擦轮
    // cfg.motor.trigger =
    //     new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);

    cfg.pid.fric_pid[0]      = new pid_t(0.9f, 0.0f, 0.0f, 0.8f, 20.0f);
    cfg.pid.fric_pid[1]      = new pid_t(0.9f, 0.0f, 0.0f, 0.8f, 20.0f);
    cfg.pid.trig_pos_pid     = new pid_t(1000.0f, 0.0f, 0.0f, 100.0f, 1000.0f);
    cfg.pid.trig_spd_pid     = new pid_t(5.0f, 1.5f, 0.0f, 5.0f, 10.0f);
    cfg.pid.bullet_speed_pid = new pid_t(0.01f, 0.0f, 0.00f, 5.00f, 10.0f);

    cfg.target_fric_speed    = 800;
}

extern "C"
{
    void booster_rc2cmd(void const *rc_ctrl)
    {
        read_scope_lock lock(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state ||
            dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
        {
            if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_l.state ||
                dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_l.state ||
                auto_fire)
            {
                booster_cmd_ptr->is_fric_on = true;

                // 情况 A：拨杆保持在下方 (SW_DOWN) -> 连发模式
                if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_l.state ||
                    auto_fire)
                {
                    down_time++;
                    if (down_time > 800)
                    {
                        booster_cmd_ptr->continue_shoot = true;
                        booster_cmd_ptr->single_shoot   = false;
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
                if (dr16_drv_t::sw_ctrl_t::SW_MID_TO_DOWN ==
                        p_ctrl->rc.s_l.ctrl &&
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

    void chassis2booster()
    {
        booster_cmd_ptr->current_bullet_mps = bullet_speed;
        booster_cmd_ptr->power_heat         = power_heat;
    }

    void speed_control(void)
    {
        float current_speed = bullet_speed;
        if (current_speed < 1.0f)
        {
            current_speed = booster_cfg_ptr->target_fric_speed;
        }
        if (booster_cfg_ptr->target_fric_speed > 7.5f)
        {
            // --- A. 计算当前瞬时误差 ---
            float error_now =
                current_speed - booster_cfg_ptr->target_fric_speed;

            // --- B. 一阶低通滤波 (核心逻辑) ---
            // 公式: Output = Alpha * Input + (1 - Alpha) * Output_Last
            if (!first_ball_received)
            {
                // 第一发弹：直接初始化滤波器
                filtered_speed_error = error_now;
                first_ball_received  = true;
            }
            else
            {
                // 后续发弹：平滑累积误差
                filtered_speed_error =
                    FILTER_ALPHA * error_now +
                    (1 - FILTER_ALPHA) * filtered_speed_error;
            }

            // --- C. 构造带符号的“类平方误差” ---
            // 作用：让大误差被更大权重地修正，小误差被抑制
            float pid_input =
                filtered_speed_error * std::abs(filtered_speed_error);

            // --- D. PID 计算速度增量 ---
            // 逻辑保持不变：将 pid_input 视为误差，期望将其控制到 0
            float speed_increment =
                booster_cfg_ptr->pid.bullet_speed_pid->calculate(0.0f,
                                                                 pid_input);

            // --- E. 执行与限幅 ---
            booster_cfg_ptr->target_fric_speed += speed_increment;

            constexpr float MAX_FRIC1_MPS = 820.0f;
            constexpr float MIN_FRIC1_MPS = 790.0f;

            // 使用 std::clamp (C++17) 更简洁，若不支持则换回 if-else
            booster_cfg_ptr->target_fric_speed =
                std::clamp(booster_cfg_ptr->target_fric_speed, MIN_FRIC1_MPS,
                           MAX_FRIC1_MPS);
        }
    }

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
