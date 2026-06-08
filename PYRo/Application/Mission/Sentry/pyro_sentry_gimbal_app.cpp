#include "pyro_core_config.h"
#if BOARD_ID == GIMBAL_ID

#include "pyro_module_base.h"
#include "pyro_sentry_gimbal.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"
#include "pyro_com_cantx.h"
#include "pyro_com_canrx.h"
#include "pyro_uart_comm.h"
#include "pyro_crc.h"
#include "pyro_uart_message.h"

using namespace pyro;

uint8_t if_aim = 0;

float aim_yaw, aim_pitch, test_rx;
static float big_yaw = 0.0f;

uint8_t game_started;
bool autoaim = false;
uint8_t enemy_color{};
uint8_t in_aim{};
bool scan{};

// 轮子几何参数（与底盘端 RUD_RADIUS=0.060f 保持一致）
#define WHEEL_DIAMETER       0.120f
#define WHEEL_CIRCUMFERENCE  ((float)PI * WHEEL_DIAMETER)

// 底盘里程计坐标（底盘端 CAN 0x124 发送累计绝对值）
static float current_odom_x = 0.0f;
static float current_odom_y = 0.0f;

// 路点队列 —— 首次手动模式可连续多次调用
static constexpr uint8_t MAX_WAYPOINTS = 8;
static float    waypoint_sx[MAX_WAYPOINTS];
static float    waypoint_sy[MAX_WAYPOINTS];
static uint16_t waypoint_delay[MAX_WAYPOINTS];
static uint8_t  waypoint_count    = 0;
static uint8_t  waypoint_index    = 0;
static bool     first_manual_done = false;

// 距离移动功能状态
static bool     distance_move_active = false;
static float    move_target_sx       = 0.0f;
static float    move_target_sy       = 0.0f;
static float    move_start_odom_x    = 0.0f;
static float    move_start_odom_y    = 0.0f;
static uint16_t move_delay_counter   = 0;

static constexpr float DISTANCE_MOVE_SPEED = 0.5f;
static constexpr float DISTANCE_TOLERANCE  = 0.01f;

gimbal_t *gimbal_ptr                       = nullptr;
gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
gimbal_cfg_t *gimbal_cfg_ptr               = nullptr;
uart_comm_t *comm                          = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

__attribute__((section(".dma_heap"))) mcu2aim_msg_t mcu2aim_msg;
__attribute__((section(".dma_heap"))) aim2mcu_msg_t aim2mcu_msg;

    // 添加移动路点，自动拆为先直走再横走，避免斜向移动
    void sentry_move_by_distance(float sx, float sy, uint16_t delay_ms)
    {
        if (first_manual_done) return;
        if (waypoint_count + 1 >= MAX_WAYPOINTS) return;

        waypoint_sx[waypoint_count]    = sx;
        waypoint_sy[waypoint_count]    = 0.0f;
        waypoint_delay[waypoint_count] = delay_ms;
        waypoint_count++;

        waypoint_sx[waypoint_count]    = 0.0f;
        waypoint_sy[waypoint_count]    = sy;
        waypoint_delay[waypoint_count] = 0;
        waypoint_count++;
    }

    // 从队列加载下一个路点，队列为空时结束
    static void waypoint_load_next()
    {
        if (waypoint_index < waypoint_count)
        {
            move_target_sx      = waypoint_sx[waypoint_index];
            move_target_sy      = waypoint_sy[waypoint_index];
            move_delay_counter  = waypoint_delay[waypoint_index];
            move_start_odom_x   = current_odom_x;
            move_start_odom_y   = current_odom_y;
            distance_move_active = true;
            waypoint_index++;
        }
        else
        {
            distance_move_active = false; // 队列空，交还遥控器控制
        }
    }


void gimbal_config(gimbal_cfg_t &gimbal_cfg)
{
    gimbal_cfg.motor.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_1, can_hub_t::can1);
    gimbal_cfg.motor.pitch = new dm_motor_drv_t(0x01, 0x00, can_hub_t::can2);
    gimbal_cfg.motor.pitch->set_position_range(-PI, PI);
    gimbal_cfg.motor.pitch->set_rotate_range(-20, 20);
    gimbal_cfg.motor.pitch->set_torque_range(-10, 10);

    gimbal_cfg.pitch_max_rad     = -0.11f; // 最高的时候
    gimbal_cfg.pitch_min_rad     = -0.34f; // 最低的时候
    gimbal_cfg.yaw_max_rad       = 0.70f;
    gimbal_cfg.yaw_min_rad       = -0.70f;

    gimbal_cfg.pid.pitch_pos_pid = new pid_t(30.0f, 0.08f, 0.2f, 0.8f, 12);
    gimbal_cfg.pid.pitch_spd_pid = new pid_t(0.6f, 0.0f, 0.001f, 0.5f, 7.0f);

    gimbal_cfg.pid.yaw_pos_pid =
            new pyro::pid_t(40.0f, 0.0f, 0.0f, 0, 18.0f);
    gimbal_cfg.pid.yaw_spd_pid = new pyro::pid_t(0.7f, 0.0f, 0.0f, 0.2f, 3);

    gimbal_cfg.yaw_offset      = 3.10401011f;
}

extern "C"
{
    void aim2mcu_process()
    {
        if(aim2mcu_msg.data.target_id == 9)
        {
            gimbal_cmd_ptr->is_aiming = false;   
        }
        else
        {
            gimbal_cmd_ptr->is_aiming = true;
        }
        
        if_aim = aim2mcu_msg.data.fire;
        auto_fire                         = aim2mcu_msg.data.fire;
        if(abs(aim2mcu_msg.data.shoot_yaw) <= PI)
            gimbal_cmd_ptr->aim_imu_yaw_rad   = aim2mcu_msg.data.shoot_yaw;
        if(abs(aim2mcu_msg.data.shoot_pitch) <= PI / 2)
             gimbal_cmd_ptr->aim_imu_pitch_rad = aim2mcu_msg.data.shoot_pitch;
        aim_yaw = aim2mcu_msg.data.shoot_yaw;
        aim_pitch = aim2mcu_msg.data.shoot_pitch;
    }

    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        auto *dr16_driver = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
        read_scope_lock lockdr16(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        auto *vt03_driver = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03);
        read_scope_lock lockvt03(vt03_driver->get_lock());
        const auto *rc_data = 
            static_cast<vt03_drv_t::vt03_ctrl_t const*>(vt03_driver->read());

        if(vt03_driver->check_online())
        {
            if (rc_data->rc.gear.state == pyro::vt03_drv_t::gear_state_t::GEAR_LEFT)
            {
                gimbal_cmd_ptr->mode = gimbal_cmd_t::mode_t::PASSIVE;
                gimbal_cmd_ptr->target_delta_pitch_rad = 0.0f;
                gimbal_cmd_ptr->target_delta_yaw_rad   = 0.0f;
                autoaim                                = false;
                auto_fire = false; 
            }
            else if (rc_data->rc.gear.state == pyro::vt03_drv_t::gear_state_t::GEAR_MID)
            {
                gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
                gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::MANUAL;
                gimbal_cmd_ptr->target_delta_pitch_rad = rc_data->rc.ch_ry * 0.005f;
                gimbal_cmd_ptr->target_delta_yaw_rad   = rc_data->rc.ch_rx * 0.02f;
                autoaim                                = false;
                auto_fire = false;
            }
            else if (rc_data->rc.gear.state == pyro::vt03_drv_t::gear_state_t::GEAR_RIGHT)
            {
                gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
                gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::SCANNING;
                autoaim                     = true;
                aim2mcu_process();
            }

        }
        else if(dr16_driver->check_online())
        {
            if (dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
            {
                gimbal_cmd_ptr->mode = gimbal_cmd_t::mode_t::PASSIVE;
                gimbal_cmd_ptr->target_delta_pitch_rad = 0.0f;
                gimbal_cmd_ptr->target_delta_yaw_rad   = 0.0f;
                autoaim                                = false;
                auto_fire = false;
            }
            else if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state)
            {
                gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
                gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::MANUAL;
                gimbal_cmd_ptr->target_delta_pitch_rad = p_ctrl->rc.ch_ry * 0.005f;
                gimbal_cmd_ptr->target_delta_yaw_rad   = p_ctrl->rc.ch_rx * 0.02f;
                autoaim                                = false;
                auto_fire = false;
            }
            else if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
            {
                gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
                gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::SCANNING;
                autoaim                     = true;
                aim2mcu_process();
            }

        }
        
    }

    void chassis_rc2cmd(void const *rc_ctrl)
    {
        auto *dr16_driver = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);
        read_scope_lock lockdr16(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        auto *vt03_driver = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03);
        read_scope_lock lockvt03(vt03_driver->get_lock());
        const auto *rc_data = 
            static_cast<vt03_drv_t::vt03_ctrl_t const*>(vt03_driver->read());
        
        static int8_t vx        = 0;
        static int8_t vy        = 0;
        static int8_t wz        = 0;
        static int8_t delta_yaw = 0;
        static bool active      = false;
        static bool follow_yaw  = false;
        static bool nav_enable  = false;

        can_tx_drv_t::clear(0x123);

        if(vt03_driver->check_online())
        {
            if (rc_data->rc.gear.state == pyro::vt03_drv_t::gear_state_t::GEAR_LEFT)
            {
                vx         = 0;
                vy         = 0;
                wz         = 0;
                delta_yaw  = 0;
                follow_yaw = false;
                active     = false;
                nav_enable = false;
                if (waypoint_count > 0) first_manual_done = true;
            }
            else if (rc_data->rc.gear.state == pyro::vt03_drv_t::gear_state_t::GEAR_MID)
            {   
                if (abs(rc_data->rc.ch_ly) < 0.1f)
                    vx = 0;
                else
                    vx = static_cast<int8_t>(rc_data->rc.ch_ly * 127);
                if (abs(rc_data->rc.ch_lx) < 0.1f)
                    vy = 0;
                else
                    vy = static_cast<int8_t>(rc_data->rc.ch_lx * 127);
                wz         = 0;
                delta_yaw  = static_cast<int8_t>(rc_data->rc.ch_rx * 127);
                test_rx = static_cast<int8_t>(rc_data->rc.ch_rx * 127);
                follow_yaw = true;
                active     = true;
                nav_enable = false;

                if (!first_manual_done && waypoint_count == 0)
                {
                    // 在这里可连续多次调用 sentry_move_by_distance() 添加路点
                    sentry_move_by_distance(0.5f, 0.0f, 500);
                    // ... 可添加更多路点 ...
                    waypoint_load_next(); // 启动第一个路点
                }

                if(rc_data->rc.ch_ly == 0 &&
                    rc_data->rc.ch_lx == 0 &&
                    rc_data->rc.ch_rx == 0 &&
                    rc_data->rc.ch_ry == 0
                )
                {
                    if (abs(rc_data->mouse.x) < 0.1f)
                        vx = 0;
                    else
                        vx = static_cast<int8_t>(rc_data->rc.ch_ly * 127);
                    if (abs(rc_data->rc.ch_lx) < 0.1f)
                        vy = 0;
                    else
                        vy = static_cast<int8_t>(rc_data->rc.ch_lx * 127);
                    wz         = 0;
                    delta_yaw  = static_cast<int8_t>(rc_data->rc.ch_rx * 127);
                    test_rx = static_cast<int8_t>(rc_data->rc.ch_rx * 127);
                }
            }
            else if (rc_data->rc.gear.state == pyro::vt03_drv_t::gear_state_t::GEAR_RIGHT)
            {
                follow_yaw = false;
                active     = true;
                nav_enable = true;
            }
        }
        else if(dr16_driver->check_online())
        {
            if (dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
            {
                vx         = 0;
                vy         = 0;
                wz         = 0;
                delta_yaw  = 0;
                follow_yaw = false;
                active     = false;
                nav_enable = false;
                if (waypoint_count > 0) first_manual_done = true;
            }
            else if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_r.state)
            {
                if (abs(p_ctrl->rc.ch_lx) < 0.1f)
                    vx = 0;
                else
                    vx = static_cast<int8_t>(p_ctrl->rc.ch_lx * 127);
                if (abs(p_ctrl->rc.ch_ly) < 0.1f)
                    vy = 0;
                else
                    vy = static_cast<int8_t>(p_ctrl->rc.ch_ly * 127);
                wz         = 0;
                delta_yaw  = static_cast<int8_t>(p_ctrl->rc.ch_rx * 127);
                test_rx = static_cast<int8_t>(p_ctrl->rc.ch_rx * 127);
                follow_yaw = true;
                active     = true;
                nav_enable = false;

                if (!first_manual_done && waypoint_count == 0)
                {
                    sentry_move_by_distance(0.5f, 0.0f, 500);
                    waypoint_load_next();
                }
            }
            else if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
            {
                follow_yaw = false;
                active     = true;
                nav_enable = true;
            }
        }
        

        if (distance_move_active)
        {
            if (move_delay_counter > 0)
            {
                move_delay_counter--;
                vx = 0;
                vy = 0;
            }
            else
            {
            float traveled_x = current_odom_x - move_start_odom_x;
            float traveled_y = current_odom_y - move_start_odom_y;
            float remain_x   = move_target_sx - traveled_x;
            float remain_y   = move_target_sy - traveled_y;

            float speed_x = 0.0f, speed_y = 0.0f;
            if (fabsf(remain_x) > DISTANCE_TOLERANCE)
                speed_x =
                    (remain_x > 0.0f) ? DISTANCE_MOVE_SPEED : -DISTANCE_MOVE_SPEED;
            if (fabsf(remain_y) > DISTANCE_TOLERANCE)
                speed_y =
                    (remain_y > 0.0f) ? DISTANCE_MOVE_SPEED : -DISTANCE_MOVE_SPEED;

            vx = static_cast<int8_t>(speed_x / 2.0f * 127.0f);
            vy = static_cast<int8_t>(speed_y / 2.0f * 127.0f);

            if (fabsf(remain_x) <= DISTANCE_TOLERANCE &&
                fabsf(remain_y) <= DISTANCE_TOLERANCE)
            {
                vx = 0;
                vy = 0;
                waypoint_load_next(); // 加载下一个路点，队列空时自动结束
            }
            } // end else (move_delay_counter == 0)
        }

        can_tx_drv_t::add_data(0x123, 8, vx);
        can_tx_drv_t::add_data(0x123, 8, vy);
        can_tx_drv_t::add_data(0x123, 8, wz);
        can_tx_drv_t::add_data(0x123, 8, delta_yaw);
        can_tx_drv_t::add_data(0x123, 1, static_cast<uint8_t>(follow_yaw));
        can_tx_drv_t::add_data(0x123, 1, static_cast<uint8_t>(active));
        can_tx_drv_t::add_data(0x123, 1, static_cast<uint8_t>(nav_enable));

        can_tx_drv_t::send(0x123, can_hub_t::get_instance()->hub_get_can_obj(
                                      can_hub_t::which_can::can3));
    }

    void chassis2gimbal()
    {
        std::array<uint8_t, 8> raw_data{};
        can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x102, raw_data);
        uint8_t bullet_speed_int = raw_data[0];
        uint8_t bullet_speed_dec = raw_data[1];

        // 简单的弹速滤波: 只有当新弹速明显变化时才更新, 避免小幅波动引起的频繁调整
        if(bullet_speed_int + bullet_speed_dec / 100.0f > 10.0f)
        {
            last_bullet_speed = bullet_speed;
            bullet_speed             = bullet_speed_int + bullet_speed_dec / 100.0f;
        }

        in_aim = raw_data[2];
        game_started = raw_data[3] & 0x01;
        enemy_color = raw_data[3] >> 1 & 0x01;
        scan = raw_data[3] >> 2 & 0x01;
    }

    void chassis2gimbal_heat()
    {
        std::array<uint8_t, 8> raw_data{};
        can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x105, raw_data);
        power_heat   = raw_data[0] | (raw_data[1] << 8);
        heat_limit   = raw_data[2] | (raw_data[3] << 8);
        cooling_rate = raw_data[4] | (raw_data[5] << 8);
        int16_t yaw_scaled = raw_data[6] | (raw_data[7] << 8);
        big_yaw = static_cast<float>(yaw_scaled) / 10000.0f;
    }

    void chassis2gimbal_odom()
    {
        std::array<uint8_t, 8> raw_data{};
        if (can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x124, raw_data))
        {
            memcpy(&current_odom_x, raw_data.data(), 4);
            memcpy(&current_odom_y, raw_data.data() + 4, 4);
        }
    }

    void mcu2aim_process()
    {
        float yaw, pitch, roll;
        ins_drv_t *ins = ins_drv_t::get_instance();
        ins->get_angles_b(&yaw, &pitch, &roll);
        yaw                               = yaw / 180 * PI;
        pitch                             = pitch / 180 * PI;

        mcu2aim_msg.data.curr_yaw         = yaw;
        mcu2aim_msg.data.curr_pitch       = pitch;
        mcu2aim_msg.data.self_v_magnitude = 0;
        mcu2aim_msg.data.self_v_angle     = 0;
        mcu2aim_msg.data.curr_speed       = bullet_speed;
        mcu2aim_msg.data.shoot_delay      = 0;
        mcu2aim_msg.data.state            = in_aim;
        mcu2aim_msg.data.stop_record      = 0;
        mcu2aim_msg.data.autoaim          = autoaim;
        mcu2aim_msg.data.enemy_color      = enemy_color;
        mcu2aim_msg.data.big_yaw          = big_yaw;
        append_crc16_check_sum(reinterpret_cast<uint8_t *>(&mcu2aim_msg),
                               sizeof(mcu2aim_msg) - 1);
        comm->write(mcu2aim_msg);
    }

    void sentry_gimbal_thread(void *argument)
    {
        while (true)
        {
            comm->read(aim2mcu_msg);
            chassis2gimbal_odom();
            chassis_rc2cmd(rc_ctrl_ptr);
            gimbal_rc2cmd(rc_ctrl_ptr);
            chassis2gimbal();
            chassis2gimbal_heat();
            mcu2aim_process();
            gimbal_ptr->set_command(*gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }
    uint8_t header = 0xA5;
    status_t sentry_gimbal_init()
    {
        gimbal_cmd_ptr = new gimbal_cmd_t();
        gimbal_cfg_ptr = new gimbal_cfg_t();
        comm           = new uart_comm_t(uart_drv_t::which_uart::uart10, 0x01);

        mcu2aim_msg.header.sof  = 0xA5;
        mcu2aim_msg.enter.enter = '\n';

        comm->register_msg_type(sizeof(aim2mcu_msg), &header,
                                sizeof(aim2mcu_msg.header));

        can_rx_drv_t::subscribe(can_hub_t::which_can::can3, 0x124);

        gimbal_ptr = gimbal_t::instance();
        gimbal_config(*gimbal_cfg_ptr);
        gimbal_ptr->configure(*gimbal_cfg_ptr);
        gimbal_ptr->start();

        rc_ctrl_ptr = static_cast<dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(rc_hub_t::DR16)->read());
        xTaskCreate(sentry_gimbal_thread, "sentry_gimbal_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        return PYRO_OK;
    }
}

#endif