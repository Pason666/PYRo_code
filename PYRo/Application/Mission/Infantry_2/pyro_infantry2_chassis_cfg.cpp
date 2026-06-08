/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-28 13:05:25
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
//the cofficients for phi calculation
#define PHI_K0 22506.0f
#define PHI_K1 475976946.0f
#define PHI_K2 296955904.0f
#define PHI_K3 179021042.0f
#define PHI_K4 18922.0f
//the cofficients for position in polar coordinates
#define POLAR_K0 236976927.0f
#define POLAR_K1 21059.0f
#define POLAR_K2 946100000.0f
#define POLAR_K3 100000.0f
//the cofficients for VMC transformation matrix
#define TRANS_K0 21059.0f
#define TRANS_K1 100000.0f
//the offsets of motors, which is the angle between the zero point of motor and
// the forward direction of robot, counter clockwise is positive(rad)
#define R_MOTOR1_OFFSET 2.66f
#define R_MOTOR2_OFFSET 3.979092653f
#define L_MOTOR1_OFFSET 0.0f
#define L_MOTOR2_OFFSET 0.0f
// the cofficients of lqr gain, 36 values in total, every value has 3 cofficients,
#define LQR_GAIN 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,\
                 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
#include "pyro_wl_chassis.h"
using namespace pyro;

float infantry2_lqr_coef[36] = {LQR_GAIN};
wl_chassis_cfg_t infantry2_chassis_cfg = {
    .phi_k = {
        .k0 = PHI_K0,
        .k1 = PHI_K1,
        .k2 = PHI_K2,
        .k3 = PHI_K3,
        .k4 = PHI_K4,
    },
    .polar_k = {
        .k0 = POLAR_K0,
        .k1 = POLAR_K1,
        .k2 = POLAR_K2,
        .k3 = POLAR_K3,
    },
    .vmc_k = {
        .k0 = TRANS_K0,
        .k1 = TRANS_K1,
    },
    .joint_motor_cfg = {
        {
            .tx_id = 0x01,
            .rx_id = 0x11,
            .can = can_hub_t::can1,
            .offset_angle = R_MOTOR1_OFFSET,
        },
        {
            .tx_id = 0x02,
            .rx_id = 0x12,
            .can = can_hub_t::can1,
            .offset_angle = R_MOTOR2_OFFSET,
        },
        {
            .tx_id = 0x03,
            .rx_id = 0x13,
            .can = can_hub_t::can2,
            .offset_angle = L_MOTOR1_OFFSET,
        },
        {
            .tx_id = 0x04,
            .rx_id = 0x14,
            .can = can_hub_t::can2,
            .offset_angle = L_MOTOR2_OFFSET,
        }
    },
    .wheel_motor_cfg = {
        {
            .tx_id = dji_motor_tx_frame_t::id_1,
            .can = can_hub_t::can1,
        },
        {
            .tx_id = dji_motor_tx_frame_t::id_2,
            .can = can_hub_t::can2,
        }
    },
    .T_pid_cfg = {
        {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
    .F_pid_cfg = {
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
    .lqr_coef = infantry2_lqr_coef,
    .wheel_radius = 0.076f,
    .reduction_ratio = 36.0f,
    .rotate_min = -45.0f,
    .rotate_max = 45.0f,
    .position_min = -12.5f,
    .position_max = 12.5f,
    .torque_min = -54.0f,
    .torque_max = 54.0f,
};