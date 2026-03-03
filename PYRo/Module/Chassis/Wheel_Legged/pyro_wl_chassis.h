/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-07 15:14:47
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-28 13:13:15
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_kin.wl.h"

#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"
#include "pyro_algo_pid.h"

namespace pyro
{
/* DM joint motor configuration structure. */
struct wl_dm_motor_cfg_t
{
    /* can tx id of motor*/
    uint8_t tx_id;
    /* can rx id of motor*/
    uint8_t rx_id;
    /* Which CAN bus the motor is connected to */
    can_hub_t::which_can can;
    /* Offset angle to eliminate the bias of installation(rad)*/
    float offset_angle;
};

/* DJI 3508 configuration structure.*/
struct wl_dji_motor_cfg_t
{
    /* can tx id of motor*/
    dji_motor_tx_frame_t::register_id_t tx_id;
    /* Which CAN bus the motor is connected to */
    can_hub_t::which_can can;
};

struct wl_pid_cfg_t
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float max_out;
};

/* Configuration structure for wheel-legged chassis, every parameter
   should be set here */
struct wl_chassis_cfg_t
{
    /* Kinematic coefficients for the chassis. */
    wheel_legged_kin_t::phi_k_t phi_k;
    wheel_legged_kin_t::polar_k_t polar_k;
    wheel_legged_kin_t::vmc_k_t vmc_k;

    /* Joint motor configuration. The order of array is: front-right,
       rear-right, front-left, rear-left */
    wl_dm_motor_cfg_t joint_motor_cfg[4];
    /* Wheel motor configuration. The order of array is: right wheel, left
       wheel */
    wl_dji_motor_cfg_t wheel_motor_cfg[2];
    /* PID configuration for the chassis control. The order of array is: 
right leg, left leg */
    wl_pid_cfg_t T_pid_cfg[2];
    wl_pid_cfg_t F_pid_cfg[2];
    /* LQR coefficients for the chassis control. 2 raw x 6 column, 12 values
       in total. Every value has 3 coefficients.*/
    float *lqr_coef;
    /* Wheel radius(m)*/
    float wheel_radius;
    /* Reduction ratio of the motor */
    float reduction_ratio;
    /* Rotation range of the motor(rad/s)*/
    float rotate_min;
    float rotate_max;
    /* Position range of the motor(rad)*/
    float position_min;
    float position_max;
    /* Torque range of the motor(Nm)*/
    float torque_min;
    float torque_max;
};
//command structure for wheel-legged chassis
struct wl_cmd_t final : public cmd_base_t
{
    /* Linear velocity in x direction(m/s), positive toward the front of chassis
       negative toward the back of chassis */
    float vx;   
    /* Linear velocity in y direction(m/s), positive toward the left of chassis
       negative toward the right of chassis */
    float vy;  
    /* Angular velocity in z direction(rad/s), positive for counter-clockwise
       negative for clockwise */
    float vz;
    /* Leg length of both sides after normalization, value is between 0 and 1*/
    float l_leg;
    float r_leg;

    float l_angle;
    float r_angle;
    uint8_t active_mode; // 0 for normal mode, 1 for test mode, other value is reserved

    /* Construct function, set zero values */
    wl_cmd_t() : vx(0), vy(0), vz(0), l_leg(0), r_leg(0),l_angle(0), r_angle(0), active_mode(0)
    {
    }
};

class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_cmd_t, wl_chassis_cfg_t>
{
   friend class module_base_t<wl_chassis_t, wl_cmd_t, wl_chassis_cfg_t>;

public:
   
    wl_chassis_t(const wl_chassis_t &)            = delete;
    wl_chassis_t &operator=(const wl_chassis_t &) = delete;

private:
    /**
     * @description:
       Construct function, initialize the kinematic solver with given 
       cofficients. 
    *  @param {cfg_t*} cfg: configuration structure pointer, all parameters for 
       chassis should be set in this structure.
     */
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    /* base interface define */
    /**
     * @description: 
       Initialize the wheel-legged chassis module, this function will be called in
       the function 'start', if this function not return PYRO_OK, function 'start'
       will not create main tread of this module.
     * @return {status_t} 
       PYRO_OK if initialize successfully, otherwise return error code.
     */
    status_t _init() override;
    /**
     * @description: 
       Update the feedback of the chassis, include the data of ins and motor. 
       Call function '_kinematics_solve' to calculate the current states of 
       the chassis, if the kinematic solver not return PYRO_OK, cnt 
       'solver_error' will add 1. This function will be called in the main tread
       of this module.
     * @return {*}
     */
    void _update_feedback() override;
    void _fsm_execute() override;

    /* private function*/

    /* the ID of motor array */
    enum
    {
        RF = 0,
        RB = 1,
        LF = 2,
        LB = 3,
    };
    /* the ID of leg array */
    enum
    {
        R = 0,
        L = 1,
    };

    /* Kinematic solver, provide kinematic calculations and VMC matrix update
       function. */
    wheel_legged_kin_t _kinematic_solver;

    /* INS Drv */
    ins_drv_t *_ins_drv;

    float yaw, pitch, roll;
    float g_yaw, g_pitch, g_roll;

    /* LQR coefficients for the chassis control. 2 raw x 6 column, 12 values
       in total. Every value has 3 coefficients.*/
    float _lqr_cof[36];

    /* DM joint motors driver, the order of array is: front-right, rear-right, 
       front-left, rear-left */
    dm_motor_drv_t *_motor_drv[4];

    /* DJI wheel motors driver, the order of array is: right wheel, left wheel*/
    dji_m3508_motor_drv_t *_wheel_drv[2];
    /* Linear velocity of wheel equals angular velocity of wheel x wheel radius
       x reduction ratio */
    float _wheel_radius;
    float _reduction_ratio;
    /* Offset angle to eliminate the bias of installation(rad), order is same
       as _motor_drv array */
    float _motor_offset[4];
    /* data of each leg */
    struct leg_data_t
    {
        /* Angle between big rod and direction of movement(rad) */
        float theta1, theta2;
        /* Angle between little rod and direction of movement(rad) */
        float phi1, phi2;
        /* Polar radius of j9(m) */
        float l;
        /* Polar angle of j9(rad), clockwise is positive with forward direction
           as fixed side. */
        float alpha;
        /* The angle between the vertical direction(towards ground) and the 
           leg(rad), it equal pi/2 - alpha - pitch angle of chassis */
        float beta;
        /* The angle between the ground and the body(rad) */
        float gamma;
        /* Displacement distance of a j9(m) */
        float x;
        /* Velocity of the displacement of j9(m/s) */
        float dx;
        /* VMC transfotm matrix */
        arm_matrix_instance_f32 T_mat;
        /* VMC transform matrix value, the order is [T11, T12, T21, T22] */
        float T_mat_val[4];
        /* Target torque of motors, front is 0, rear is 1 */
        float T[2];
        /* VMC output force and torque, force is 0, torque is 1*/
        float F[2];
    } _leg_data[2];
    pid_t *_T_pid[2];
    pid_t *_F_pid[2];

    /* CAN bus configuration for motors, the order is same as _motor_drv array */
    struct
    {
        uint16_t solver_error;
    }_cnt;

    class fsm_active_t : public fsm_t<wl_chassis_t>
    {
    public:
        class state_test_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_test;
        class state_normal_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_normal;
        void on_enter(wl_chassis_t *owner) override;
        void on_execute(wl_chassis_t *owner) override;
        void on_exit(wl_chassis_t *owner) override;
    }_state_active;

    class state_passive_t : public state_t<wl_chassis_t>
    {
    public:
        void enter(wl_chassis_t *owner) override;
        void execute(wl_chassis_t *owner) override;
        void exit(wl_chassis_t *owner) override;
    }_state_passive;

    friend class fsm_active_t;
    friend class state_passive_t;
    fsm_t<wl_chassis_t> _fsm;
    wl_cmd_t *_cmd;
};

}
#endif // __PYRO_WL_CHASSIS_H__