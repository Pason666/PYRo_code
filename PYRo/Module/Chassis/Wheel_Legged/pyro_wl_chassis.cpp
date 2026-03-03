/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-03-01 15:41:26
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

 #include "pyro_wl_chassis.h"


 namespace pyro
 {
wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 0, 512)
{
}


status_t wl_chassis_t::_init()
{
    status_t ret;

    /* Initialize kinematic solver with given coefficients. */
    ret = _kinematic_solver.init(&_module_deps.phi_k, 
             &_module_deps.polar_k, &_module_deps.vmc_k);
    CHECK_PYRO_RET(ret);
    /* Save LQR coefficients */
    memcpy(_lqr_cof, _module_deps.lqr_coef, sizeof(float) * 36);

    /* Save wheel radius and reduction ratio */
    _wheel_radius = _module_deps.wheel_radius;
    _reduction_ratio = _module_deps.reduction_ratio;

    /* Initialize joint motor driver */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i] = new dm_motor_drv_t(_module_deps.joint_motor_cfg[i].tx_id,
                                           _module_deps.joint_motor_cfg[i].rx_id,
                                           _module_deps.joint_motor_cfg[i].can);
        if(!_motor_drv[i])
        {
            return PYRO_NO_MEMORY;
        }
        _motor_offset[i] = _module_deps.joint_motor_cfg[i].offset_angle;
        _motor_drv[i]->set_rotate_range(_module_deps.rotate_min, 
                                                  _module_deps.rotate_max);
        _motor_drv[i]->set_position_range(_module_deps.position_min,
                                                  _module_deps.position_max);
        _motor_drv[i]->set_torque_range(_module_deps.torque_min,
                                                  _module_deps.torque_max);
    }
    /* Initialize wheel motor driver */
    for(uint8_t i = 0; i < 2; i++)
    {
        _wheel_drv[i] = 
               new dji_m3508_motor_drv_t(_module_deps.wheel_motor_cfg[i].tx_id,
                                     _module_deps.wheel_motor_cfg[i].can);
        if(!_wheel_drv[i])
        {
            return PYRO_NO_MEMORY;
        }
    }
    /* Initialize VMC matrix */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_init_f32(&_leg_data[i].T_mat, 2, 2, 
                                            _leg_data[i].T_mat_val);
    }
    /* Initialize PID controllers */
    for(uint8_t i = 0; i < 2; i++)
    {
        _T_pid[i] = new pid_t(_module_deps.T_pid_cfg[i].kp, _module_deps.T_pid_cfg[i].ki, 
                            _module_deps.T_pid_cfg[i].kd, 
                            _module_deps.T_pid_cfg[i].integral_limit,
                            _module_deps.T_pid_cfg[i].max_out);
        if(!_T_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
        _F_pid[i] = new pid_t(_module_deps.F_pid_cfg[i].kp, _module_deps.F_pid_cfg[i].ki, 
                            _module_deps.F_pid_cfg[i].kd, 
                            _module_deps.F_pid_cfg[i].integral_limit,
                            _module_deps.F_pid_cfg[i].max_out);
        if(!_F_pid[i])        
        {
            return PYRO_NO_MEMORY;
        }
    }
    /* get INS drv */
    _ins_drv = ins_drv_t::get_instance();

    return ret;
}

void wl_chassis_t::_update_feedback()
{
    /* Update INS data */
    if(_ins_drv)
    {
        _ins_drv->get_rads_b(&yaw, &pitch, &roll);
        _ins_drv->get_gyro_b(&g_yaw, &g_pitch, &g_roll);
    }
    /* Update the feedback of joint motors and wheel motors. */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i]->update_feedback();
    }
    /* angle theta takes clockwise as positive, with the forward direction as
       the fixed side. When the legged overlap with forward direction, the value
       of theta is 0, so we need to add offset to fix the installation bias. 
       Due to the direction of right motor is opposite to the direction of 
       theta, we need to invert the sign for the right leg motors. */
    _leg_data[R].theta1 = -_motor_drv[RF]->get_current_position() 
                                                        + _motor_offset[RF];
    _leg_data[R].theta2 = -_motor_drv[RB]->get_current_position() 
                                                        + _motor_offset[RB];
    _leg_data[L].theta1 = _motor_drv[LF]->get_current_position() 
                                                        + _motor_offset[LF];
    _leg_data[L].theta2 = _motor_drv[LB]->get_current_position() 
                                                        + _motor_offset[LB];
    for(uint8_t i = 0; i < 2; i++)
    {
        _wheel_drv[i]->update_feedback();
        _leg_data[i].dx = _wheel_drv[i]->get_current_rotate() * _wheel_radius 
                                        * _reduction_ratio;
        _leg_data[i].x += _leg_data[i].dx * 0.001f;
    }

    /* kinematic solve the current states of the chassis */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;

        ret = _kinematic_solver.solve(_leg_data[i].theta1, 
                                      _leg_data[i].theta2,
                                        &_leg_data[i].phi1,
                                        &_leg_data[i].phi2,
                                      &_leg_data[i].alpha,
                                       &_leg_data[i].l);
        if(ret != PYRO_OK)
        {
            _cnt.solver_error++;
        }
    }

    /* update VMC matrix */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;

        ret = _kinematic_solver.get_VMC_value(_leg_data[i].theta1, 
                                             _leg_data[i].theta2,
                                             _leg_data[i].phi1,
                                             _leg_data[i].phi2,
                                             _leg_data[i].l,
                                             _leg_data[i].alpha,
                                             _leg_data[i].T_mat.pData);
        if(ret != PYRO_OK)
        {
            _cnt.solver_error++;
        }
        _leg_data[i].beta = PI / 2 - _leg_data[i].alpha - pitch;
    }

}

void wl_chassis_t::_fsm_execute()
{
    _cmd = &_current_cmd;
    if (cmd_base_t::mode_t::PASSIVE == _cmd->mode)
        _fsm.change_state(&_state_passive)  ;
    else if (cmd_base_t::mode_t::ACTIVE == _cmd->mode)
        _fsm.change_state(&_state_active);
    _fsm.execute(this);
}
 }