#include "pyro_algo_common.h"
#include "pyro_sentry_gimbal.h"

float yaw_cangle;
float yaw_tangle;
float yaw_ctorque;

namespace pyro
{
enum
{
    DOWN  = 0,
    UP    = 1,
    Right = 0,
    Left  = 1,
};

bool pitch_direction = UP;
bool yaw_direction   = Left;
bool mode_change = 1;

void gimbal_t::fsm_active_t::state_scanning_t::enter(owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.yaw->enable();
    owner->_ctx.gimbal_config.motor.pitch->enable();
}

void gimbal_t::fsm_active_t::state_scanning_t::execute(owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
        owner->_ctx.gimbal_config.motor.yaw->enable();
        owner->_ctx.gimbal_config.motor.pitch->enable();
    }
    else
    {
        if (owner->_ctx.cmd->is_aiming)
        {
            float yaw, pitch, roll;
            ins_drv_t *ins = ins_drv_t::get_instance();
            ins->get_rads_b(&yaw, &pitch, &roll);

            owner->_ctx.data.aim_imu_max_yaw =
                wrap_pi(yaw + (owner->_ctx.gimbal_config.yaw_max_rad -
                            owner->_ctx.data.current_yaw_rad));
            owner->_ctx.data.aim_imu_min_yaw =
                wrap_pi(yaw + (owner->_ctx.gimbal_config.yaw_min_rad -
                            owner->_ctx.data.current_yaw_rad));

            // ------------------没经过测试，但是我觉得这样就可以-----------------
            if (owner->_ctx.data.aim_imu_max_yaw <
            owner->_ctx.data.aim_imu_min_yaw)
            {
                owner->_ctx.data.aim_imu_max_yaw += 2 * PI;
            }

            owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->aim_imu_pitch_rad;
            owner->_ctx.data.target_yaw_rad   = owner->_ctx.cmd->aim_imu_yaw_rad;

            if (owner->_ctx.data.target_pitch_rad >
                PI / 6)
                owner->_ctx.data.target_pitch_rad =
                    PI / 6;
            if (owner->_ctx.data.target_pitch_rad <
                -PI / 6)
                owner->_ctx.data.target_pitch_rad =
                    -PI / 6;

            // if (owner->_ctx.data.aim_imu_max_yaw > owner->_ctx.data.aim_imu_min_yaw)
            // {
            //     owner->_ctx.data.target_yaw_rad = loop_fp32_constrain(
            //         owner->_ctx.data.target_yaw_rad, owner->_ctx.data.aim_imu_min_yaw,
            //         owner->_ctx.data.aim_imu_max_yaw);
            // }
            // else
            // {
            //     if (owner->_ctx.data.target_yaw_rad >
            //             owner->_ctx.data.aim_imu_max_yaw &&
            //         owner->_ctx.data.target_yaw_rad < owner->_ctx.data.aim_imu_min_yaw)
            //     {
            //         float dist_to_max = fabs(wrap_pi(owner->_ctx.data.target_yaw_rad -
            //                                          owner->_ctx.data.aim_imu_max_yaw));
            //         float dist_to_min = fabs(wrap_pi(owner->_ctx.data.target_yaw_rad -
            //                                          owner->_ctx.data.aim_imu_min_yaw));
            //         owner->_ctx.data.target_yaw_rad =
            //             dist_to_max < dist_to_min ? owner->_ctx.data.aim_imu_max_yaw
            //                                       : owner->_ctx.data.aim_imu_min_yaw;
            //     }
            // }
            mode_change = 0;
            _gimbal_imu_control(&owner->_ctx);
        }
        else
        {    
            if(mode_change == 0)
            {
                owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_rad;
                owner->_ctx.data.target_yaw_rad   = owner->_ctx.data.current_yaw_rad;
            }
            owner->_ctx.gimbal_config.motor.yaw->enable();
            owner->_ctx.gimbal_config.motor.pitch->enable();

            if (pitch_direction)
            {
                owner->_ctx.data.target_pitch_rad += 0.0002f;
                if (owner->_ctx.data.target_pitch_rad >=
                    owner->_ctx.gimbal_config.pitch_max_rad)
                    pitch_direction = DOWN;
            }
            else
            {
                owner->_ctx.data.target_pitch_rad -= 0.0002f;
                if (owner->_ctx.data.target_pitch_rad <=
                    owner->_ctx.gimbal_config.pitch_min_rad)
                    pitch_direction = UP;
            }

            owner->_ctx.data.target_yaw_rad = wrap_pi(owner->_ctx.data.target_yaw_rad);
            yaw_tangle                      = owner->_ctx.data.target_yaw_rad;
            yaw_cangle                      = owner->_ctx.data.current_yaw_rad;
            if (yaw_direction)
            {
                if (owner->_ctx.data.target_yaw_rad <
                    owner->_ctx.gimbal_config.yaw_max_rad)
                {
                    owner->_ctx.data.target_yaw_rad += 0.0005f; // 步进值可根据需求调整
                }
                else
                {
                    // 目标角度到达上限，切换为向左扫描
                    yaw_direction = Right;
                    // 钳制目标角度，避免超出限位（关键）
                    owner->_ctx.data.target_yaw_rad =
                        owner->_ctx.gimbal_config.yaw_max_rad;
                }
            }
            else
            {
                // 先判断目标角度是否即将低于最小值，再修改
                if (owner->_ctx.data.target_yaw_rad >
                    owner->_ctx.gimbal_config.yaw_min_rad)
                {
                    owner->_ctx.data.target_yaw_rad -= 0.0005f;
                }
                else
                {
                    // 目标角度到达下限，切换为向右扫描
                    yaw_direction = Left;
                    // 钳制目标角度，避免超出限位（关键）
                    owner->_ctx.data.target_yaw_rad =
                        owner->_ctx.gimbal_config.yaw_min_rad;
                }
            }
            mode_change = 1;
            _gimbal_mec_control(&owner->_ctx);
        }    
        _send_motor_command(&owner->_ctx);
    }
}

void gimbal_t::fsm_active_t::state_scanning_t::exit(owner *owner)
{
}

} // namespace pyro