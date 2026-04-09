/**
 *******************************************************************************
 * @file    speed_compensater.cpp
 * @brief   简要描述
 *******************************************************************************
 * @attention
 *
 * none
 *
 *******************************************************************************
 * @note
 *
 * none
 *
 *******************************************************************************
 * @author  MekLi
 * @date    2026/3/14
 * @version 1.0
 *******************************************************************************
 */


/* ------- define ----------------------------------------------------------------------------------------------------*/





/* ------- include ---------------------------------------------------------------------------------------------------*/

#include "speed_compensater.h"




/* ------- class prototypes-------------------------------------------------------------------------------------------*/





/* ------- macro -----------------------------------------------------------------------------------------------------*/





/* ------- variables -------------------------------------------------------------------------------------------------*/





/* ------- function implement ----------------------------------------------------------------------------------------*/


void SpeedCompensator::update(float newInitialSpeed) {
    // 1. 判断弹速是否变化 (事件驱动：只有新子弹才触发计算)
    // 注意：使用浮点数绝对误差判断，防止极小精度问题导致的误判
    if (std::abs(_lastInitialSpeed - newInitialSpeed) < 0.01f) {
        return;
    }

    // 记录新弹速
    _lastInitialSpeed = newInitialSpeed;

    // 2. 过滤无效数据或异常子弹 (如卡弹、碎弹或测速模块乱码)
    if (newInitialSpeed < 15.0f || newInitialSpeed > 30.0f) {
        return;
    }

    // 3. 将有效弹速加入滑动平均滤波器，获取宏观趋势
    _filterBuffer[_filterIndex] = newInitialSpeed;
    _filterIndex                = (_filterIndex + 1) % FILTER_SIZE;
    if (_filterCount < FILTER_SIZE) {
        _filterCount++;
    }

    // 计算当前滑动窗口内的平均弹速
    float sum = 0.0f;
    for (uint8_t i = 0; i < _filterCount; ++i) {
        sum += _filterBuffer[i];
    }
    float averageSpeed = sum / _filterCount;

    // 4. 计算宏观误差 (目标弹速 - 平均真实弹速)
    float error        = _targetSpeed - averageSpeed;

    // 5. 引入防震荡死区 (Deadband)
    // 只有当误差大于死区阈值时才进行补偿，过滤掉弹丸本身的物理公差扰动
    if (std::abs(error) > _deadband) {
        // 离散积分补偿 (I 控制)
        // 误差为正(打慢了)，累加正值提升转速；误差为负(打快了)，累加负值降低转速
        _radsCompensation += _ki * error;
    }

    // 6. 严格限幅防暴走 (Absolute Clamp)
    if (_radsCompensation > _maxCompensation) {
        _radsCompensation = _maxCompensation;
    }
    if (_radsCompensation < -_maxCompensation) {
        _radsCompensation = -_maxCompensation;
    }
}


// 获取补偿后的最终角速度指令
float SpeedCompensator::getCompensatedRadPerSec(float baseRadPerSec) const { return baseRadPerSec + _radsCompensation; }

// 重置补偿器状态 (通常在摩擦轮关闭时调用)
void SpeedCompensator::reset() {
    _radsCompensation = 0.0f;
    _lastInitialSpeed = 0.0f;

    // 清空滤波器
    _filterIndex      = 0;
    _filterCount      = 0;
    for (float & i : _filterBuffer) {
        i = 0.0f;
    }
}