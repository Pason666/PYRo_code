/**
* @file pyro_rc_base_drv.cpp
 * @brief Implementation of the PYRO RC Driver Base Class.
 * PYRO 遥控器驱动基类实现文件。
 *
 * Handles the construction and destruction of shared resources, ensuring
 * proper cleanup of FreeRTOS objects to prevent memory leaks.
 * 处理共享资源的构造与析构，确保 FreeRTOS 对象的正确清理以防止内存泄漏。
 *
 * @author Lucky
 * @version 2.0.0
 * @date 2025-10-09
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_rc_base_drv.h"
#include "task.h"
#include <cstring>

namespace pyro
{

/* Constructor ---------------------------------------------------------------*/
/**
 * @brief Initialize base driver and set default sequence state.
 * 初始化基础驱动并设置默认序列状态。
 */
rc_drv_t::rc_drv_t(uart_drv_t *uart)
{
    _rc_uart = uart;
    sequence = 0x80;
}

rw_lock &rc_drv_t::get_lock() const
{
    return *_lock;
}

/**
 * @brief Check if this specific driver instance is currently active.
 * 检查当前驱动实例是否处于活动状态。
 */
bool rc_drv_t::check_online() const
{
    return sequence >> _priority & 0x01;
}

void const *rc_drv_t::read() const
{
    return _rc_data;
}

/* Destructor ----------------------------------------------------------------*/
/**
 * @brief Cleanup FreeRTOS resources (MessageBuffers and Tasks).
 * 清理 FreeRTOS 资源（消息缓冲区和任务）。
 */
rc_drv_t::~rc_drv_t()
{
    if (_rc_msg_buffer)
    {
        vMessageBufferDelete(_rc_msg_buffer);
        _rc_msg_buffer = nullptr;
    }

    if (_rc_task_handle)
    {
        vTaskDelete(_rc_task_handle);
        _rc_task_handle = nullptr;
    }
    delete _lock;
}
} // namespace pyro