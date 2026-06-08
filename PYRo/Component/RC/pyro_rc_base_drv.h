/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-01-29 14:33:03
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-01-29 14:33:53
 * @FilePath: \PYRo-uCtrl-Unity-dev\PYRo\Component\RC\pyro_rc_base_drv.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file pyro_rc_base_drv.h
 * @brief Header file for the PYRO Remote Control (RC) Driver Base Class.
 * PYRO 遥控器驱动基类头文件。
 *
 * Defines the abstract interface `rc_drv_t` for RC protocols. It standardizes
 * lifecycle management (init, enable, disable), thread processing, and
 * priority-based arbitration between different receiver modules.
 * 定义遥控协议的抽象接口 `rc_drv_t`。标准化了生命周期管理（初始化、使能、禁用）、
 * 线程处理以及不同接收机模块基于优先级的仲裁机制。
 *
 * @author Lucky
 * @version 2.0.0
 * @date 2025-10-09
 */

#ifndef __PYRO_RC_BASE_DRV_H__
#define __PYRO_RC_BASE_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_rw_lock.h"
#include "pyro_uart_drv.h"
#include "task.h"
#include "message_buffer.h"

namespace pyro
{
class rc_hub_t;

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief Abstract base class for Remote Control (RC) drivers.
 * 遥控器驱动抽象基类。
 *
 * Establishes the contract for specific RC implementations (e.g., DR16, VT03).
 * Handles shared FreeRTOS resources (Tasks, MessageBuffers) and thread safety.
 * 为具体遥控实现（如 DR16, VT03）建立契约。处理共享的 FreeRTOS 资源（任务、
 * 消息缓冲区）及线程安全。
 */
class rc_drv_t
{
  public:
    /**
     * @brief Global sequence bitmask for protocol priority arbitration.
     * 用于协议优先级仲裁的全局序列位掩码。
     *
     * Used to determine which driver is currently "active" and has control.
     * 用于确定当前哪个驱动处于“活动”状态并拥有控制权。
     */
    inline static uint8_t sequence = 0x80;

    /* Public Methods - Pure Virtual Interface
     * ---------------------------------*/
    virtual void enable()          = 0;
    virtual void disable()         = 0;
    virtual void thread()          = 0;

    [[nodiscard]] bool check_online() const;
    [[nodiscard]] void const *read() const;
    [[nodiscard]] rw_lock &get_lock() const;

  protected:
    virtual status_t init()        = 0;
    /**
     * @brief ISR Callback: Handles raw UART data ingestion.
     * ISR 回调：处理原始 UART 数据摄入。
     *
     * Implementations must filter data and send it to the processing task via
     * the message buffer.
     * 实现类须过滤数据并通过消息缓冲区将其发送至处理任务。
     *
     * @param buf Raw data buffer pointer. 原始数据缓冲指针。
     * @param len Length of received data. 接收数据长度。
     * @param xHigherPriorityTaskWoken FreeRTOS context switch flag.
     * @return true if data was accepted. 如果数据被接收则返回 true。
     */
    virtual bool rc_callback(uint8_t *buf, uint16_t len,
                             BaseType_t& xHigherPriorityTaskWoken) = 0;

    /* Protected Members - Resources and State
     * ---------------------------------*/
    explicit rc_drv_t(uart_drv_t *uart);
    virtual ~rc_drv_t();

    void const *_rc_data{};                 ///< Pointer to decoded struct.
    rw_lock *_lock{};                       ///< Thread-safe access lock.
    MessageBufferHandle_t _rc_msg_buffer{}; ///< Buffer from ISR to Task.
    TaskHandle_t _rc_task_handle{};         ///< Parsing task handle.
    uart_drv_t *_rc_uart;                   ///< Underlying hardware driver.
    uint8_t _priority{};                    ///< Arbitration priority index.
};
} // namespace pyro

#endif