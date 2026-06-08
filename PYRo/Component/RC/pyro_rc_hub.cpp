/**
* @file pyro_rc_hub.cpp
 * @brief Implementation of the PYRO RC Hub Factory.
 * PYRO 遥控器枢纽工厂实现文件。
 *
 * Implements the lazy initialization of RC drivers. It binds specific UART
 * hardware peripherals (e.g., UART5, UART1) to their corresponding software
 * drivers upon the first request.
 * 实现遥控器驱动的懒加载初始化。它在首次请求时将特定的 UART 硬件外设（如 UART5,
 * UART1）绑定到对应的软件驱动上。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-11-14
 * @copyright [Copyright Information Here]
 */

#include "pyro_rc_hub.h"

#include "pyro_core_config.h"

namespace pyro
{
/**
 * @brief Factory method: Returns the requested driver singleton.
 * 工厂方法：返回请求的驱动单例。
 *
 * Uses C++ "Magic Statics" to ensure thread-safe, lazy initialization.
 * The drivers and their required UART dependencies are constructed only once
 * when this method is first called for a specific type.
 * 使用 C++ "Magic Statics" 确保线程安全的懒加载初始化。驱动及其所需的 UART 依
 * 赖仅在首次调用该方法请求特定类型时构造一次。
 *
 * @param which_rc Driver type identifier. 驱动类型标识符。
 * @return rc_drv_t* Polymorphic pointer to the driver instance. 驱动实例的多态指针。
 */
rc_drv_t *rc_hub_t::get_instance(which_rc_t which_rc)
{
    switch (which_rc)
    {
        case DR16:
        {
            static uart_drv_t *dr16_uart =
                uart_drv_t::get_instance(static_cast<uart_drv_t::which_uart>(DR16_UART));
            static dr16_drv_t dr16_rc_drv(dr16_uart);
            return &dr16_rc_drv;
        }
        case VT03:
        {
            static uart_drv_t *vt03_uart =
                uart_drv_t::get_instance(static_cast<uart_drv_t::which_uart>(VT03_UART));
            static vt03_drv_t vt03_rc_drv(vt03_uart);
            return &vt03_rc_drv;
        }
        default:;
    }
    return nullptr;
}

} // namespace pyro