/**
 * @file pyro_bsp_uart.h
 * @brief Board-level UART singleton accessors.
 */

#ifndef __PYRO_BSP_UART_H__
#define __PYRO_BSP_UART_H__

#include "pyro_uart_drv.h"

namespace pyro
{

class bsp_uart
{
  public:
    static uart_drv_t &get_uart1();
    static uart_drv_t &get_uart5();
    static uart_drv_t &get_uart7();
    static uart_drv_t &get_uart10();

    static void init_all();
};

} // namespace pyro

#endif // __PYRO_BSP_UART_H__
