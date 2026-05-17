/**
 * @file pyro_bsp_uart.cpp
 * @brief Board-level UART singleton accessors.
 */

#include "pyro_bsp_uart.h"

#include "usart.h"

namespace pyro
{

uart_drv_t &bsp_uart::get_uart1()
{
    static uart_drv_t instance(&huart1, 512);
    return instance;
}

uart_drv_t &bsp_uart::get_uart5()
{
    static uart_drv_t instance(&huart5, 512);
    return instance;
}

uart_drv_t &bsp_uart::get_uart7()
{
    static uart_drv_t instance(&huart7, 512);
    return instance;
}

uart_drv_t &bsp_uart::get_uart10()
{
    static uart_drv_t instance(&huart10, 512);
    return instance;
}

void bsp_uart::init_all()
{
    get_uart1().enable_rx_dma();
    get_uart5().enable_rx_dma();
    get_uart7().enable_rx_dma();
    get_uart10().enable_rx_dma();
}

} // namespace pyro
