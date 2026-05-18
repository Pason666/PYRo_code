/**
 * @file pyro_uart_drv.h
 * @brief C++ UART driver with DMA RX, DMA TX, and ISR callback dispatch.
 */

#ifndef __PYRO_UART_DRV_H__
#define __PYRO_UART_DRV_H__

#include "FreeRTOS.h"
#include "pyro_core_def.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#include <functional>
#include <map>
#include <vector>

namespace pyro
{

class bsp_uart;

class uart_drv_t
{
    friend class bsp_uart;

  public:
    using rx_event_func = std::function<bool(
        uint8_t *p, uint16_t size, BaseType_t &xHigherPriorityTaskWoken)>;
    using tx_cplt_func =
        std::function<void(BaseType_t &xHigherPriorityTaskWoken)>;

    struct rx_event_callback_t
    {
        uint32_t owner;
        rx_event_func func;
    };

    struct state_t
    {
        volatile uint8_t init_flag      : 1;
        volatile uint8_t tx_busy        : 1;
        volatile uint8_t tx_timeout     : 1;
        volatile uint8_t rx_dma_enable  : 1;
        volatile uint8_t rx_busy        : 1;
        volatile uint8_t rx_error       : 1;
        volatile uint8_t pin_swapped    : 1;
        volatile uint8_t level_inverted : 1;
    };

    uart_drv_t(const uart_drv_t &)            = delete;
    uart_drv_t &operator=(const uart_drv_t &) = delete;
    uart_drv_t(uart_drv_t &&)                 = delete;
    uart_drv_t &operator=(uart_drv_t &&)      = delete;
    ~uart_drv_t();

    status_t reset(uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits,
                   uint32_t Parity);
    status_t set_pin_swap(bool enable);
    status_t set_level_invert(bool tx_invert, bool rx_invert);

    status_t write(const uint8_t *p, uint16_t size, uint32_t waittime);
    status_t write(const uint8_t *p, uint16_t size);

    status_t enable_rx_dma();
    status_t disable_rx_dma();

    void add_rx_event_callback(const rx_event_func &func, uint32_t owner);
    status_t remove_rx_event_callback(uint32_t owner);

    void set_tx_cplt_callback(const tx_cplt_func &func)
    {
        _tx_cplt_callback = func;
    }

    status_t register_event_callback(pUART_RxEventCallbackTypeDef pCallback) const;
    status_t unregister_event_callback() const;
    status_t register_callback(HAL_UART_CallbackIDTypeDef CB_ID,
                               pUART_CallbackTypeDef pCallback) const;
    status_t unregister_callback(HAL_UART_CallbackIDTypeDef CB_ID) const;

    static std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_map();

    std::vector<rx_event_callback_t> rx_event_callbacks;
    tx_cplt_func _tx_cplt_callback = nullptr;
    uint8_t *rx_buf[2]{};
    uint8_t rx_buf_switch{};
    state_t state{};

  private:
    explicit uart_drv_t(UART_HandleTypeDef *huart, uint16_t buf_length);

    UART_HandleTypeDef *_huart;
    uint16_t _rx_buf_size{};
};

} // namespace pyro

#endif // __PYRO_UART_DRV_H__
