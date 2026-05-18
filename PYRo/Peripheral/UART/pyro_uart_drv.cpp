/**
 * @file pyro_uart_drv.cpp
 * @brief UART driver implementation.
 */

#include "pyro_uart_drv.h"

#include "dma.h"
#include "pyro_core_dma_heap.h"
#include "stm32h7xx_hal_dma.h"

#include <cstring>
#include <map>

namespace pyro
{

uart_drv_t::uart_drv_t(UART_HandleTypeDef *huart, const uint16_t buf_length)
    : _huart(huart)
{
    uart_map()[huart] = this;

    rx_buf[0] = static_cast<uint8_t *>(pvPortDmaMalloc(buf_length));
    rx_buf[1] = static_cast<uint8_t *>(pvPortDmaMalloc(buf_length));

    if (rx_buf[0] && rx_buf[1])
    {
        state.init_flag = true;
        std::memset(rx_buf[0], 0, buf_length);
        std::memset(rx_buf[1], 0, buf_length);
        _rx_buf_size = buf_length;
    }
}

uart_drv_t::~uart_drv_t()
{
    disable_rx_dma();

    if (rx_buf[0])
    {
        vPortFree(rx_buf[0]);
        rx_buf[0] = nullptr;
    }

    if (rx_buf[1])
    {
        vPortFree(rx_buf[1]);
        rx_buf[1] = nullptr;
    }

    uart_map().erase(_huart);
}

std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_drv_t::uart_map()
{
    static std::map<UART_HandleTypeDef *, uart_drv_t *> instance;
    return instance;
}

status_t uart_drv_t::write(const uint8_t *p, const uint16_t size,
                           const uint32_t waittime)
{
    const uint8_t ret = HAL_UART_Transmit(_huart, p, size, waittime);
    if (ret == HAL_OK)
    {
        state.tx_busy    = 0;
        state.tx_timeout = 0;
        return PYRO_OK;
    }

    if (ret == HAL_BUSY)
    {
        state.tx_busy = 1;
        return PYRO_BUSY;
    }

    if (ret == HAL_TIMEOUT)
    {
        state.tx_timeout = 1;
        return PYRO_TIMEOUT;
    }

    return PYRO_ERROR;
}

status_t uart_drv_t::write(const uint8_t *p, const uint16_t size)
{
    const uint8_t ret = HAL_UART_Transmit_DMA(_huart, p, size);
    if (ret == HAL_OK)
    {
        state.tx_busy    = 1;
        state.tx_timeout = 0;
        return PYRO_OK;
    }

    if (ret == HAL_BUSY)
    {
        state.tx_busy = 1;
        return PYRO_BUSY;
    }

    return PYRO_ERROR;
}

__attribute__((section(".itcm_text"))) status_t uart_drv_t::enable_rx_dma()
{
    if (!state.init_flag || !rx_buf[rx_buf_switch])
    {
        return PYRO_ERROR;
    }

    const uint8_t ret = HAL_UARTEx_ReceiveToIdle_DMA(
        _huart, rx_buf[rx_buf_switch], _rx_buf_size);

    if (ret != HAL_OK)
    {
        state.rx_dma_enable = 0;
        if (ret == HAL_BUSY)
        {
            state.rx_busy = 1;
        }
        else
        {
            state.rx_error = 1;
        }
        return PYRO_ERROR;
    }

    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
    state.rx_dma_enable = 1;
    state.rx_error      = 0;
    state.rx_busy       = 0;
    return PYRO_OK;
}

status_t uart_drv_t::disable_rx_dma()
{
    if (!state.rx_dma_enable)
    {
        return PYRO_OK;
    }

    if (HAL_UART_AbortReceive(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    state.rx_dma_enable = 0;
    return PYRO_OK;
}

status_t uart_drv_t::reset(const uint32_t BaudRate, const uint32_t WordLength,
                           const uint32_t StopBits, const uint32_t Parity)
{
    if (disable_rx_dma() != PYRO_OK)
    {
        return PYRO_ERROR;
    }

    _huart->Init.BaudRate   = BaudRate;
    _huart->Init.WordLength = WordLength;
    _huart->Init.StopBits   = StopBits;
    _huart->Init.Parity     = Parity;

    if (HAL_UART_DeInit(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    if (HAL_UART_Init(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    __HAL_UART_CLEAR_FLAG(_huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                      UART_CLEAR_NEF | UART_CLEAR_OREF |
                                      UART_CLEAR_RTOF | UART_CLEAR_CMF |
                                      UART_CLEAR_WUF);
    return PYRO_OK;
}

status_t uart_drv_t::set_pin_swap(const bool enable)
{
    if (disable_rx_dma() != PYRO_OK)
    {
        return PYRO_ERROR;
    }

    _huart->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_SWAP_INIT;
    _huart->AdvancedInit.Swap =
        enable ? UART_ADVFEATURE_SWAP_ENABLE : UART_ADVFEATURE_SWAP_DISABLE;

    if (HAL_UART_DeInit(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    if (HAL_UART_Init(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    state.pin_swapped = enable ? 1 : 0;
    return PYRO_OK;
}

status_t uart_drv_t::set_level_invert(const bool tx_invert,
                                      const bool rx_invert)
{
    if (disable_rx_dma() != PYRO_OK)
    {
        return PYRO_ERROR;
    }

    _huart->AdvancedInit.AdvFeatureInit |=
        (UART_ADVFEATURE_TXINVERT_INIT | UART_ADVFEATURE_RXINVERT_INIT);
    _huart->AdvancedInit.TxPinLevelInvert =
        tx_invert ? UART_ADVFEATURE_TXINV_ENABLE : UART_ADVFEATURE_TXINV_DISABLE;
    _huart->AdvancedInit.RxPinLevelInvert =
        rx_invert ? UART_ADVFEATURE_RXINV_ENABLE : UART_ADVFEATURE_RXINV_DISABLE;

    if (HAL_UART_DeInit(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    if (HAL_UART_Init(_huart) != HAL_OK)
    {
        return PYRO_ERROR;
    }

    state.level_inverted = (tx_invert || rx_invert) ? 1 : 0;
    return PYRO_OK;
}

void uart_drv_t::add_rx_event_callback(const rx_event_func &func,
                                       const uint32_t owner)
{
    remove_rx_event_callback(owner);
    rx_event_callbacks.push_back({owner, func});
}

status_t uart_drv_t::remove_rx_event_callback(const uint32_t owner)
{
    for (auto it = rx_event_callbacks.begin(); it != rx_event_callbacks.end();
         ++it)
    {
        if (it->owner == owner)
        {
            rx_event_callbacks.erase(it);
            return PYRO_OK;
        }
    }

    return PYRO_NOT_FOUND;
}

status_t uart_drv_t::register_event_callback(
    const pUART_RxEventCallbackTypeDef pCallback) const
{
    return HAL_UART_RegisterRxEventCallback(_huart, pCallback) == HAL_OK
               ? PYRO_OK
               : PYRO_ERROR;
}

status_t uart_drv_t::unregister_event_callback() const
{
    return HAL_UART_UnRegisterRxEventCallback(_huart) == HAL_OK ? PYRO_OK
                                                                : PYRO_ERROR;
}

status_t
uart_drv_t::register_callback(const HAL_UART_CallbackIDTypeDef CB_ID,
                              const pUART_CallbackTypeDef pCallback) const
{
    return HAL_UART_RegisterCallback(_huart, CB_ID, pCallback) == HAL_OK
               ? PYRO_OK
               : PYRO_ERROR;
}

status_t
uart_drv_t::unregister_callback(const HAL_UART_CallbackIDTypeDef CB_ID) const
{
    return HAL_UART_UnRegisterCallback(_huart, CB_ID) == HAL_OK ? PYRO_OK
                                                                : PYRO_ERROR;
}

} // namespace pyro

extern "C" __attribute__((section(".itcm_text"))) void
HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv = it->second;
        for (auto &cb : drv->rx_event_callbacks)
        {
            if (cb.func(drv->rx_buf[drv->rx_buf_switch], Size,
                        xHigherPriorityTaskWoken))
            {
                drv->rx_buf_switch ^= 0x01U;
                break;
            }
        }

        drv->enable_rx_dma();
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

extern "C" __attribute__((section(".itcm_text"))) void
HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                         UART_CLEAR_NEF | UART_CLEAR_OREF |
                                         UART_CLEAR_RTOF);
        it->second->enable_rx_dma();
    }
}

extern "C" __attribute__((section(".itcm_text"))) void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    const auto it = pyro::uart_drv_t::uart_map().find(huart);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (it != pyro::uart_drv_t::uart_map().end() && it->second)
    {
        const auto drv    = it->second;
        drv->state.tx_busy = 0;

        if (drv->_tx_cplt_callback)
        {
            drv->_tx_cplt_callback(xHigherPriorityTaskWoken);
        }
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
