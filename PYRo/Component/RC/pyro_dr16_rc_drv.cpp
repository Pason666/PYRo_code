/**
 * @file pyro_dr16_rc_drv.cpp
 * @brief Implementation of the PYRO DR16 Remote Control Driver.
 * PYRO DR16 遥控器驱动实现文件。
 *
 * Implements the data decoding pipeline: ISR capture -> Priority Check ->
 * Message Buffer -> Task Processing -> Unpacking & State Machine -> Output.
 * 实现数据解码流水线：ISR 捕获 -> 优先级检查 -> 消息缓冲 -> 任务处理 ->
 * 解包与状态机 -> 输出。
 *
 * @author Lucky
 * @version 2.1.0
 * @date 2026-1-28
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_rw_lock.h"
#include "task.h"
#include <cstring>

// External FreeRTOS task entry point
extern "C" void dr16_task(void *argument);

namespace pyro
{
static constexpr uint16_t DR16_CH_VALUE_MIN    = 364;
static constexpr uint16_t DR16_CH_VALUE_MAX    = 1684;
static constexpr uint16_t DR16_CH_VALUE_OFFSET = 1024;

/* Constructor ---------------------------------------------------------------*/
dr16_drv_t::dr16_drv_t(uart_drv_t *dr16_uart) : rc_drv_t(dr16_uart)
{
    _priority = 1; // Lower priority than VT03 (which is 0)
    dr16_drv_t::init();
}

/* Initialization ------------------------------------------------------------*/
/**
 * @brief Sets up OS primitives (Task, Buffer) for async processing.
 * 为异步处理设置操作系统原语（任务、缓冲区）。
 */
status_t dr16_drv_t::init()
{
    // Create the message buffer (108 bytes capacity)
    _rc_msg_buffer = xMessageBufferCreate(108);

    // Create the processing task
    const BaseType_t x_ret =
        xTaskCreate(dr16_task, "dr16_task", 256, this, configMAX_PRIORITIES - 1,
                    &_rc_task_handle);

    if (x_ret != pdPASS) return PYRO_ERROR;
    if (_rc_msg_buffer == nullptr) return PYRO_ERROR;

    _lock    = new rw_lock;
    _rc_data = &_dr16_ctrl;
    return PYRO_OK;
}

/* Enable/Disable ------------------------------------------------------------*/
void dr16_drv_t::enable()
{
    // Link the ISR callback to the UART driver
    _rc_uart->add_rx_event_callback(
        [this](uint8_t *buf, const uint16_t len,
               BaseType_t& xHigherPriorityTaskWoken) -> bool
        { return rc_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
}

void dr16_drv_t::disable()
{
    // Remove sequence bit and unregister callback
    sequence &= ~(1 << _priority);
    _rc_uart->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
}


 /* Data Processing - Error Check ---------------------------------------------*/
status_t dr16_drv_t::error_check(const dr16_buf_t *dr16_buf)
{
    // Basic boundary validation for stick channels
    if (dr16_buf->ch0 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch0 > DR16_CH_VALUE_MAX ||
        dr16_buf->ch1 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch1 > DR16_CH_VALUE_MAX ||
        dr16_buf->ch2 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch2 > DR16_CH_VALUE_MAX ||
        dr16_buf->ch3 < DR16_CH_VALUE_MIN ||
        dr16_buf->ch3 > DR16_CH_VALUE_MAX )
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/**
 * @brief Updates switch state transitions (e.g., UP -> MID).
 * 更新拨杆状态跃迁（例如：上 -> 中）。
 */
void dr16_drv_t::check_ctrl(switch_t &dr16_switch, const uint8_t raw_state)
{
    switch_t switch_ = {};
    const auto state = static_cast<sw_state_t>(raw_state);

    // Detect transition edges and assign control events
    if (dr16_switch.state != state)
    {
        if (sw_state_t::SW_UP == dr16_switch.state && sw_state_t::SW_MID == state)
        {
            switch_.ctrl = sw_ctrl_t::SW_UP_TO_MID;
        }
        else if (sw_state_t::SW_MID == dr16_switch.state &&
                 sw_state_t::SW_DOWN == state)
        {
            switch_.ctrl = sw_ctrl_t::SW_MID_TO_DOWN;
        }
        else if (sw_state_t::SW_DOWN == dr16_switch.state &&
                 sw_state_t::SW_MID == state)
        {
            switch_.ctrl = sw_ctrl_t::SW_DOWN_TO_MID;
        }
        else if (sw_state_t::SW_MID == dr16_switch.state &&
                 sw_state_t::SW_UP == state)
        {
            switch_.ctrl = sw_ctrl_t::SW_MID_TO_UP;
        }
        switch_.change_time = pyro::dwt_drv_t::get_timeline_ms();
    }
    else
    {
        switch_.ctrl = dr16_switch.ctrl;
        switch_.change_time = dr16_switch.change_time;
    }
    switch_.state = state;
    dr16_switch   = switch_;
}

/**
 * @brief Advanced Key State Machine (Debounce / Click / Hold).
 * 高级按键状态机（消抖 / 点击 / 长按）。
 * * Filters noise and distinguishes between short clicks and long holds.
 * 过滤噪声并区分短按点击与长按保持。
 */
void dr16_drv_t::check_ctrl(key_t &key, const uint8_t raw_state)
{

    const float now                  = pyro::dwt_drv_t::get_timeline_ms();
    constexpr float DEBOUNCE_MS      = 10.0f;
    constexpr float HOLD_TRIGGER_MS  = 200.0f;
    constexpr float REPEAT_WINDOW_MS = 220.0f;

    bool rising_edge                 = false;
    bool falling_edge                = false;

    // 1. Physical Debounce Logic / 物理消抖逻辑
    if (raw_state != key.state)
    {
        if (key.last_time == 0.0f)
        {
            key.last_time = now;
        }
        else if ((now - key.last_time) > DEBOUNCE_MS)
        {
            key.state     = raw_state;
            key.last_time = 0.0f;

            if (key.state == 1)
                rising_edge = true;
            else
                falling_edge = true;
        }
    }
    else
    {
        key.last_time = 0.0f;
    }

    // 2. Rising Edge (Press Start) / 上升沿（按下开始）
    if (rising_edge)
    {
        // Check for double/triple clicks
        if (key.pending)
        {
            key.repeat_times++;
            key.pending = false;
        }
        else
        {
            key.repeat_times = 1;
        }
        key.press_time = now;
        key.ctrl       = static_cast<key_ctrl_t>(0);
    }
    // 3. Falling Edge (Release) / 下降沿（松开）
    else if (falling_edge)
    {
        if (key.ctrl != key_ctrl_t::KEY_HOLD)
        {
            key.release_time = now;
            key.pending      = true; // Wait to confirm if it's a click
        }
    }

    // 4. Hold Detection / 长按检测
    if (key.state == 1)
    {
        const float duration = now - key.press_time;

        if (key.ctrl == key_ctrl_t::KEY_HOLD)
        {
            key.hold_time = duration - HOLD_TRIGGER_MS;
        }
        else if (duration > HOLD_TRIGGER_MS)
        {
            key.ctrl        = key_ctrl_t::KEY_HOLD;
            key.change_time = now;
            key.hold_time   = 0.0f;
            key.pending     = false;
        }
    }

    // 5. Click Confirmation (Timeout) / 点击确认（超时）
    if (key.pending)
    {
        if ((now - key.release_time) > REPEAT_WINDOW_MS)
        {
            key.ctrl        = key_ctrl_t::KEY_PRESSED;
            key.change_time = now;
            key.pending     = false;
        }
    }
}

/* Data Processing - Unpack --------------------------------------------------*/
/**
 * @brief Converts raw bitfields to application-usable data types.
 * 将原始位域转换为应用可用的数据类型。
 */
void dr16_drv_t::unpack(const dr16_buf_t *dr16_buf)
{
    if (PYRO_OK == error_check(dr16_buf))
    {
        write_scope_lock rc_write_lock(get_lock());

        // Normalize channels to [-1.0, 1.0] range
        _dr16_ctrl.rc.ch_rx =
            static_cast<float>(dr16_buf->ch0 - DR16_CH_VALUE_OFFSET) / 660.0f;
        _dr16_ctrl.rc.ch_ry =
            static_cast<float>(dr16_buf->ch1 - DR16_CH_VALUE_OFFSET) / 660.0f;
        _dr16_ctrl.rc.ch_lx =
            static_cast<float>(dr16_buf->ch2 - DR16_CH_VALUE_OFFSET) / 660.0f;
        _dr16_ctrl.rc.ch_ly =
            static_cast<float>(dr16_buf->ch3 - DR16_CH_VALUE_OFFSET) / 660.0f;
        _dr16_ctrl.rc.wheel =
            static_cast<float>(dr16_buf->wheel - DR16_CH_VALUE_OFFSET) / 660.0f;

        // Process discrete inputs (switches, mouse, keys)
        check_ctrl(_dr16_ctrl.rc.s_r, dr16_buf->s1);
        check_ctrl(_dr16_ctrl.rc.s_l, dr16_buf->s2);

        _dr16_ctrl.mouse.x = static_cast<float>(dr16_buf->mouse_x) / 32768.0f;
        _dr16_ctrl.mouse.y = static_cast<float>(dr16_buf->mouse_y) / 32768.0f;
        _dr16_ctrl.mouse.z = static_cast<float>(dr16_buf->mouse_z) / 32768.0f;

        check_ctrl(_dr16_ctrl.mouse.press_l, dr16_buf->press_l);
        check_ctrl(_dr16_ctrl.mouse.press_r, dr16_buf->press_r);

        // Iterate through keyboard bitmask
        for (int i = 0; i < 16; ++i)
        {
            check_ctrl(*(reinterpret_cast<key_t *>(&_dr16_ctrl.key) + i),
                       (dr16_buf->key_code >> i) & 0x01);
        }
    }
}

/* Interrupt Service Routine (ISR) Callback ----------------------------------*/
bool dr16_drv_t::rc_callback(uint8_t *buf, const uint16_t len,
                             BaseType_t& xHigherPriorityTaskWoken)
{
    if (len == sizeof(dr16_buf_t))
    {
        // Priority Arbitration:
        // Only process if no higher priority driver is active.
        // 优先级仲裁：仅在无更高优先级驱动活动时处理。
        if (__builtin_ctz(sequence) >= _priority)
        {
            xMessageBufferSendFromISR(_rc_msg_buffer, buf, len,
                                      &xHigherPriorityTaskWoken);
            return true;
        }
    }
    return false;
}

/* FreeRTOS Task Thread ------------------------------------------------------*/
/**
 * @brief Main processing loop. Manages connection timeout and decoding.
 * 主处理循环。管理连接超时和数据接收。
 */
void dr16_drv_t::thread()
{
    static dr16_buf_t dr16_buf;
    static size_t xReceivedBytes;

    // Phase 1: Connection establishment (Wait forever for first packet)
    if (xMessageBufferReceive(_rc_msg_buffer, &dr16_buf, sizeof(dr16_buf_t),
                              portMAX_DELAY) == sizeof(dr16_buf_t))
    {
        // Mark this protocol as active in the global sequence
        sequence |= (1 << _priority);
    }

    // Phase 2: Active processing loop
    while (sequence >> _priority & 0x01)
    {
        // Wait for data with timeout (approx 100ms/ticks)
        xReceivedBytes = xMessageBufferReceive(_rc_msg_buffer, &dr16_buf,
                                               sizeof(dr16_buf_t), 100);
        if (xReceivedBytes == sizeof(dr16_buf_t))
        {
            unpack(&dr16_buf);
        }
        else if (xReceivedBytes == 0)
        {
            // Timeout: Connection lost, clear active bit
            sequence &= ~(1 << _priority);
        }
    }
}

/* External FreeRTOS Task Entry ----------------------------------------------*/
extern "C" void dr16_task(void *argument)
{
    auto *drv = static_cast<pyro::dr16_drv_t *>(argument);
    if (drv)
    {
        while (true)
        {
            drv->thread();
        }
    }
    vTaskDelete(nullptr);
}

}