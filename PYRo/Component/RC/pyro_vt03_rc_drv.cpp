/**
 * @file pyro_vt03_rc_drv.cpp
 * @brief Implementation file for the PYRO VT03 Remote Control Driver.
 * PYRO VT03 遥控器驱动实现文件。
 *
 * Implements the VT03 protocol handling. This driver has higher priority
 * (Priority 0) than DR16, meaning it will override DR16 inputs if valid data
 * is detected.
 * 实现 VT03 协议处理。该驱动具有比 DR16 更高的优先级（优先级 0），这意味着如果
 * 检测到有效数据，它将覆盖 DR16 的输入。
 *
 * @author Lucky
 * @version 2.1.0
 * @date 2026-1-28
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_vt03_rc_drv.h"
#include "pyro_crc.h"
#include "pyro_rw_lock.h"
#include "task.h"
#include <cstring>
#include "pyro_dwt_drv.h"

// External FreeRTOS task entry point
extern "C" void vt03_task(void *argument);

namespace pyro
{
static constexpr uint16_t VT03_CH_VALUE_MIN    = 364;
static constexpr uint16_t VT03_CH_VALUE_MAX    = 1684;
static constexpr uint16_t VT03_CH_VALUE_OFFSET = 1024;

/* Constructor ---------------------------------------------------------------*/
vt03_drv_t::vt03_drv_t(uart_drv_t *vt03_uart) : rc_drv_t(vt03_uart)
{
    _priority = 0; // Highest priority
    vt03_drv_t::init();
}

/* Initialization ------------------------------------------------------------*/
status_t vt03_drv_t::init()
{
    // Create the message buffer (108 bytes capacity)
    _rc_msg_buffer   = xMessageBufferCreate(108);

    // Create the processing task
    const BaseType_t x_ret = xTaskCreate(vt03_task, "vt03_task", 256, this,
                                   configMAX_PRIORITIES - 1, &_rc_task_handle);

    if (x_ret != pdPASS) return PYRO_ERROR;
    if (_rc_msg_buffer == nullptr) return PYRO_ERROR;

    _lock = new rw_lock;
    _rc_data = &_vt03_ctrl;
    return PYRO_OK;
}

/* Enable/Disable ------------------------------------------------------------*/
void vt03_drv_t::enable()
{
    _rc_uart->add_rx_event_callback(
        [this](uint8_t *buf, const uint16_t len,
               BaseType_t& xHigherPriorityTaskWoken) -> bool
        { return rc_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
}

void vt03_drv_t::disable()
{
    sequence &= ~(1 << _priority);
    _rc_uart->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
}


/* Data Processing - Error Check ---------------------------------------------*/
/**
 * @brief Validates channel ranges and CRC Checksum.
 * 验证通道范围和 CRC 校验和。
 */
status_t vt03_drv_t::error_check(const vt03_buf_t *vt03_buf)
{
    if (vt03_buf->ch0 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch0 > VT03_CH_VALUE_MAX ||
        vt03_buf->ch1 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch1 > VT03_CH_VALUE_MAX ||
        vt03_buf->ch2 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch2 > VT03_CH_VALUE_MAX ||
        vt03_buf->ch3 < VT03_CH_VALUE_MIN ||
        vt03_buf->ch3 > VT03_CH_VALUE_MAX ||
        vt03_buf->wheel < VT03_CH_VALUE_MIN ||
        vt03_buf->wheel > VT03_CH_VALUE_MAX)
    {
        return PYRO_ERROR;
    }
    // Verify CRC16 to ensure data integrity
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const*>(vt03_buf), sizeof(vt03_buf_t)))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

/**
 * @brief Updates logic for the 3-position gear switch.
 * 更新三档拨杆开关的逻辑。
 */
void vt03_drv_t::check_ctrl(vt03_gear_t &vt03_gear, const uint8_t raw_state)
{
    vt03_gear_t gear = {};
    const auto state = static_cast<gear_state_t>(raw_state);

    if (vt03_gear.state != state)
    {
        if (gear_state_t::GEAR_LEFT == vt03_gear.state && gear_state_t::GEAR_MID == state)
        {
            gear.ctrl = gear_ctrl_t::GEAR_LEFT_TO_MID;
        }
        else if (gear_state_t::GEAR_MID == vt03_gear.state && gear_state_t::GEAR_LEFT == state)
        {
            gear.ctrl = gear_ctrl_t::GEAR_MID_TO_LEFT;
        }
        else if (gear_state_t::GEAR_MID == vt03_gear.state && gear_state_t::GEAR_RIGHT == state)
        {
            gear.ctrl = gear_ctrl_t::GEAR_MID_TO_RIGHT;
        }
        else if (gear_state_t::GEAR_RIGHT == vt03_gear.state && gear_state_t::GEAR_MID == state)
        {
            gear.ctrl = gear_ctrl_t::GEAR_RIGHT_TO_MID;
        }
        vt03_gear.change_time = pyro::dwt_drv_t::get_timeline_ms();
    }
    else
    {
        gear.ctrl = vt03_gear.ctrl;
        gear.change_time = vt03_gear.change_time;
    }
    gear.state = state;
    vt03_gear  = gear;
}

/**
 * @brief Advanced Key State Machine (Debounce / Click / Hold).
 * 高级按键状态机（消抖 / 点击 / 长按）。
 * * Filters noise and distinguishes between short clicks and long holds.
 * 过滤噪声并区分短按点击与长按保持。
 */
void vt03_drv_t::check_ctrl(key_t &key, const uint8_t raw_state)
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
 * @brief Unpacks the raw VT03 buffer into the consumer control structure.
 * 将原始 VT03 缓冲区解包为消费者控制结构。
 */
void vt03_drv_t::unpack(const vt03_buf_t *vt03_buf)
{
    if (PYRO_OK == error_check(vt03_buf))
    {
        write_scope_lock rc_write_lock(get_lock());

        // Normalize RC channels to [-1.0, 1.0]
        _vt03_ctrl.rc.ch_rx =
            static_cast<float>(vt03_buf->ch0 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.ch_ry =
            static_cast<float>(vt03_buf->ch1 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.ch_lx =
            static_cast<float>(vt03_buf->ch2 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.ch_ly =
            static_cast<float>(vt03_buf->ch3 - VT03_CH_VALUE_OFFSET) / 660.0f;
        _vt03_ctrl.rc.wheel =
            static_cast<float>(vt03_buf->wheel - VT03_CH_VALUE_OFFSET) / 660.0f;

        // Copy discrete inputs
        check_ctrl(_vt03_ctrl.rc.gear, vt03_buf->gear);
        check_ctrl(_vt03_ctrl.rc.fn_l, vt03_buf->fn_l);
        check_ctrl(_vt03_ctrl.rc.fn_r, vt03_buf->fn_r);
        check_ctrl(_vt03_ctrl.rc.pause, vt03_buf->pause);
        check_ctrl(_vt03_ctrl.rc.trigger, vt03_buf->trigger);

        _vt03_ctrl.mouse.x = static_cast<float>(vt03_buf->mouse_x) / 32768.0f;
        _vt03_ctrl.mouse.y = static_cast<float>(vt03_buf->mouse_y) / 32768.0f;
        _vt03_ctrl.mouse.z = static_cast<float>(vt03_buf->mouse_z) / 32768.0f;

        check_ctrl(_vt03_ctrl.mouse.press_l, vt03_buf->press_l);
        check_ctrl(_vt03_ctrl.mouse.press_r, vt03_buf->press_r);
        check_ctrl(_vt03_ctrl.mouse.press_m, vt03_buf->press_m);

        for (int i = 0; i < 16; ++i)
        {
            check_ctrl(*(reinterpret_cast<key_t *>(&_vt03_ctrl.key) + i),
                       (vt03_buf->key_code >> i) & 0x01);
        }
    }
}

/* Interrupt Service Routine (ISR) Callback ----------------------------------*/
/**
 * @brief ISR handler. Checks Header, Size, and Priority.
 * ISR 处理程序。检查包头、大小和优先级。
 */
bool vt03_drv_t::rc_callback(uint8_t *buf, const uint16_t len,
                             BaseType_t& xHigherPriorityTaskWoken)
{
    if (len == sizeof(vt03_buf_t))
    {
        // Check magic numbers (VT03 Header: 0xA9 0x53)
        if (buf[0] == 0xA9 && buf[1] == 0x53)
        {
            // Priority check: VT03 is high priority (0), usually wins
            if (__builtin_ctz(sequence) >= _priority)
            {
                xMessageBufferSendFromISR(_rc_msg_buffer, buf, len,
                                          &xHigherPriorityTaskWoken);
                return true;
            }
        }
    }
    return false;
}

/* FreeRTOS Task Thread ------------------------------------------------------*/
/**
 * @brief Main processing loop. Manages connection timeout and decoding.
 * 主处理循环。管理连接超时和数据接收。
 */
void vt03_drv_t::thread()
{
    static vt03_buf_t vt03_buf;
    static size_t xReceivedBytes;

    // Wait for initial connection
    if (xMessageBufferReceive(_rc_msg_buffer, &vt03_buf, sizeof(vt03_buf_t),
                              portMAX_DELAY) == sizeof(vt03_buf_t))
    {
        // Signal that this high-priority protocol is now online
        sequence |= (1 << _priority);
    }

    // Active processing
    while (sequence >> _priority & 0x01)
    {
        xReceivedBytes = xMessageBufferReceive(_rc_msg_buffer, &vt03_buf,
                                               sizeof(vt03_buf_t), 120);
        if (xReceivedBytes == sizeof(vt03_buf_t))
        {
            unpack(&vt03_buf);
        }
        else if (xReceivedBytes == 0)
        {
            // Timeout -> Offline
            sequence &= ~(1 << _priority);
        }
    }
}

/* External FreeRTOS Task Entry ----------------------------------------------*/
extern "C" void vt03_task(void *argument)
{
    auto *drv = static_cast<pyro::vt03_drv_t *>(argument);
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