/**
 * @file pyro_dr16_rc_drv.h
 * @brief Header file for the PYRO DR16 Remote Control Driver.
 * PYRO DR16 遥控器驱动头文件。
 *
 * Encapsulates the DJI DR16 protocol logic. It defines the specific bit-packed
 * structures for the SBUS/DBUS stream and the state machines required to
 * convert raw signals into logical events (e.g., Key Press/Hold).
 * 封装 DJI DR16 协议逻辑。定义其协议的特定结构，以及将原始信号转换为逻辑事件。
 *
 * @author Lucky
 * @version 2.1.0
 * @date 2026-1-28
 */

#ifndef __PYRO_DR16_RC_DRV_H__
#define __PYRO_DR16_RC_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_rc_base_drv.h"

namespace pyro
{

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief Driver implementation for DJI DR16 receivers.
 * DJI DR16 接收机驱动实现。
 */
class dr16_drv_t : public rc_drv_t
{
    friend class rc_hub_t;
    /* Private Types - Raw Buffer --------------------------------------------*/
    /**
     * @brief Raw 18-byte bit-packed data packet (Standard DJI Protocol).
     * 18 字节位压缩原始数据包（标准 DJI 协议）。
     */
    typedef struct __packed
    {
        uint32_t ch0 : 11; // Right Stick Horizontal (RX)
        uint32_t ch1 : 11; // Right Stick Vertical (RY)
        uint32_t ch2 : 11; // Left Stick Horizontal (LX)
        uint32_t ch3 : 11; // Left Stick Vertical (LY)
        uint32_t s1  : 2;  // Left Switch
        uint32_t s2  : 2;  // Right Switch

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t press_l;
        uint8_t press_r;
        uint16_t key_code; // Keyboard bitmask
        uint16_t wheel;

    } dr16_buf_t;

    /* Private Types - Control Data ------------------------------------------*/
  public:
    // Switch physical states / 拨杆物理状态
    enum class sw_state_t
    {
        SW_UP   = 1,
        SW_MID  = 3,
        SW_DOWN = 2
    };
    // Switch logic transitions / 拨杆逻辑跃迁
    enum class sw_ctrl_t
    {
        SW_UP_TO_MID   = 1,
        SW_MID_TO_DOWN = 2,
        SW_DOWN_TO_MID = 3,
        SW_MID_TO_UP   = 4,
    };
    // Key logical actions / 按键逻辑动作
    enum class key_ctrl_t
    {
        KEY_PRESSED = 1,
        KEY_HOLD    = 2
    };

    /**
     * @brief Key state container for the debounce state machine.
     * 消抖状态机使用的按键状态容器。
     */
    struct key_t
    {
        friend class dr16_drv_t;
        // === using Interface / 使用者接口 ===
        key_ctrl_t ctrl;       // Current logical event (Click/Hold)
        float hold_time;       // Duration of hold state
        float change_time;     // Timestamp of last event
        uint32_t repeat_times; // Count for rapid clicks
        uint8_t state;         // Real-time physical state

        // === Internal State / 内部状态 ===
      private:
        float last_time{};    // Debounce accumulator
        float press_time{};   // Timestamp of press start
        float release_time{}; // Timestamp of release
        bool pending{};       // State for click-confirmation
    };

    struct switch_t
    {
        sw_state_t state;
        sw_ctrl_t ctrl;
        float change_time;
    };

    /**
     * @brief Final normalized control data structure.
     * 最终归一化的控制数据结构。
     */
    typedef struct dr16_ctrl_t
    {
        struct
        {
            float ch_lx;  ///< Left X [-1.0, 1.0]
            float ch_ly;  ///< Left Y
            float ch_rx;  ///< Right X
            float ch_ry;  ///< Right Y
            switch_t s_l; ///< Left Switch
            switch_t s_r; ///< Right Switch
            float wheel;  ///< Wheel
        } rc;
        struct
        {
            float x;
            float y;
            float z;
            key_t press_l;
            key_t press_r;
        } mouse;
        struct
        {
            key_t w;
            key_t s;
            key_t a;
            key_t d;
            key_t shift;
            key_t ctrl;
            key_t q;
            key_t e;
            key_t r;
            key_t f;
            key_t g;
            key_t z;
            key_t x;
            key_t c;
            key_t v;
            key_t b;
        } key;
    } dr16_ctrl_t;

    /* Public Methods - Construction and Lifecycle (Override)
     * ------------------*/
    status_t init() override;
    void enable() override;
    void disable() override;
    void thread() override;

  private:
    explicit dr16_drv_t(uart_drv_t *dr16_uart);
    dr16_ctrl_t _dr16_ctrl{}; ///< Latest decoded data. 最新解码数据。

    /* Private Methods - Overrides
     * ---------------------------------------------*/
    /**
     * @brief ISR Callback: Buffers data based on protocol priority.
     * 中断回调：基于协议优先级缓冲数据。
     */
    bool rc_callback(uint8_t *buf, uint16_t len,
                     BaseType_t& xHigherPriorityTaskWoken) override;

    /* Private Methods - Processing
     * --------------------------------------------*/
    static status_t error_check(const dr16_buf_t *dr16_buf);
    static void check_ctrl(switch_t &dr16_switch, uint8_t raw_state);
    static void check_ctrl(key_t &key, uint8_t raw_state);

    /**
     * @brief Decodes raw buffer
     * 将原始缓冲区解码
     */
    void unpack(const dr16_buf_t *dr16_buf);
};

} // namespace pyro
#endif