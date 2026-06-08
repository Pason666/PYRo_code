/**
 * @file pyro_vt03_rc_drv.h
 * @brief Header file for the PYRO VT03 Remote Control Driver.
 * PYRO VT03 遥控器驱动头文件。
 *
 * Defines the logic for the newer DJI VT03 protocol (used in DJI FPV/RC2).
 * It supports more channels, CRC validation, and a different packet structure
 * compared to DR16.
 * 定义较新的 DJI VT03 协议逻辑（用于 DJI FPV/RC2）。与 DR16 相比，它支持更多
 * 通道、CRC 校验以及不同的数据包结构。
 *
 * @author Lucky
 * @version 2.1.0
 * @date 2026-1-28
 */

#ifndef __PYRO_VT03_RC_DRV_H__
#define __PYRO_VT03_RC_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_rc_base_drv.h"

namespace pyro
{

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief Driver class for the DJI VT03 remote control protocol.
 * DJI VT03 遥控器协议驱动类。
 */
class vt03_drv_t : public rc_drv_t
{
    friend class rc_hub_t;
    /* Private Types - Raw Buffer --------------------------------------------*/
    /**
     * @brief Raw VT03 packet structure. Includes Headers and CRC.
     * 原始 VT03 数据包结构。包含包头和 CRC。
     */
    typedef struct __packed
    {
        uint8_t sof1; // Start of Frame 1
        uint8_t sof2; // Start of Frame 2

        uint64_t ch0     : 11;
        uint64_t ch1     : 11;
        uint64_t ch2     : 11;
        uint64_t ch3     : 11;
        uint64_t gear    : 2;  // 3-position switch
        uint64_t pause   : 1;  // Button
        uint64_t fn_l    : 1;
        uint64_t fn_r    : 1;
        uint64_t wheel   : 11;
        uint64_t trigger : 1;

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t press_l : 2;
        uint8_t press_r : 2;
        uint8_t press_m : 2;
        uint16_t key_code;

        uint16_t crc; // Checksum

    } vt03_buf_t;

    /* Private Types - Control Data ------------------------------------------*/

  public:
    enum class gear_state_t
    {
        GEAR_LEFT  = 0,
        GEAR_MID   = 1,
        GEAR_RIGHT = 2,
    };
    enum class gear_ctrl_t
    {
        GEAR_LEFT_TO_MID  = 1,
        GEAR_MID_TO_RIGHT = 2,
        GEAR_RIGHT_TO_MID = 3,
        GEAR_MID_TO_LEFT  = 4,
    };
    enum class key_ctrl_t
    {
        KEY_PRESSED = 1,
        KEY_HOLD    = 2
    };

    /**
     * @brief Key state structure with internal debounce logic.
     * 包含内部消抖逻辑的按键状态结构体。
     */
    struct key_t
    {
        friend class vt03_drv_t;
        // === Interface / 接口 ===
        key_ctrl_t ctrl;
        float hold_time;
        float change_time;
        uint32_t repeat_times;
        uint8_t state;

        // === Private State / 私有状态 ===
    private:
        float last_time{};         // Debounce timer
        float press_time{};        // Hold calculator
        float release_time{};      // Repeat calculator
        bool  pending{};           // Click confirmation
    };

    typedef struct vt03_gear_t
    {
        gear_state_t state;
        gear_ctrl_t ctrl;
        float change_time;
    } vt03_gear_t;

    /**
     * @brief Final decoded control data structure for VT03.
     * VT03 的最终解码控制数据结构。
     */
    typedef struct vt03_ctrl_t
    {
        struct
        {
            float ch_lx;
            float ch_ly;
            float ch_rx;
            float ch_ry;
            float wheel;
            vt03_gear_t gear;
            key_t fn_l;
            key_t fn_r;
            key_t pause;
            key_t trigger;
        } rc;

        struct
        {
            float x; float y; float z;
            key_t press_l; key_t press_r; key_t press_m;
        } mouse;

        struct
        {
            key_t w; key_t s; key_t a; key_t d;
            key_t shift; key_t ctrl;
            key_t q; key_t e; key_t r; key_t f; key_t g;
            key_t z; key_t x; key_t c; key_t v; key_t b;
        } key;
    } vt03_ctrl_t;

    /* Public Members --------------------------------------------------------*/

    /* Public Methods - Construction and Lifecycle (Override)
     * ------------------*/
    status_t init() override;
    void enable() override;
    void disable() override;
    void thread() override;

  private:
    explicit vt03_drv_t(uart_drv_t *vt03_uart);
    vt03_ctrl_t _vt03_ctrl{}; ///< The latest decoded control data.

    /* Private Methods - Overrides
     * ---------------------------------------------*/
    /**
     * @brief ISR Callback: Validates header and buffers data.
     * 中断回调：验证包头并缓冲数据。
     */
    bool rc_callback(uint8_t *buf, uint16_t len,
                     BaseType_t& xHigherPriorityTaskWoken) override;

    /* Private Methods - Processing
     * --------------------------------------------*/
    static status_t error_check(const vt03_buf_t *vt03_buf);
    static void check_ctrl(vt03_gear_t &vt03_gear, uint8_t raw_state);
    static void check_ctrl(key_t &key, uint8_t raw_state);

    /**
     * @brief Unpacks raw VT03 data structure.
     * 解包原始 VT03 数据结构。
     */
    void unpack(const vt03_buf_t *vt03_buf);
};

} // namespace pyro
#endif