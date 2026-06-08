/**
* @file pyro_rc_hub.h
 * @brief Header file for the PYRO RC Hub (Factory).
 * PYRO 遥控器枢纽（工厂）头文件。
 *
 * Defines `rc_hub_t`, a static factory class that provides a unified entry point
 * to access specific Remote Control driver singletons. It abstracts the
 * creation and management of underlying driver instances.
 * 定义 `rc_hub_t` 静态工厂类，提供统一入口以访问特定的遥控器驱动单例。它抽象了
 * 底层驱动实例的创建与管理。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-11-14
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_RC_HUB_H__
#define __PYRO_RC_HUB_H__

#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"

namespace pyro
{
/**
 * @brief Static factory class for RC driver management.
 * 遥控器驱动管理的静态工厂类。
 *
 * This class cannot be instantiated. It serves solely as a namespace-like
 * container for the `get_instance` factory method.
 * 该类无法被实例化。它仅作为 `get_instance` 工厂方法的容器（类似命名空间）使用。
 */
class rc_hub_t
{
public:
    /* Delete constructors to enforce static usage / 删除构造函数以强制静态使用 */
    rc_hub_t() = delete;
    ~rc_hub_t() = delete;

    /**
     * @brief Supported RC protocol identifiers.
     * 支持的遥控协议标识符。
     */
    enum which_rc_t
    {
        VT03 = 0, ///< DJI VT03 Protocol (e.g., DJI FPV/RC2)
        DR16 = 1, ///< DJI DR16 Protocol (e.g., DT7/Standard)
    };

    /**
     * @brief Retrieves the singleton instance of a specific RC driver.
     * 获取特定 RC 驱动的单例实例。
     *
     * @param which_rc The protocol identifier (DR16 or VT03).
     * @return Pointer to the `rc_drv_t` base class.
     * @return rc_drv_t* 指向驱动基类的指针。
     */
    static rc_drv_t *get_instance(which_rc_t which_rc);
};
} // namespace pyro
#endif