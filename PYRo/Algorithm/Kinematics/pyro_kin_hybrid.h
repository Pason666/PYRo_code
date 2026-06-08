#ifndef __PYRO_KIN_HYBRID__
#define __PYRO_KIN_HYBRID__

namespace pyro
{

/**
 * @brief Hybrid Chassis Kinematics (Tracks + Mecanum)
 * 混合底盘运动学解算 (履带 + 麦克纳姆轮)
 * Supports standard and arbitrary asymmetric chassis geometries.
 * 支持标准对称底盘以及任意非对称畸形底盘。
 */
class hybrid_kin_t
{
public:
    /**
     * @brief Output structure containing target speeds for all 6 traction motors
     * 包含全部6个驱动电机目标速度的输出结构体
     */
    struct hybrid_speeds_t
    {
        // Front Tracks 前置履带
        float track_l;
        float track_r;

        // Rear Mecanum Wheels 后置麦克纳姆轮
        float mec_fl;
        float mec_fr;
        float mec_bl;
        float mec_br;
    };

    /**
     * @brief 1. Expert Constructor (Independent K-values for each wheel)
     * 专家模式构造函数 (为每个轮子传入独立的K值)
     * For extreme asymmetric or shifted center-of-gravity setups.
     * 适用于重心严重偏移或非对称的异形底盘。
     * K = |X distance to Center of Rotation| + |Y distance to Center of Rotation|
     * K值定义: K = |轮子到旋转中心的X轴距离| + |轮子到旋转中心的Y轴距离|
     * @param track_spacing Distance between the centers of the two tracks (m) 左右履带中心间距(m)
     * @param k_fl          Left-Front Mecanum K-value 左前麦轮旋转系数
     * @param k_fr          Right-Front Mecanum K-value 右前麦轮旋转系数
     * @param k_bl          Back-Left Mecanum K-value 左后麦轮旋转系数
     * @param k_br          Back-Right Mecanum K-value 右后麦轮旋转系数
     */
    hybrid_kin_t(float track_spacing, float k_fl, float k_fr, float k_bl, float k_br);

    /**
     * @brief 2. Standard Constructor (Ideal rectangular chassis)
     * 标准模式构造函数 (理想矩形对称底盘)
     * Automatically calculates unified K-value from dimensions.
     * 自动通过长宽计算统一的K值。
     * @param track_spacing   Distance between the centers of the two tracks (m) 左右履带中心间距(m)
     * @param mec_wheelbase   Wheelbase of the mecanum section (m) 麦轮总轴距(前后轮心距离)(m)
     * @param mec_track_width Track width of the mecanum section (m) 麦轮总轮距(左右轮心距离)(m)
     */
    hybrid_kin_t(float track_spacing, float mec_wheelbase, float mec_track_width);

    /**
     * @brief Inverse Kinematics Solver 逆运动学解算器
     * @param vx       Linear velocity X (m/s) 前后平移速度
     * @param vy       Linear velocity Y (m/s) 左右平移速度 (Ignored when track_en is true / 履带接地时忽略)
     * @param wz       Angular velocity Z (rad/s) 自转角速度
     * @param track_en true: Tracks enabled (Climbing mode), false: Cruising mode 履带使能(true=爬坡模式/禁止横移, false=巡航模式)
     * @return hybrid_speeds_t Solved speeds for all motors 解算得到的各电机速度
     */
    [[nodiscard]] hybrid_speeds_t solve(float vx, float vy, float wz, bool track_en) const;

private:
    float _k_track; // Track rotation coefficient 履带旋转系数 (track_spacing / 2)

    // Independent geometry coefficients for the 4 mecanum wheels
    // 四个麦轮独立的几何结构常数 (K = lx + ly)
    float _k_fl;
    float _k_fr;
    float _k_bl;
    float _k_br;
};

} // namespace pyro

#endif