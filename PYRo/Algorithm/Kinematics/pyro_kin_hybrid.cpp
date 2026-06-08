#include "pyro_kin_hybrid.h"
#include "arm_math.h" 
#include <cmath> // For std::abs or fabsf

namespace pyro
{

// ============================================================================
// 1. Expert Constructor / 专家模式构造函数
// ============================================================================
hybrid_kin_t::hybrid_kin_t(const float track_spacing,
                           const float k_fl, const float k_fr,
                           const float k_bl, const float k_br)
{
    // Tracks are assumed symmetric left-to-right
    // 假设履带左右对称，取间距的一半
    _k_track = abs(track_spacing) / 2.0f;

    // Store directly calculated K-values for each wheel
    // 直接存储用户传入的独立 K 值
    _k_fl = abs(k_fl);
    _k_fr = abs(k_fr);
    _k_bl = abs(k_bl);
    _k_br = abs(k_br);
}

// ============================================================================
// 2. Standard Constructor / 标准模式构造函数
// ============================================================================
hybrid_kin_t::hybrid_kin_t(const float track_spacing,
                           const float mec_wheelbase, const float mec_track_width)
{
    _k_track = abs(track_spacing) / 2.0f;

    // Standard formula: K = (Wheelbase + TrackWidth) / 2
    // 标准底盘公式：K = (轴距 + 轮距) / 2  (等价于 半轴距 + 半轮距)
    const float k_std = (abs(mec_wheelbase) + abs(mec_track_width)) / 2.0f;

    // Apply unified K to all wheels
    // 将统一的 K 赋值给所有轮子
    _k_fl = k_std;
    _k_fr = k_std;
    _k_bl = k_std;
    _k_br = k_std;
}

// ============================================================================
// Kinematics Solver / 运动学解算器
// ============================================================================
hybrid_kin_t::hybrid_speeds_t
hybrid_kin_t::solve(const float vx, const float vy, const float wz, const bool track_en) const
{
    hybrid_speeds_t ws{}; // Zero initialize / 初始化为零

    // Pre-calculate rotational velocity components for each independent wheel
    // 根据各自的K值，预计算每个轮子因旋转产生的切向速度分量 (v = wz * K)
    const float v_rot_fl = wz * _k_fl;
    const float v_rot_fr = wz * _k_fr;
    const float v_rot_bl = wz * _k_bl;
    const float v_rot_br = wz * _k_br;

    if (!track_en)
    {
        // --- Mode: CRUISING / 巡航模式 ---
        // Tracks are lifted or idle. Only Mecanum handles full Omni-directional movement.
        // 履带悬空或怠速。纯麦轮提供全向移动能力。
        ws.track_l = 0.0f;
        ws.track_r = 0.0f;

        // Apply independent rotation components
        // 麦轮公式：代入各自独立的旋转分量
        ws.mec_fl = vx - vy - v_rot_fl;
        ws.mec_fr = vx + vy + v_rot_fr;
        ws.mec_bl = vx + vy - v_rot_bl;
        ws.mec_br = vx - vy + v_rot_br;
    }
    else
    {
        // --- Mode: CLIMBING (OBSTACLE) / 爬坡(越障)模式 ---
        // 1. Force Vy to 0.
        // Physics constraint: Tracks have immense friction sideways. Attempting to strafe will stall motors.
        // 物理限制：履带横向摩擦力极大，履带接地时强行横移会导致电机堵转，因此强制 Vy = 0。
        constexpr float effective_vy = 0.0f;

        // 2. Tracks Logic (Differential Drive)
        // 履带差速逻辑：只提供前进和基于左右间距的旋转差速
        const float v_rot_track = wz * _k_track;
        ws.track_l = vx - v_rot_track;
        ws.track_r = vx + v_rot_track;

        // 3. Mecanum Logic (Assisting Differential Drive)
        // 麦轮辅助逻辑：即使无横移，麦轮仍需输出准确的旋转速度以配合履带，防止产生拖拽阻力
        ws.mec_fl = vx - effective_vy - v_rot_fl;
        ws.mec_fr = vx + effective_vy + v_rot_fr;
        ws.mec_bl = vx + effective_vy - v_rot_bl;
        ws.mec_br = vx - effective_vy + v_rot_br;
    }

    return ws;
}

} // namespace pyro