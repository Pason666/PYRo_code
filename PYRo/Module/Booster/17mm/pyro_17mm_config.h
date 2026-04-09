#ifndef __PYRO_PYRO_17MM_CONFIG_H__
#define __PYRO_PYRO_17MM_CONFIG_H__

namespace pyro
{
// 速度与角度设定
constexpr float SHOOT_BULLET_MUZZLE_VELOCITY = 22.2; // 枪口初速度
constexpr float FRICTION_WHEEL_RADIUS        = 0.03f;
constexpr float SHOOT_FIRE_RADPS =
    (-SHOOT_BULLET_MUZZLE_VELOCITY / FRICTION_WHEEL_RADIUS); // 摩擦轮角速度

constexpr float TRIGGER_UNJAM_RADPS      = -6.0f; // 解堵速度
constexpr float TRIGGER_CONTINUOUS_RADPS = 10;    // 连续发射速度（拨弹盘速度）
constexpr float TRIGGER_SINGLE_JAM_SPEED = 10.0f; // 单发堵转判定速度
// 发射速度 (发/秒)
constexpr float SHOOT_SPEED                                       = 15.0f;
// 拨弹盘速度
constexpr float TRIGGER_SPEED                                     = SHOOT_SPEED / 8.0f * 2.0f * PI * 36.0f;

constexpr float TRIGGER_GEAR_RATIO       = 36.0f; // M2006拨弹电机减速比

} // namespace pyro

#endif // __PYRO_PYRO_17MM_CONFIG_H__
