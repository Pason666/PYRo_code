#ifndef __PYRO_PYRO_17MM_CONFIG_H__
#define __PYRO_PYRO_17MM_CONFIG_H__

namespace pyro
{
// 速度与角度设定
constexpr float SHOOT_BULLET_MUZZLE_VELOCITY = 22.2; // 枪口初速度
constexpr float FRICTION_WHEEL_RADIUS        = 0.03f;
constexpr float SHOOT_FIRE_RADPS =
    (-SHOOT_BULLET_MUZZLE_VELOCITY / FRICTION_WHEEL_RADIUS); // 摩擦轮角速度

// constexpr float TRIGGER_UNJAM_RADPS      = 6.0f;   // 解堵速度
constexpr float TRIGGER_CONTINUOUS_RADPS = 20; // 连续发射速度（拨弹盘速度）

// 堵转判定
// constexpr float TRIGGER_BLOCK_RAD        = 0.2f; // 堵转判定弧度阈值
// constexpr float TRIGGER_BLOCK_RADPS      = 5.0f; // 堵转判定速度阈值
// constexpr uint16_t TRIGGER_BLOCK_TIME    = 300;  // 堵转判定时间阈值

constexpr float TRIGGER_GEAR_RATIO       = 36.0f; // M2006拨弹电机减速比

}

#endif // __PYRO_PYRO_17MM_CONFIG_H__
