#ifndef PYRO_HEAT_CONTROLLER_H
#define PYRO_HEAT_CONTROLLER_H

#include <algorithm>
#include <cstdint>

namespace pyro
{

class HeatController
{
  public:
    static constexpr float HEAT_PER_BULLET    = 10.0f;
    static constexpr float BULLETS_PER_CIRCLE = 8.0f;
    static constexpr float SAFE_MARGIN        = 50.0f;
    static constexpr uint32_t REFEREE_DELAY_MS = 200;

    HeatController() = default;

    void tickCooling(float dt)
    {
        if (_coolingRate > 0.0f && _localHeat > 0.0f)
        {
            _localHeat -= _coolingRate * dt;
            if (_localHeat < 0.0f)
                _localHeat = 0.0f;
        }
    }

    void syncWithReferee(uint16_t refHeat, uint16_t refLimit,
                         uint16_t refCoolingRate, uint32_t current_time_ms)
    {
        _heatLimit   = static_cast<float>(refLimit);
        _coolingRate = static_cast<float>(refCoolingRate);

        if (_heatLimit <= 0.0f)
            return;

        if ((current_time_ms - _lastShotTimeMs) > REFEREE_DELAY_MS)
            _localHeat = static_cast<float>(refHeat);
        else
            _localHeat = std::max(_localHeat, static_cast<float>(refHeat));
    }

    void recordBulletShot(uint32_t current_time_ms)
    {
        _localHeat += HEAT_PER_BULLET;
        _lastShotTimeMs = current_time_ms;
    }

    [[nodiscard]] bool canShootSingle() const
    {
        if (_heatLimit <= 0.0f)
            return true;
        return (_localHeat + HEAT_PER_BULLET) <= (_heatLimit - SAFE_MARGIN);
    }

    [[nodiscard]] bool isApproachingHeatLimit() const
    {
        if (_heatLimit <= 0.0f)
            return false;
        return (_localHeat + HEAT_PER_BULLET * 2.0f) > (_heatLimit - SAFE_MARGIN);
    }

    // 返回拨弹盘目标角速度 (rad/s)。三区间: 安全区=全速, 维持区=冷却速率受限, 临界区=制动
    [[nodiscard]] float getSafeBurstRadps(float maxRadps) const
    {
        if (_heatLimit <= 0.0f)
            return maxRadps;
        if (_localHeat + HEAT_PER_BULLET > _heatLimit - SAFE_MARGIN)
            return -maxRadps * 0.12f;
        if (_localHeat < _heatLimit - SAFE_MARGIN - HEAT_PER_BULLET * 2.0f)
            return maxRadps;

        // 维持区: 根据冷却速率计算可持续的每秒发射数, 转换为拨弹盘角速度
        float sustainBulletsPerSec = (_coolingRate / HEAT_PER_BULLET) * 0.95f;
        float sustainRadps = sustainBulletsPerSec * (2.0f * 3.14159265f / BULLETS_PER_CIRCLE);
        return std::min(sustainRadps, maxRadps);
    }

    [[nodiscard]] float getLocalHeat() const { return _localHeat; }

  private:
    float _localHeat{0.0f};
    float _heatLimit{0.0f};
    float _coolingRate{0.0f};
    uint32_t _lastShotTimeMs{0};
};

} // namespace pyro

#endif // PYRO_HEAT_CONTROLLER_H
