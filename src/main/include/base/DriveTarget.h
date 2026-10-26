#pragma once
#include <array>
#include <algorithm>
#include <frc/geometry/Rotation2d.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <frc/kinematics/ChassisSpeeds.h>

namespace base
{
// Commands own the subsystem and publish intent; only the native control thread
// consumes it. A contended reader keeps its previous target, never waits.
class DriveTarget
{
  public:
    struct Value
    {
        frc::ChassisSpeeds speeds{};
        std::array<double, 4> forceX{}, forceY{}; // newtons, module order
        bool brake = false, fieldRelative = false, headingControl = false, teleopOnly = false;
        frc::Rotation2d heading{};
        double headingKP = 0, maxOmega = 0;
        double timestamp = 0;
        uint64_t epoch = 0;
    };
    template <class Enabled> void Publish(Value value, Enabled &&enabled)
    {
        // Capture the epoch before authorization: a disable between either check and
        // the write must invalidate this packet, including across a fast re-enable.
        value.epoch = epoch_.load();
        if (!enabled())
        {
            Invalidate();
            return;
        }
        std::scoped_lock lock(mutex_);
        pending_ = value;
    }
    // May be called by the robot thread even if no native tick observes disable.
    // Only the consumer mutates cached_; invalidation itself is atomic.
    inline void Invalidate()
    {
        epoch_.fetch_add(1);
    }
    inline Value Consume(double now, bool enabled)
    {
        if (!enabled)
        {
            Invalidate();
            cached_ = {};
            return {};
        }
        if (std::unique_lock lock(mutex_, std::try_to_lock); lock.owns_lock())
            cached_ = pending_;
        const auto &s = cached_.speeds;
        bool valid = std::isfinite(now) && std::isfinite(cached_.timestamp) && now >= cached_.timestamp &&
                     (cached_.brake || now - cached_.timestamp <= .1) && cached_.epoch == epoch_.load() &&
                     std::isfinite(cached_.heading.Radians().value()) && std::isfinite(cached_.headingKP) &&
                     std::isfinite(cached_.maxOmega) && cached_.maxOmega >= 0 && std::isfinite(s.vx.value()) &&
                     std::isfinite(s.vy.value()) && std::isfinite(s.omega.value());
        for (int i = 0; i < 4; ++i)
            valid = valid && std::isfinite(cached_.forceX[i]) && std::isfinite(cached_.forceY[i]);
        return valid ? cached_ : Value{};
    }
    static frc::ChassisSpeeds Resolve(const Value &target, frc::Rotation2d measuredHeading)
    {
        if ((target.fieldRelative || target.headingControl) && !std::isfinite(measuredHeading.Radians().value()))
            return {};
        auto speeds = target.speeds;
        if (target.headingControl)
            speeds.omega = units::radians_per_second_t{
                std::clamp(target.headingKP * (target.heading - measuredHeading).Radians().value(), -target.maxOmega,
                           target.maxOmega)};
        return target.fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, measuredHeading) : speeds;
    }

  private:
    std::mutex mutex_;
    std::atomic<uint64_t> epoch_{1};
    Value pending_{}, cached_{};
};
} // namespace base
