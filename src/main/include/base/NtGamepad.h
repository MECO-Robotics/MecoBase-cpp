#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace base
{
  namespace ntgamepadconfig
  {
    // Default NT subscription polling interval, in milliseconds.
    inline constexpr double kDefaultPeriodMs = 5.0;
    // Maximum sample age before the input is treated as stale, in milliseconds.
    inline constexpr double kDefaultStaleMs = 50.0;
    // Allowed NT/source timestamp disagreement and future-clock offset, in microseconds.
    inline constexpr int64_t kMaxTimestampDisagreementUs = 20000, kMaxFutureOffsetUs = 5000;
  }

  // Axes-only input supplied over NT4. Buttons and enable authority stay on Driver Station.
  class NtGamepad
  {
  public:
    struct State
    {
      uint64_t sequence{};
      int64_t sourceTimestampUs{};
      double ageMs{std::numeric_limits<double>::infinity()};
      double leftY{}, leftX{}, rightX{};
      bool accepted{};
    };

    explicit NtGamepad(nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault(),
                       double periodMs = ntgamepadconfig::kDefaultPeriodMs, double staleMs = ntgamepadconfig::kDefaultStaleMs)
        : subscriber_(instance.GetDoubleArrayTopic("/Input/Gamepad").Subscribe(std::span<const double>{}, SubscribeOptions(periodMs))),
          staleUs_(static_cast<int64_t>(staleMs * 1000.0))
    {
    }

    // Payload layout: [version=1, sequence, source NT timestamp in us, leftY, leftX, rightX].
    static bool ValidPayload(std::span<const double> value)
    {
      if (value.size() != 6 || value[0] != 1.0 || !Integer(value[1]) || !Integer(value[2]))
        return false;
      return Axis(value[3]) && Axis(value[4]) && Axis(value[5]);
    }

    State Read(int64_t nowUs = nt::Now())
    {
      const auto sample = subscriber_.GetAtomic();
      // This subscriber lives on the robot NT server; local time is the server clock.
      // serverTime can be the sentinel 1 for locally published values.
      const int64_t publisherTime = sample.time;
      const int64_t embeddedTime = sample.value.size() == 6 && Integer(sample.value[2])
                                       ? static_cast<int64_t>(sample.value[2])
                                       : 0;
      const int64_t ageUs = nowUs - publisherTime;
      const int64_t embeddedAgeUs = nowUs - embeddedTime;

      bool validSample = false;
      if (ValidPayload(sample.value))
      {
        const auto sequence = static_cast<uint64_t>(sample.value[1]);
        if ((!hasSequence_ || sequence > lastSequence_) && Fresh(ageUs) && Fresh(embeddedAgeUs) &&
            std::abs(publisherTime - embeddedTime) <= ntgamepadconfig::kMaxTimestampDisagreementUs)
        {
          hasSequence_ = true;
          lastSequence_ = sequence;
          lastPublisherTimeUs_ = publisherTime;
          state_ = {sequence, embeddedTime, ageUs / 1000.0,
                    sample.value[3], sample.value[4], sample.value[5], true};
          validSample = true;
        }
        else if (hasSequence_ && sequence == lastSequence_ &&
                 embeddedTime == state_.sourceTimestampUs && sample.value[3] == state_.leftY &&
                 sample.value[4] == state_.leftX && sample.value[5] == state_.rightX &&
                 Fresh(nowUs - lastPublisherTimeUs_) && Fresh(nowUs - state_.sourceTimestampUs))
        {
          // Re-reading the same latest NT value is normal between producer updates.
          state_.ageMs = (nowUs - lastPublisherTimeUs_) / 1000.0;
          state_.accepted = true;
          validSample = true;
        }
      }

      if (!validSample || !Fresh(nowUs - lastPublisherTimeUs_) || !Fresh(nowUs - state_.sourceTimestampUs))
      {
        state_.accepted = false;
        state_.leftY = state_.leftX = state_.rightX = 0.0;
        if (hasSequence_)
          state_.ageMs = (nowUs - lastPublisherTimeUs_) / 1000.0;
      }
      return state_;
    }

  private:
    static nt::PubSubOptions SubscribeOptions(double periodMs)
    {
      nt::PubSubOptions options;
      options.periodic = periodMs / 1000.0;
      options.sendAll = true;
      return options;
    }
    static bool Integer(double value)
    {
      return std::isfinite(value) && value >= 0.0 && value <= 9007199254740991.0 && std::floor(value) == value;
    }
    static bool Axis(double value) { return std::isfinite(value) && value >= -1.0 && value <= 1.0; }
    bool Fresh(int64_t ageUs) const { return ageUs >= -ntgamepadconfig::kMaxFutureOffsetUs && ageUs <= staleUs_; }

    nt::DoubleArraySubscriber subscriber_;
    State state_;
    uint64_t lastSequence_{};
    int64_t lastPublisherTimeUs_{};
    bool hasSequence_{};
    int64_t staleUs_{static_cast<int64_t>(ntgamepadconfig::kDefaultStaleMs * 1000.0)};
  };
} // namespace base
