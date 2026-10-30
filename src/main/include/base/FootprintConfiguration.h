#pragma once
#include <base/FootprintAutoDrive.h>
#include <optional>
#include <string_view>
#include <vector>

namespace base
{
    // Application-owned geometry and limits; all lengths are metres and angles radians.
    struct FootprintConfiguration
    {
        base::FootprintAutoDriveConfig drive;
        std::vector<base::FieldRectangle> obstacles;
    };
    // Missing, malformed, or nonfinite required values return no configuration.
    std::optional<FootprintConfiguration> ParseFootprintConfiguration(std::string_view text);
}
