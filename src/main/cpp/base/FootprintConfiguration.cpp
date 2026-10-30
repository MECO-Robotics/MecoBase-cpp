#include "base/FootprintConfiguration.h"
#include <wpi/json.h>
#include <base/FootprintTerrainMap.h>
#include <memory>
#include <algorithm>
#include <cmath>

namespace base {
namespace {
double Number(const wpi::json& value, const char* key) {
    const auto& item=value.at(key);
    if (!item.is_number()) throw std::invalid_argument("expected number");
    const double result=item.get<double>();
    if (!std::isfinite(result)) throw std::invalid_argument("nonfinite number");
    return result;
}
base::FieldRectangle Rectangle(const wpi::json& value) {
    base::FieldRectangle r{Number(value,"min_x"),Number(value,"min_y"),
                           Number(value,"max_x"),Number(value,"max_y")};
    if (r.minX>=r.maxX || r.minY>=r.maxY) throw std::invalid_argument("invalid rectangle");
    return r;
}
}
std::optional<FootprintConfiguration> ParseFootprintConfiguration(std::string_view text) {
    try {
        const auto json=wpi::json::parse(text.begin(),text.end());
        if (!json.at("version").is_number_integer() || json.at("version").get<int>()!=1) return std::nullopt;
        FootprintConfiguration result;
        const auto& chassis=json.at("footprint");
        const double length=Number(chassis,"length_m"), width=Number(chassis,"width_m");
        const double margin=Number(chassis,"margin_m");
        if (length<=0 || width<=0 || margin<0) return std::nullopt;
        result.drive.footprint={length+2*margin,width+2*margin};
        result.drive.field=Rectangle(json.at("field"));
        const auto& field=result.drive.field;
        if (result.drive.footprint.length>field.maxX-field.minX ||
            result.drive.footprint.width>field.maxY-field.minY) return std::nullopt;
        const auto& boxes=json.at("obstacles");
        if (!boxes.is_array() || boxes.empty() || boxes.size()>1024) return std::nullopt;
        for (const auto& box:boxes) {
            const auto r=Rectangle(box);
            if (r.maxX<field.minX || r.minX>field.maxX || r.maxY<field.minY || r.minY>field.maxY)
                return std::nullopt;
            result.obstacles.push_back(r);
        }
        const auto& planner=json.at("planner");
        result.drive.resolution=Number(planner,"resolution_m");
        const auto& budget=planner.at("expansion_budget");
        if (!budget.is_number_integer()) return std::nullopt;
        result.drive.expansionBudget=budget.get<int>();
        const auto& limits=json.at("limits");
        result.drive.follow.maxSpeed=Number(limits,"max_speed_mps");
        result.drive.follow.maxOmega=Number(limits,"max_omega_radps");
        result.drive.follow.maxAcceleration=Number(limits,"max_acceleration_mps2");
        result.drive.follow.maxAlpha=Number(limits,"max_alpha_radps2");
        if (result.drive.resolution<.05 || result.drive.resolution>1 ||
            result.drive.expansionBudget<1 || result.drive.expansionBudget>200000 ||
            result.drive.follow.maxSpeed<=0 || result.drive.follow.maxOmega<=0 ||
            result.drive.follow.maxAcceleration<=0 || result.drive.follow.maxAlpha<=0 ||
            !std::isfinite(result.drive.footprint.length) || !std::isfinite(result.drive.footprint.width))
            return std::nullopt;
        if (json.contains("terrain")) {
            const auto& terrain=json.at("terrain");
            const auto& regions=terrain.at("regions");
            if (!regions.is_array() || regions.empty() || regions.size()>128) return std::nullopt;
            std::vector<base::FootprintTerrainRegion> data;
            for (const auto& item:regions) {
                const auto r=Rectangle(item.at("bounds"));
                if (r.minX<field.minX || r.maxX>field.maxX || r.minY<field.minY || r.maxY>field.maxY)
                    return std::nullopt;
                data.push_back({r,Number(item,"grade_x"),Number(item,"grade_y"),Number(item,"rolling_resistance")});
            }
            result.drive.follow.profileLateralAcceleration=Number(terrain,"lateral_acceleration_mps2");
            result.drive.follow.profileSteeringRate=Number(terrain,"steering_rate_radps");
            result.drive.follow.translationStallTimeout=Number(terrain,"stall_timeout_s");
            result.drive.follow.translationStallProgress=Number(terrain,"stall_progress_m");
            result.drive.follow.translationStallRequest=Number(terrain,"stall_request_mps");
            if (result.drive.follow.profileLateralAcceleration<=0 || result.drive.follow.profileSteeringRate<=0 ||
                result.drive.follow.translationStallTimeout<=0 || result.drive.follow.translationStallProgress<=0 ||
                result.drive.follow.translationStallRequest<=0 ||
                result.drive.follow.translationStallRequest>=result.drive.follow.maxSpeed)
                return std::nullopt;
            auto map=std::make_shared<const base::FootprintTerrainMap>(std::move(data));
            result.drive.follow.terrain=[map](base::FootprintPose a,base::FootprintPose b) {return map->Segment(a,b);};
            result.drive.follow.terrainBoundaries=[map](base::FootprintPose a,base::FootprintPose b) {return map->Boundaries(a,b);};
        }
        return result;
    } catch (...) {return std::nullopt;}
}
}
