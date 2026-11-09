#include "base/NavigationGrid.h"
#include "base/Replay.h"

#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>
#include <cmath>
#include <climits>
#include <limits>
#include <numbers>
#include <queue>

namespace base {
namespace {
bool ValidGrid(const NavigationGrid& grid) {
    if (!std::isfinite(grid.cellSize) || grid.cellSize <= 0 || grid.blocked.empty() || grid.blocked[0].empty())
        return false;
    const auto width = grid.blocked[0].size();
    if (width > INT_MAX || grid.blocked.size() > static_cast<std::size_t>(INT_MAX) / width)
        return false;
    for (const auto& row : grid.blocked)
        if (row.size() != width) return false;
    return true;
}
}
std::optional<NavigationGrid> LoadNavigationGrid(const std::string& path) {
    const auto file = wpi::MemoryBuffer::GetFile(path);
    if (!file) return std::nullopt;
    const auto contents = file.value()->GetCharBuffer();
    return ParseNavigationGrid(std::string_view{contents.data(), contents.size()});
}
std::optional<NavigationGrid> ProcessNavigationGridInput(
    std::string_view key, const std::string& path, std::size_t maxBytes) {
    const auto contents = replay::ProcessFileInput(key, path, maxBytes);
    return contents ? ParseNavigationGrid(*contents) : std::nullopt;
}
std::optional<NavigationGrid> ParseNavigationGrid(std::string_view contents) {
    try {
        const auto json = wpi::json::parse(contents);
        NavigationGrid result;
        result.cellSize = json.at("nodeSizeMeters").get<double>();
        result.blocked = json.at("grid").get<std::vector<std::vector<bool>>>();
        if (!ValidGrid(result)) return std::nullopt;
        return result;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}
bool IsNavigationGridPointFree(const NavigationGrid& grid, frc::Translation2d point) {
    if (!std::isfinite(grid.cellSize) || grid.cellSize <= 0 ||
        !std::isfinite(point.X().value()) || !std::isfinite(point.Y().value())) return false;
    const double x = std::floor(point.X().value() / grid.cellSize);
    const double y = std::floor(point.Y().value() / grid.cellSize);
    if (x < 0 || y < 0 || y >= grid.blocked.size()) return false;
    const auto& row = grid.blocked[static_cast<std::size_t>(y)];
    if (x >= row.size()) return false;
    return !row[static_cast<std::size_t>(x)];
}
std::optional<frc::Translation2d> FindReachableGridPoint(
    const NavigationGrid& grid, frc::Translation2d start,
    const std::function<bool(frc::Translation2d)>& accept) {
    if (!ValidGrid(grid) || !accept || !std::isfinite(start.X().value()) || !std::isfinite(start.Y().value()))
        return std::nullopt;
    const auto rows = static_cast<int>(grid.blocked.size());
    const auto cols = static_cast<int>(grid.blocked[0].size());
    const auto free = [&](int x, int y) { return x >= 0 && y >= 0 && x < cols && y < rows && !grid.blocked[y][x]; };
    if (accept(start)) return start;
    const double cellX = std::floor(start.X().value() / grid.cellSize);
    const double cellY = std::floor(start.Y().value() / grid.cellSize);
    if (cellX < 0 || cellY < 0 || cellX >= cols || cellY >= rows) return std::nullopt;
    const int sx = static_cast<int>(cellX), sy = static_cast<int>(cellY);
    if (!free(sx, sy)) return std::nullopt;
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::vector<double> distance(rows * cols, std::numeric_limits<double>::infinity());
    distance[sy * cols + sx] = 0;
    open.emplace(0, sy * cols + sx);
    const double diagonalCost = grid.cellSize * std::numbers::sqrt2;
    while (!open.empty()) {
        const auto [cost, index] = open.top(); open.pop();
        if (cost > distance[index]) continue;
        const int x = index % cols, y = index / cols;
        const frc::Translation2d position{units::meter_t{(x + .5) * grid.cellSize},
                                          units::meter_t{(y + .5) * grid.cellSize}};
        if (accept(position)) return position;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                if ((!dx && !dy) || !free(x + dx, y + dy) || (dx && dy && (!free(x + dx, y) || !free(x, y + dy))))
                    continue;
                const int next = (y + dy) * cols + x + dx;
                const double nextCost = cost + (dx && dy ? diagonalCost : grid.cellSize);
                if (nextCost < distance[next]) { distance[next] = nextCost; open.emplace(nextCost, next); }
            }
    }
    return std::nullopt;
}
} // namespace base
