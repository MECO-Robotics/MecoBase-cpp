#pragma once

#include <frc/geometry/Translation2d.h>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace base {
// PathPlanner occupancy grid in blue-origin field coordinates, metres.
struct NavigationGrid {
    double cellSize = 0;
    std::vector<std::vector<bool>> blocked;
};

// Read the PathPlanner navgrid.json at path. Missing or invalid files return
// nullopt. The caller selects the file path and handles failure.
std::optional<NavigationGrid> LoadNavigationGrid(const std::string& path);

// True when the field-coordinate point lies in an unblocked cell. Invalid
// coordinates, cell sizes, and points outside the selected row return false.
bool IsNavigationGridPointFree(const NavigationGrid& grid, frc::Translation2d point);

// Find the first accepted point by eight-neighbour shortest path distance.
// First consider start itself, even if its cell is blocked, then reachable cell
// centres. Diagonal travel cannot cut blocked corners. Equal-cost cells use
// row-major order. Invalid grids, nonfinite coordinates and unreachable goals
// return nullopt; a goal may accept a finite start outside the grid.
std::optional<frc::Translation2d> FindReachableGridPoint(
    const NavigationGrid& grid, frc::Translation2d start,
    const std::function<bool(frc::Translation2d)>& accept);
} // namespace base
