// Adapted from PathPlanner v2026.1.2 (MIT), copyright Michael Jansen.
// See NativeADStar_LICENSE.txt. Refreshes identical/sub-cell requests and owns its worker.
#include "NativeADStar.h"
#include "pathplanner/lib/util/GeometryUtil.h"
#include <cmath>
#include <frc/Filesystem.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>
#include <chrono>
#include <frc/Errors.h>
#include <queue>
#include <stdexcept>

using namespace pathplanner;
using base::detail::NativeADStar;

NativeADStar::NativeADStar(std::optional<std::string> navgridPath) : fieldLength(16.54), fieldWidth(8.02), nodeSize(
		0.2), nodesX(static_cast<int>(std::ceil(fieldLength / nodeSize))), nodesY(
		static_cast<int>(std::ceil(fieldWidth / nodeSize))), g(), rhs(), open(), incons(), closed(), staticObstacles(), dynamicObstacles(), requestObstacles(), requestStart(), requestRealStartPos(), requestGoal(), requestRealGoalPos(), eps(
		EPS), planningThread(), pathMutex(), requestMutex(), requestMinor(true), requestMajor(
		true), requestReset(true), newPathAvailable(false), currentWaypoints() {
	requestStart = GridPosition(0, 0);
	requestRealStartPos = frc::Translation2d(0_m, 0_m);
	requestGoal = GridPosition(0, 0);
	requestRealGoalPos = frc::Translation2d(0_m, 0_m);

	staticObstacles.clear();
	dynamicObstacles.clear();

	const std::string filePath = navgridPath.value_or(frc::filesystem::GetDeployDirectory()
			+ "/pathplanner/navgrid.json");

	auto fileBuffer = wpi::MemoryBuffer::GetFile(filePath);

	if (fileBuffer) {
		try {
			wpi::json json = wpi::json::parse(
					fileBuffer.value()->GetCharBuffer());

			nodeSize = json.at("nodeSizeMeters").get<double>();
			wpi::json::const_reference grid = json.at("grid");
			nodesY = grid.size();
			for (size_t row = 0; row < grid.size(); row++) {
				wpi::json::const_reference rowArr = grid[row];
				if (row == 0) {
					nodesX = rowArr.size();
				}
                if (rowArr.size() != static_cast<size_t>(nodesX)) throw std::invalid_argument("Non-rectangular navgrid");
				for (size_t col = 0; col < rowArr.size(); col++) {
					bool isObstacle = rowArr[col].get<bool>();
					if (isObstacle) {
						staticObstacles.emplace(col, row);
					}
				}
			}

			wpi::json::const_reference fieldSize = json.at("field_size");
			fieldLength = fieldSize.at("x").get<double>();
			fieldWidth = fieldSize.at("y").get<double>();
            navgridLoaded = std::isfinite(nodeSize) && nodeSize > 0 && nodesX > 0 && nodesY > 0 &&
                std::isfinite(fieldLength) && fieldLength > 0 && std::isfinite(fieldWidth) && fieldWidth > 0;
		} catch (...) {
			// Ignore, just use defaults
		}
	}

	requestObstacles.clear();
	requestObstacles.insert(staticObstacles.begin(), staticObstacles.end());
	requestObstacles.insert(dynamicObstacles.begin(), dynamicObstacles.end());

	requestReset = true;
	requestMajor = true;
	requestMinor = true;

	newPathAvailable = false;

	planningThread = std::jthread([this](std::stop_token stop) { runThread(stop); });
}

void NativeADStar::runThread(std::stop_token stop) {
	while (!stop.stop_requested()) {
		try {
			bool reset;
			bool minor;
			bool major;
			GridPosition start;
			frc::Translation2d realStart;
			GridPosition goal;
			frc::Translation2d realGoal;
			std::unordered_set < GridPosition > obstacles;

			{
				std::scoped_lock lock { requestMutex };
				reset = requestReset;
				minor = requestMinor;
				major = requestMajor;
				start = requestStart;
				realStart = requestRealStartPos;
				goal = requestGoal;
				realGoal = requestRealGoalPos;
				obstacles.insert(requestObstacles.begin(),
						requestObstacles.end());

				// Change the request booleans based on what will be done this loop
				if (reset) {
					requestReset = false;
				}

				if (minor) {
					requestMinor = false;
				} else if (major && (eps - 0.5) <= 1.0) {
					requestMajor = false;
				}
			}

			if (reset || minor || major) {
				doWork(reset, minor, major, start, goal, realStart, realGoal,
						obstacles);
			} else {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		} catch (...) {
			// Something messed up. Reset and hope for the best
			std::scoped_lock lock { requestMutex };
			requestReset = true;
		}
	}
}

void NativeADStar::doWork(const bool needsReset, const bool doMinor,
		const bool doMajor, const GridPosition &sStart,
		const GridPosition &sGoal, const frc::Translation2d &realStartPos,
		const frc::Translation2d &realGoalPos,
		const std::unordered_set<GridPosition> &obstacles) {
	if (needsReset) {
		reset(sStart, sGoal);
	}

	if (doMinor) {
		computeOrImprovePath(sStart, sGoal, obstacles);
		std::vector < GridPosition > pathPositions = extractPath(sStart, sGoal,
				obstacles);
		std::vector < Waypoint > waypoints = createWaypoints(pathPositions,
				realStartPos, realGoalPos, obstacles);

		{
			std::scoped_lock lock { pathMutex };
			currentPathFull = pathPositions;
			currentWaypoints = waypoints;
		}

		newPathAvailable = true;
	} else if (doMajor) {
		if (eps > 1.0) {
			eps -= 0.5;
			open.insert(incons.begin(), incons.end());

			for (auto entry : open) {
				open[entry.first] = key(entry.first, sStart);
			}
			closed.clear();
			computeOrImprovePath(sStart, sGoal, obstacles);
			std::vector < GridPosition > pathPositions = extractPath(sStart,
					sGoal, obstacles);
			std::vector < Waypoint > waypoints = createWaypoints(pathPositions,
					realStartPos, realGoalPos, obstacles);

			{
				std::scoped_lock lock { pathMutex };
				currentPathFull = pathPositions;
				currentWaypoints = waypoints;
			}

			newPathAvailable = true;
		}
	}
}

std::shared_ptr<PathPlannerPath> NativeADStar::getCurrentPath(
		PathConstraints constraints, GoalEndState goalEndState) {
	std::vector < Waypoint > waypoints;

	{
		std::scoped_lock lock { pathMutex };
		waypoints.insert(waypoints.end(), currentWaypoints.begin(),
				currentWaypoints.end());
	}

	newPathAvailable = false;

	if (waypoints.size() < 2) {
		// Not enough points to make a path
		return nullptr;
	}

	return std::make_shared < PathPlannerPath
			> (waypoints, constraints, std::nullopt, goalEndState);
}

bool NativeADStar::DirectSegmentClear(const frc::Translation2d& start, const frc::Translation2d& goal) {
    const double x0=start.X().value(), y0=start.Y().value(), x1=goal.X().value(), y1=goal.Y().value();
    if (!navgridLoaded || !std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1) ||
        x0<0 || y0<0 || x1<0 || y1<0 || x0>=fieldLength || x1>=fieldLength || y0>=fieldWidth || y1>=fieldWidth) return false;
    std::scoped_lock lock{requestMutex};
    const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(x1-x0,y1-y0)/(nodeSize*.1))));
    auto previous=getGridPos(start);
    for(int i=0;i<=steps;++i) {
        const double t=static_cast<double>(i)/steps;
        const auto cell=getGridPos({units::meter_t{x0+(x1-x0)*t}, units::meter_t{y0+(y1-y0)*t}});
        if(cell.x<0 || cell.y<0 || cell.x>=nodesX || cell.y>=nodesY || requestObstacles.contains(cell)) return false;
        // A diagonal crossing must not cut between blocked adjacent cells.
        if(cell.x!=previous.x && cell.y!=previous.y &&
           (requestObstacles.contains(GridPosition{cell.x,previous.y}) ||
            requestObstacles.contains(GridPosition{previous.x,cell.y}))) return false;
        previous=cell;
    }
    return true;
}

void NativeADStar::setStartPosition(const frc::Translation2d &start) {
	GridPosition startPos = findClosestNonObstacle(getGridPos(start),
			requestObstacles);

	{ // Refresh exact start coordinates even within the same grid cell.
		std::scoped_lock lock { requestMutex };
		requestStart = startPos;
		requestRealStartPos = start;

		requestMinor = true;
		newPathAvailable = false;
	}
}

void NativeADStar::setGoalPosition(const frc::Translation2d &goal) {
	GridPosition gridPos = findClosestNonObstacle(getGridPos(goal),
			requestObstacles);

	{ // A new command needs a new path even for the same goal cell.
		std::scoped_lock lock { requestMutex };
		requestGoal = gridPos;
		requestRealGoalPos = goal;

		requestMinor = true;
		requestMajor = true;
		requestReset = true;
		newPathAvailable = false;
	}
}

GridPosition NativeADStar::findClosestNonObstacle(const GridPosition &pos,
		const std::unordered_set<GridPosition> &obstacles) {
	if (!obstacles.contains(pos)) {
		return pos;
	}

	std::unordered_set < GridPosition > visited;
	// Workaround to be able to see which what nodes are in the queue while maintaining the ordering of the queue
	std::unordered_set < GridPosition > workaround;
	std::queue < GridPosition > queue;
	for (const GridPosition &gp : getAllNeighbors(pos)) {
		queue.push(gp);
	}

	while (!queue.empty()) {
		GridPosition check = queue.front();
		workaround.erase(check);
		if (!obstacles.contains(check)) {
			return check;
		}
		visited.emplace(check);

		for (const GridPosition &neighbor : getAllNeighbors(check)) {
			if (!visited.contains(neighbor) && !workaround.contains(check)) {
				queue.push(neighbor);
				workaround.emplace(neighbor);
			}
		}

		queue.pop();
	}

	// Somehow didn't find one, return the original position cuz everything would be messed up anyways
	return pos;
}

void NativeADStar::setDynamicObstacles(
		const std::vector<std::pair<frc::Translation2d, frc::Translation2d>> &obs,
		const frc::Translation2d &currentRobotPos) {
	std::unordered_set < GridPosition > newObs;

	for (auto obstacle : obs) {
		GridPosition gridPos1 = getGridPos(obstacle.first);
		GridPosition gridPos2 = getGridPos(obstacle.second);

		int minX = std::min(gridPos1.x, gridPos2.x);
		int maxX = std::max(gridPos1.x, gridPos2.x);

		int minY = std::min(gridPos1.y, gridPos2.y);
		int maxY = std::max(gridPos1.y, gridPos2.y);

		for (int x = minX; x <= maxX; x++) {
			for (int y = minY; y <= maxY; y++) {
				newObs.emplace(x, y);
			}
		}
	}

	dynamicObstacles.clear();
	dynamicObstacles.insert(newObs.begin(), newObs.end());

	{
		std::scoped_lock lock { requestMutex };
		requestObstacles.clear();
		requestObstacles.insert(staticObstacles.begin(), staticObstacles.end());
		requestObstacles.insert(dynamicObstacles.begin(),
				dynamicObstacles.end());
	}

	bool recalculate = false;
	{
		std::scoped_lock lock { pathMutex };
		for (GridPosition pos : currentPathFull) {
			if (requestObstacles.contains(pos)) {
				recalculate = true;
				break;
			}
		}
	}

	if (recalculate) {
		setStartPosition(currentRobotPos);
		setGoalPosition (requestRealGoalPos);
	}
}

std::vector<GridPosition> NativeADStar::extractPath(const GridPosition &sStart,
		const GridPosition &sGoal,
		const std::unordered_set<GridPosition> &obstacles) {
	if (sGoal == sStart) {
		std::vector < GridPosition > ret;
		return ret;
	}

	std::vector < GridPosition > path;
	path.push_back(sStart);

	GridPosition s = sStart;

	for (int k = 0; k < 200; k++) {
		std::unordered_map<GridPosition, double> gList;

		for (const GridPosition &x : getOpenNeighbors(s, obstacles)) {
			gList[x] = g.at(x);
		}

		auto min = std::pair<GridPosition, double>(sGoal,
				std::numeric_limits<double>::infinity());
		for (auto entry : gList) {
			if (entry.second < min.second) {
				min = entry;
			}
		}
		s = min.first;

		path.push_back(s);
		if (s == sGoal) {
			break;
		}
	}
	return path;
}

std::vector<Waypoint> NativeADStar::createWaypoints(
		const std::vector<GridPosition> &path,
		const frc::Translation2d &realStartPos,
		const frc::Translation2d &realGoalPos,
		const std::unordered_set<GridPosition> &obstacles) {
	if (path.empty()) {
		return std::vector<Waypoint>();
	}

	std::vector < GridPosition > simplifiedPath;
	simplifiedPath.push_back(path[0]);
	for (size_t i = 1; i < path.size() - 1; i++) {
		if (!walkable(simplifiedPath[simplifiedPath.size() - 1], path[i + 1],
				obstacles)) {
			simplifiedPath.push_back(path[i]);
		}
	}
	simplifiedPath.push_back(path[path.size() - 1]);

	std::vector < frc::Translation2d > fieldPosPath;
	for (const GridPosition &pos : simplifiedPath) {
		fieldPosPath.push_back(gridPosToTranslation2d(pos));
	}

	if (fieldPosPath.size() < 2) {
		return std::vector<Waypoint>();
	}

	// Replace start and end positions with their real positions
	fieldPosPath[0] = realStartPos;
	fieldPosPath[fieldPosPath.size() - 1] = realGoalPos;

	std::vector < frc::Pose2d > pathPoses;
	pathPoses.emplace_back(fieldPosPath[0],
			(fieldPosPath[1] - fieldPosPath[0]).Angle());
	for (size_t i = 1; i < fieldPosPath.size() - 1; i++) {
		frc::Translation2d last = fieldPosPath[i - 1];
		frc::Translation2d current = fieldPosPath[i];
		frc::Translation2d next = fieldPosPath[i + 1];

		frc::Translation2d anchor1 = ((current - last) * SMOOTHING_ANCHOR_PCT)
				+ last;
		frc::Rotation2d heading1 = (current - last).Angle();
		frc::Translation2d anchor2 = ((current - next) * SMOOTHING_ANCHOR_PCT)
				+ next;
		frc::Rotation2d heading2 = (next - anchor2).Angle();

		pathPoses.emplace_back(anchor1, heading1);
		pathPoses.emplace_back(anchor2, heading2);
	}
	pathPoses.emplace_back(fieldPosPath[fieldPosPath.size() - 1],
			(fieldPosPath[fieldPosPath.size() - 1]
					- fieldPosPath[fieldPosPath.size() - 2]).Angle());

	return PathPlannerPath::waypointsFromPoses(pathPoses);
}

bool NativeADStar::walkable(const GridPosition &s1, const GridPosition &s2,
		const std::unordered_set<GridPosition> &obstacles) {
	int x0 = s1.x;
	int y0 = s1.y;
	int x1 = s2.x;
	int y1 = s2.y;

	int dx = std::abs(x1 - x0);
	int dy = std::abs(y1 - y0);
	int x = x0;
	int y = y0;
	int n = 1 + dx + dy;
	int xInc = (x1 > x0) ? 1 : -1;
	int yInc = (y1 > y0) ? 1 : -1;
	int error = dx - dy;
	dx *= 2;
	dy *= 2;

	for (; n > 0; n--) {
		if (obstacles.contains(GridPosition(x, y))) {
			return false;
		}

		if (error > 0) {
			x += xInc;
			error -= dy;
		} else if (error < 0) {
			y += yInc;
			error += dx;
		} else {
			x += xInc;
			y += yInc;
			error -= dy;
			error += dx;
			n--;
		}
	}

	return true;
}

void NativeADStar::reset(const GridPosition &sStart, const GridPosition &sGoal) {
	g.clear();
	rhs.clear();
	open.clear();
	incons.clear();
	closed.clear();

	for (int x = 0; x < nodesX; x++) {
		for (int y = 0; y < nodesY; y++) {
			g[GridPosition(x, y)] = std::numeric_limits<double>::infinity();
			rhs[GridPosition(x, y)] = std::numeric_limits<double>::infinity();
		}
	}

	rhs[sGoal] = 0.0;

	eps = EPS;

	open[sGoal] = key(sGoal, sStart);
}

void NativeADStar::computeOrImprovePath(const GridPosition &sStart,
		const GridPosition &sGoal,
		const std::unordered_set<GridPosition> &obstacles) {
	while (true) {
		auto svOpt = topKey();
		if (!svOpt.has_value()) {
			break;
		}
		auto sv = svOpt.value();
		auto s = sv.first;
		auto v = sv.second;

		if (comparePair(v, key(sStart, sStart)) >= 0
				&& rhs.at(sStart) == g.at(sStart)) {
			break;
		}

		open.erase(s);

		if (g.at(s) > rhs.at(s)) {
			g[s] = rhs.at(s);
			closed.emplace(s);

			for (const GridPosition &sn : getOpenNeighbors(s, obstacles)) {
				updateState(sn, sStart, sGoal, obstacles);
			}
		} else {
			g[s] = std::numeric_limits<double>::infinity();
			for (const GridPosition &sn : getOpenNeighbors(s, obstacles)) {
				updateState(sn, sStart, sGoal, obstacles);
			}
			updateState(s, sStart, sGoal, obstacles);
		}
	}
}

void NativeADStar::updateState(const GridPosition &s, const GridPosition &sStart,
		const GridPosition &sGoal,
		const std::unordered_set<GridPosition> &obstacles) {
	if (s != sGoal) {
		rhs[s] = std::numeric_limits<double>::infinity();

		for (const GridPosition &x : getOpenNeighbors(s, obstacles)) {
			rhs[s] = std::min(rhs.at(s), g.at(x) + cost(s, x, obstacles));
		}
	}

	open.erase(s);

	if (g.at(s) != rhs.at(s)) {
		if (!closed.contains(s)) {
			open[s] = key(s, sStart);
		} else {
			incons[s] = std::pair<double, double>(0.0, 0.0);
		}
	}
}

bool NativeADStar::isCollision(const GridPosition &sStart,
		const GridPosition &sEnd,
		const std::unordered_set<GridPosition> &obstacles) {
	if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
		return true;
	}

	if (sStart.x != sEnd.x && sStart.y != sEnd.y) {
		GridPosition s1;
		GridPosition s2;

		if (sEnd.x - sStart.x == sStart.y - sEnd.y) {
			s1 = GridPosition(std::min(sStart.x, sEnd.x),
					std::min(sStart.y, sEnd.y));
			s2 = GridPosition(std::max(sStart.x, sEnd.x),
					std::max(sStart.y, sEnd.y));
		} else {
			s1 = GridPosition(std::min(sStart.x, sEnd.x),
					std::max(sStart.y, sEnd.y));
			s2 = GridPosition(std::max(sStart.x, sEnd.x),
					std::min(sStart.y, sEnd.y));
		}

		return obstacles.contains(s1) || obstacles.contains(s2);
	}

	return false;
}

std::unordered_set<GridPosition> NativeADStar::getOpenNeighbors(
		const GridPosition &s,
		const std::unordered_set<GridPosition> &obstacles) {
	std::unordered_set < GridPosition > ret;

	for (int xMove = -1; xMove <= 1; xMove++) {
		for (int yMove = -1; yMove <= 1; yMove++) {
			GridPosition sNext = GridPosition(s.x + xMove, s.y + yMove);
			if (!obstacles.contains(sNext) && sNext.x >= 0 && sNext.x < nodesX
					&& sNext.y >= 0 && sNext.y < nodesY) {
				ret.emplace(sNext);
			}
		}
	}
	return ret;
}

std::unordered_set<GridPosition> NativeADStar::getAllNeighbors(
		const GridPosition &s) {
	std::unordered_set < GridPosition > ret;

	for (int xMove = -1; xMove <= 1; xMove++) {
		for (int yMove = -1; yMove <= 1; yMove++) {
			GridPosition sNext = GridPosition(s.x + xMove, s.y + yMove);
			if (sNext.x >= 0 && sNext.x < nodesX && sNext.y >= 0
					&& sNext.y < nodesY) {
				ret.emplace(sNext);
			}
		}
	}
	return ret;
}

std::pair<double, double> NativeADStar::key(const GridPosition &s,
		const GridPosition &sStart) {
	if (g.at(s) > rhs.at(s)) {
		return std::pair<double, double>(rhs.at(s) + eps * heuristic(sStart, s),
				rhs.at(s));
	} else {
		return std::pair<double, double>(g.at(s) + heuristic(sStart, s),
				g.at(s));
	}
}

std::optional<std::pair<GridPosition, std::pair<double, double>>> NativeADStar::topKey() {
	std::optional < std::pair<GridPosition, std::pair<double, double>> > min =
			std::nullopt;

	for (auto entry : open) {
		if (!min || comparePair(entry.second, min.value().second) < 0) {
			min = entry;
		}
	}

	return min;
}
