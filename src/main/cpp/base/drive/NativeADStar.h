// Adapted from PathPlanner v2026.1.2 (MIT), copyright Michael Jansen.
// See NativeADStar_LICENSE.txt. Refreshes identical/sub-cell requests and owns its worker.
#pragma once
#include <pathplanner/lib/pathfinding/LocalADStar.h>
#include <optional>
#include <string>
namespace base::detail {
using pathplanner::GridPosition;
using pathplanner::PathConstraints;
using pathplanner::GoalEndState;
using pathplanner::PathPlannerPath;
using pathplanner::Pathfinder;
using pathplanner::Waypoint;
class NativeADStar: public Pathfinder {
public:
	explicit NativeADStar(std::optional<std::string> navgridPath = std::nullopt);

	~NativeADStar() {
        planningThread.request_stop();
        if (planningThread.joinable()) planningThread.join();
    }

	inline bool isNewPathAvailable() override {
		return newPathAvailable;
	}

	std::shared_ptr<PathPlannerPath> getCurrentPath(PathConstraints constraints,
			GoalEndState goalEndState) override;

    // Check the swept center-line against the configured inflated navgrid and
    // current dynamic obstacles. Missing or invalid grids are not clear.
    bool DirectSegmentClear(const frc::Translation2d& start, const frc::Translation2d& goal);

	void setStartPosition(const frc::Translation2d &start) override;

	void setGoalPosition(const frc::Translation2d &goal) override;

	void setDynamicObstacles(
			const std::vector<std::pair<frc::Translation2d, frc::Translation2d>> &obs,
			const frc::Translation2d &currentRobotPos) override;

	inline GridPosition getGridPos(const frc::Translation2d &pos) {
		return GridPosition(static_cast<int>(std::floor(pos.X()() / nodeSize)),
				static_cast<int>(std::floor(pos.Y()() / nodeSize)));
	}

private:
	const double SMOOTHING_ANCHOR_PCT = 0.8;
	const double EPS = 2.5;

	double fieldLength;
	double fieldWidth;

	double nodeSize;

	int nodesX;
	int nodesY;
    bool navgridLoaded = false;

	std::unordered_map<GridPosition, double> g;
	std::unordered_map<GridPosition, double> rhs;
	std::unordered_map<GridPosition, std::pair<double, double>> open;
	std::unordered_map<GridPosition, std::pair<double, double>> incons;
	std::unordered_set<GridPosition> closed;
	std::unordered_set<GridPosition> staticObstacles;
	std::unordered_set<GridPosition> dynamicObstacles;
	std::unordered_set<GridPosition> requestObstacles;

	GridPosition requestStart;
	frc::Translation2d requestRealStartPos;
	GridPosition requestGoal;
	frc::Translation2d requestRealGoalPos;

	double eps;

	std::jthread planningThread;
	wpi::mutex pathMutex;
	wpi::mutex requestMutex;

	bool requestMinor;
	bool requestMajor;
	bool requestReset;
	std::atomic_bool newPathAvailable;

	std::vector<Waypoint> currentWaypoints;
	std::vector<GridPosition> currentPathFull;

	void runThread(std::stop_token stop);

	void doWork(const bool needsReset, const bool doMinor, const bool doMajor,
			const GridPosition &sStart, const GridPosition &sGoal,
			const frc::Translation2d &realStartPos,
			const frc::Translation2d &realGoalPos,
			const std::unordered_set<GridPosition> &obstacles);

	GridPosition findClosestNonObstacle(const GridPosition &pos,
			const std::unordered_set<GridPosition> &obstacles);

	std::vector<GridPosition> extractPath(const GridPosition &sStart,
			const GridPosition &sGoal,
			const std::unordered_set<GridPosition> &obstacles);

	std::vector<Waypoint> createWaypoints(const std::vector<GridPosition> &path,
			const frc::Translation2d &realStartPos,
			const frc::Translation2d &realGoalPos,
			const std::unordered_set<GridPosition> &obstacles);

	bool walkable(const GridPosition &s1, const GridPosition &s2,
			const std::unordered_set<GridPosition> &obstacles);

	void reset(const GridPosition &sStart, const GridPosition &sGoal);

	void computeOrImprovePath(const GridPosition &sStart,
			const GridPosition &sGoal,
			const std::unordered_set<GridPosition> &obstacles);

	void updateState(const GridPosition &s, const GridPosition &sStart,
			const GridPosition &sGoal,
			const std::unordered_set<GridPosition> &obstacles);

	inline double cost(const GridPosition &sStart, const GridPosition &sGoal,
			const std::unordered_set<GridPosition> &obstacles) {
		if (isCollision(sStart, sGoal, obstacles)) {
			return std::numeric_limits<double>::infinity();
		}
		return heuristic(sStart, sGoal);
	}

	bool isCollision(const GridPosition &sStart, const GridPosition &sEnd,
			const std::unordered_set<GridPosition> &obstacles);

	std::unordered_set<GridPosition> getOpenNeighbors(const GridPosition &s,
			const std::unordered_set<GridPosition> &obstacles);

	std::unordered_set<GridPosition> getAllNeighbors(const GridPosition &s);

	std::pair<double, double> key(const GridPosition &s,
			const GridPosition &sStart);

	std::optional<std::pair<GridPosition, std::pair<double, double>>> topKey();

	inline double heuristic(const GridPosition &sStart,
			const GridPosition &sGoal) {
		return std::hypot(sGoal.x - sStart.x, sGoal.y - sStart.y);
	}

	constexpr int comparePair(const std::pair<double, double> &a,
			const std::pair<double, double> &b) {
		int first = compareDouble(a.first, b.first);
		if (first == 0) {
			return compareDouble(a.second, b.second);
		} else {
			return first;
		}
	}

	constexpr int compareDouble(const double a, const double b) {
		if (a < b) {
			return -1;
		} else if (a > b) {
			return 1;
		}
		return 0;
	}

	inline frc::Translation2d gridPosToTranslation2d(const GridPosition &pos) {
		return frc::Translation2d(
				units::meter_t { (pos.x * nodeSize) + (nodeSize / 2.0) },
				units::meter_t { (pos.y * nodeSize) + (nodeSize / 2.0) });
	}
};

}
