#pragma once
#include "point.h"
#include "utils.h"
#include <vector>
#include <memory>
#include <optional>

struct Trajectory
{
	void draw(Magnum2D::col3 color);
	void draw(double fromTime, double toTime, Magnum2D::col3 color);
	void draw(size_t fromIndex, size_t toIndex, Magnum2D::col3 color);

	size_t getClosestPointOnTrajectory(const Magnum2D::vec2& point);
	size_t getClosestPointOnTrajectoryAroundIndex(const Magnum2D::vec2& point, size_t previousIndex); // get closest point given previous index
	size_t getPoint(double time);

	void extend(Trajectory&& trajectory, size_t fromIndex = 0);
	void extend(const Trajectory& trajectory, size_t fromIndex = 0);

	void clear();

	std::vector<Magnum2D::vec2> positions;
	std::vector<Magnum2D::vec2> velocities;
	std::vector<float> times;
};

using TrajectoryPtr = std::unique_ptr<Trajectory>;
