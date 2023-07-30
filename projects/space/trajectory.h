#pragma once
#include "point.h"
#include "utils.h"
#include <vector>
#include <memory>
#include <optional>

struct Trajectory
{
	void draw(Magnum2D::col3 color);

	size_t getClosestPointOnTrajectory(const Magnum2D::vec2& point);
	size_t getClosestPointOnTrajectoryAroundIndex(const Magnum2D::vec2& point, size_t previousIndex); // get closest point given previous index
	size_t getPoint(double time);

	std::vector<Magnum2D::vec2> points;
	std::vector<float> times;
};

using TrajectoryPtr = std::unique_ptr<Trajectory>;
