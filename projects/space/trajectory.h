#pragma once
#include "point.h"
#include "utils.h"
#include <vector>
#include <memory>
#include <optional>

struct Trajectory
{
	Trajectory(const Magnum2D::vec2d& initPos);

	void draw();
	void simulate(const std::vector<MassPoint>& massPoints, const std::vector<BurnPtr>& burns, double dt, double seconds, int32_t numPoints);

	size_t getClosestPointOnTrajectory(const Magnum2D::vec2& point);
	size_t getClosestPointOnTrajectoryAroundIndex(const Magnum2D::vec2& point, size_t previousIndex); // get closest point given previous index
	size_t getPoint(double time);

	Magnum2D::vec2d initialPosistion;

	std::vector<Magnum2D::vec2> points;
	std::vector<float> times;

	std::vector<std::vector<Magnum2D::vec2>> testPoints;
};

using TrajectoryPtr = std::unique_ptr<Trajectory>;
