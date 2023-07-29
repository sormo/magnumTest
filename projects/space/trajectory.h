#pragma once
#include "point.h"
#include <vector>
#include <memory>
#include <optional>
#include "utils.h"
#include "common.h"
#include "burnsHandler.h"

struct Trajectory
{
	Trajectory();

	UpdateResult update();
	void draw();

	void simulate(const std::vector<PointMass>& massPoints, double dt, double seconds, int32_t numPoints);
	void addBurn(double time, const Magnum2D::vec2& velocity);

	size_t getClosestPointOnTrajectory(const Magnum2D::vec2& point);
	size_t getPoint(double time);

	std::vector<Magnum2D::vec2> points;
	std::vector<float> times;

	std::vector<std::vector<Magnum2D::vec2>> testPoints;

	std::vector<BurnPtr> burns;

	BurnsHandler burnsHandler;
};
