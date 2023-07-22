#pragma once
#include "point.h"
#include <vector>
#include <memory>
#include <optional>

struct Trajectory
{
	void simulate(const std::vector<PointMass>& massPoints, float dt, float seconds, int32_t numPoints);
	void draw();
	void addBurn(float time, const Magnum2D::vec2& velocity);
	bool handleBurns();

	std::vector<Magnum2D::vec2> points;
	std::vector<float> times;

	std::vector<Burn> burns;
	std::vector<Magnum2D::vec2> burnPositions;
	std::optional<size_t> grabbedBurn;
	Magnum2D::vec2 grabOffset;

	std::vector<std::vector<Magnum2D::vec2>> testPoints;
};
