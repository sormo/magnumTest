#pragma once
#include "point.h"
#include "common.h"
#include "trajectory.h"
#include "burnsHandler.h"
#include <memory>

struct Ship
{
	Ship(const Magnum2D::vec2d& initPos);
	void AddBurn(double time, const Magnum2D::vec2& velocity);
	void Draw();
	UpdateResult Update();
	void Simulate(const std::vector<MassPoint>& massPoints, double dt, double seconds, int32_t numPoints);

	Magnum2D::vec2 initialPosition;

	Trajectory trajectoryEuler;
	Trajectory trajectoryVerlet;
	Trajectory trajectoryRungeKuta;
	std::vector<BurnPtr> burns;
	BurnsHandler burnsHandler;
};
using ShipPtr = std::unique_ptr<Ship>;
