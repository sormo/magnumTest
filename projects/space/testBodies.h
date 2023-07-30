#pragma once
#include <Magnum2D.h>
#include "point.h"
#include "utils.h"
#include "simulation.h"

extern double SimulationDt;
extern double SimulationSeconds;

namespace TestBodies
{
	using namespace Magnum2D;

	std::vector<PointEuler> pointsEuler;
	std::vector<Trajectory> trajectoriesEuler;
	std::vector<PointVerlet> pointsVerlet;
	std::vector<Trajectory> trajectoriesVerlet;
	std::vector<col3> colors;

	void Simulate()
	{
		std::vector<std::vector<BurnPtr>> burns(pointsEuler.size());

		auto localEuler = pointsEuler;
		trajectoriesEuler = Simulation::Simulate(localEuler, burns, SimulationDt, SimulationSeconds, 60);

		auto localVerlet = pointsVerlet;
		trajectoriesVerlet = Simulation::Simulate(localVerlet, burns, SimulationDt, SimulationSeconds, 60);
	}

	void Setup()
	{
		auto cameraHsize = getCameraSize() / 2.0f;

		for (int i = 0; i < 5; i++)
		{
			static const float IntVelRad = 0.2f;

			auto position = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
			auto velocity = (vec2d)Utils::GetRandomPosition(-IntVelRad, IntVelRad, -IntVelRad, IntVelRad);

			pointsEuler.emplace_back(position, velocity);
			pointsVerlet.emplace_back(position, velocity);
			colors.push_back(Utils::GetRandomColor());
		}

		Simulate();
	}

	bool Update()
	{
		for (size_t i = 0; i < trajectoriesEuler.size(); i++)
			trajectoriesEuler[i].draw(rgb(50, 50, 50));

		for (size_t i = 0; i < trajectoriesVerlet.size(); i++)
			trajectoriesVerlet[i].draw(colors[i]);

		return false;
	}
}
