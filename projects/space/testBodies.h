#pragma once
#include <Magnum2D.h>
#include "point.h"
#include "utils.h"
#include "simulation.h"

extern double SimulationDt;

namespace TestBodies
{
	float SimulatedSeconds = 0.0f;
	float CurrentTime = 0.0f;
	bool IsPlaying = false;

	using namespace Magnum2D;

	template <typename T>
	struct Bodies
	{
		size_t AddBody(vec2d position, vec2d velocity)
		{
			initialPoints.emplace_back(position, velocity);
			currentPoints.emplace_back(position, velocity);
			trajectories.push_back({});

			return initialPoints.size() - 1;
		}

		void SimulateClear(double time)
		{
			std::vector<std::vector<BurnPtr>> burns(initialPoints.size());
			currentPoints = initialPoints;

			auto newTrajectories = Simulation::Simulate(currentPoints, burns, SimulationDt, time);
			for (size_t i = 0; i < newTrajectories.size(); i++)
			{
				trajectories[i].points = std::move(newTrajectories[i].points);
				trajectories[i].times = std::move(newTrajectories[i].times);
			}
			currentSimulatedSeconds = time;
		}

		void SimulateExtend(double time)
		{
			std::vector<std::vector<BurnPtr>> burns(currentPoints.size());
			auto newTrajectories = Simulation::Simulate(currentPoints, burns, SimulationDt, time, currentSimulatedSeconds, 60);
			for (size_t i = 0; i < newTrajectories.size(); i++)
				trajectories[i].extend(std::move(newTrajectories[i]));

			currentSimulatedSeconds += time;
		}

		void Draw(const std::vector<col3>& cols)
		{
			for (size_t i = 0; i < trajectories.size(); i++)
				trajectories[i].draw(0.0, CurrentTime, cols[i]);
		}

		void Draw(const col3& color)
		{
			for (size_t i = 0; i < trajectories.size(); i++)
				trajectories[i].draw(0.0, CurrentTime, color);
		}

		double currentSimulatedSeconds = 0.0;

		std::vector<T> initialPoints;
		std::vector<T> currentPoints;
		std::vector<Trajectory> trajectories;
	};

	Bodies<PointEuler> bodiesEuler;
	Bodies<PointVerlet> bodiesVerlet;
	Bodies<PointRungeKutta> bodiesRungeKutta;

	std::vector<col3> colors;

	void Simulate()
	{
		bodiesEuler.SimulateClear(SimulatedSeconds);
		bodiesVerlet.SimulateClear(SimulatedSeconds);
		bodiesRungeKutta.SimulateClear(SimulatedSeconds);
	}

	void Simulate(double seconds)
	{
		bodiesEuler.SimulateExtend(seconds);
		bodiesVerlet.SimulateExtend(seconds);
		bodiesRungeKutta.SimulateExtend(seconds);
		SimulatedSeconds += seconds;
	}

	void Gui()
	{
		ImGui::Text("Simulated Seconds: %.1f", TestBodies::SimulatedSeconds);
		ImGui::SameLine();
		if (ImGui::Button("Simulate"))
			TestBodies::Simulate(10.0);
		ImGui::SameLine();
		ImGui::Checkbox("Playing", &TestBodies::IsPlaying);

		ImGui::SliderFloat("Current Time", &TestBodies::CurrentTime, 0.0f, TestBodies::SimulatedSeconds);
	}

	void Setup()
	{
		auto cameraHsize = getCameraSize() / 2.0f;

		for (int i = 0; i < 5; i++)
		{
			static const float IntVelRad = 0.5f;

			auto position = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
			auto velocity = (vec2d)Utils::GetRandomPosition(-IntVelRad, IntVelRad, -IntVelRad, IntVelRad);

			bodiesEuler.AddBody(position, velocity);
			bodiesVerlet.AddBody(position, velocity);
			bodiesRungeKutta.AddBody(position, velocity);
			colors.push_back(Utils::GetRandomColor());
		}

		Simulate(SimulatedSeconds);
	}

	bool Update()
	{
		if (IsPlaying && CurrentTime < SimulatedSeconds)
			CurrentTime += getDeltaTimeMs() / 1000.0;
		if (CurrentTime > SimulatedSeconds)
			CurrentTime = SimulatedSeconds;

		bodiesEuler.Draw(rgb(50, 50, 50));
		bodiesVerlet.Draw(rgb(100, 100, 100));
		bodiesRungeKutta.Draw(colors);

		return false;
	}
}
