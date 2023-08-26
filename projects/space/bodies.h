#pragma once
#include <Magnum2D.h>
#include "vectorHandler.h"
#include "simulation.h"
#include <set>

namespace TestBodies
{
	extern int32_t TrajectoryPointCount;
	extern float CurrentTime;
}

using namespace Magnum2D;

template <typename T>
struct Bodies
{
	const float ForceDrawFactor = 0.2f;

	size_t AddBody(const char* name, vec2d position, vec2d velocity, double mass = 1.0)
	{
		names.push_back(name);
		parents.push_back({});
		childs.push_back({});

		initialPoints.emplace_back(position, velocity);
		currentPoints.emplace_back(position, velocity);
		trajectories.push_back({});
		trajectoriesParent.push_back({});

		initialPoints.back().setMass(mass);
		currentPoints.back().setMass(mass);

		vec2 from = (vec2)position;
		vec2 to = (vec2)(position + velocity * ForceDrawFactor);
		VectorHandler::OnChange onFromChange = [this, index = initialPoints.size() - 1](const vec2& v, void*)
		{
			initialPoints[index].position = (vec2d)v;
			//SimulateClear(SimulatedSeconds);
			return v;
		};
		VectorHandler::OnChange onToChange = [this, index = initialPoints.size() - 1](const vec2& v, void*)
		{
			initialPoints[index].setVelocity((vec2d)(v / ForceDrawFactor) - initialPoints[index].position);
			//SimulateClear(SimulatedSeconds);
			return v;
		};

		vectorHandler.Push(from, to, nullptr, onFromChange, onToChange);

		return initialPoints.size() - 1;
	}

	void ProcessTrajectoriesParentRecursive(size_t body)
	{
		trajectoriesParent[body].clear();

		for (size_t i = 0; i < trajectories[body].positions.size(); i++)
		{
			trajectoriesParent[body].positions.push_back(trajectories[body].positions[i] - trajectoriesParent[*parents[body]].positions[i]);
			trajectoriesParent[body].velocities.push_back(trajectories[body].velocities[i] - trajectoriesParent[*parents[body]].velocities[i]);
		}
		trajectoriesParent[body].times = trajectories[body].times;

		for (auto child : childs[body])
			ProcessTrajectoriesParentRecursive(child);
	}

	void ProcessTrajectoriesParent()
	{
		for (size_t i = 0; i < parents.size(); i++)
		{
			if (parents[i])
				continue;
			// from the root
			trajectoriesParent[i] = trajectories[i];

			for (auto child : childs[i])
				ProcessTrajectoriesParentRecursive(child);
		}
	}

	void SimulateClear(double time)
	{
		std::vector<std::vector<BurnPtr>> burns(initialPoints.size());
		currentPoints = initialPoints;

		auto newTrajectories = Simulation::Simulate(currentPoints, burns, SimulationDt, time, 0.0, TestBodies::TrajectoryPointCount);
		for (size_t i = 0; i < newTrajectories.size(); i++)
		{
			trajectories[i].positions = std::move(newTrajectories[i].positions);
			trajectories[i].velocities = std::move(newTrajectories[i].velocities);
			trajectories[i].times = std::move(newTrajectories[i].times);
		}
		currentSimulatedSeconds = time;

		ProcessTrajectoriesParent();
	}

	void SimulateExtend(double time)
	{
		std::vector<std::vector<BurnPtr>> burns(currentPoints.size());
		auto newTrajectories = Simulation::Simulate(currentPoints, burns, SimulationDt, time, currentSimulatedSeconds, TestBodies::TrajectoryPointCount);
		for (size_t i = 0; i < newTrajectories.size(); i++)
			trajectories[i].extend(std::move(newTrajectories[i]));

		currentSimulatedSeconds += time;

		ProcessTrajectoriesParent();
	}

	void DrawBody(size_t body, const col3& color)
	{
		if (parents[body])
		{
			auto currentIndex = trajectoriesParent[*parents[body]].getPoint(TestBodies::CurrentTime);
			setTransform({ trajectoriesParent[*parents[body]].positions[currentIndex], 0.0f });
		}
		trajectoriesParent[body].draw(0.0, TestBodies::CurrentTime, color);

		setTransform({});
	}

	void Draw(const std::vector<col3>& cols)
	{
		for (size_t i = 0; i < trajectories.size(); i++)
			DrawBody(i, cols[i]);
	}

	void Draw(const col3& color)
	{
		for (size_t i = 0; i < trajectories.size(); i++)
			DrawBody(i, color);
	}

	vec2 GetPosition(size_t index, double time)
	{
		if (time == 0.0)
			return (vec2)initialPoints[index].position;

		return trajectories[index].positions[trajectories[index].getPoint(time)];
	}

	std::optional<size_t> SelectBody(double time, const vec2& selectPosition, float selectRadius)
	{
		for (size_t i = 0; i < initialPoints.size(); i++)
		{
			auto bodyPosition = GetPosition(i, time);
			if ((bodyPosition - selectPosition).length() < selectRadius)
				return i;
		}
		return {};
	}

	void SetParent(size_t parent, size_t child)
	{
		parents[child] = parent;
		childs[parent].insert(child);

		ProcessTrajectoriesParent();
	}

	void ClearParent(size_t child)
	{
		size_t parent = *parents[child];

		parents[child] = {};
		childs[parent].erase(child);
	}

	double currentSimulatedSeconds = 0.0;

	std::vector<std::string> names;
	std::vector<std::optional<size_t>> parents;
	std::vector<std::set<size_t>> childs;
	std::vector<T> initialPoints;
	std::vector<T> currentPoints;
	std::vector<Trajectory> trajectories;
	std::vector<Trajectory> trajectoriesParent;

	VectorHandler vectorHandler;
};
