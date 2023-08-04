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
	std::optional<size_t> CurrentBody;

	// how much distance was mouse moved while pressed
	float accumulatedMouseDelta = 0.0f;

	enum State : int32_t
	{
		View = 0,
		AddBody
	};
	State CurrentState = State::View;

	using namespace Magnum2D;

	template <typename T>
	struct Bodies
	{
		size_t AddBody(vec2d position, vec2d velocity)
		{
			initialPoints.emplace_back(position, velocity);
			currentPoints.emplace_back(position, velocity);
			trajectories.push_back({});

			VectorHandler::Vector vec;
			vec.from = (vec2)position;
			vec.to = (vec2)(position + velocity);
			vec.onFromChange = [this, index = initialPoints.size() - 1](const vec2& v, void*)
			{
				initialPoints[index].position = (vec2d)v;
				//SimulateClear(SimulatedSeconds);
				return v;
			};
			vec.onToChange = [this, index = initialPoints.size() - 1](const vec2& v, void*)
			{
				initialPoints[index].setVelocity((vec2d)v - initialPoints[index].position);
				//SimulateClear(SimulatedSeconds);
				return v;
			};

			vectorHandler.Push(std::move(vec));

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

		vec2 GetPosition(size_t index, double time)
		{
			return trajectories[index].points[trajectories[index].getPoint(time)];
		}

		double currentSimulatedSeconds = 0.0;

		std::vector<T> initialPoints;
		std::vector<T> currentPoints;
		std::vector<Trajectory> trajectories;

		VectorHandler vectorHandler;
	};

	Bodies<PointEuler> bodiesEuler;
	Bodies<PointVerlet> bodiesVerlet;
	Bodies<PointRungeKutta> bodiesRungeKutta;

	std::vector<col3> colors;

	void SynchronizeWithRungeKutta()
	{
		// something has changed, copy runge-kutta to others
		for (size_t i = 0; i < bodiesRungeKutta.initialPoints.size(); i++)
		{
			bodiesEuler.initialPoints[i].position = bodiesRungeKutta.initialPoints[i].position;
			bodiesEuler.initialPoints[i].setVelocity(bodiesRungeKutta.initialPoints[i].velocity);
			bodiesVerlet.initialPoints[i].position = bodiesRungeKutta.initialPoints[i].position;
			bodiesVerlet.initialPoints[i].setVelocity(bodiesRungeKutta.initialPoints[i].velocity);
		}
	}

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

		ImGui::RadioButton("View", (int32_t*)&CurrentState, 0); ImGui::SameLine();
		ImGui::RadioButton("Add", (int32_t*)&CurrentState, 1);

		if (CurrentBody)
		{
			float mass = bodiesRungeKutta.initialPoints[*CurrentBody].mass;
			if (ImGui::SliderFloat("Mass", &mass, 0.1f, 10.0f))
			{
				bodiesRungeKutta.initialPoints[*CurrentBody].mass = mass;
				Simulate();
			}
		}
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

	void UdpateCurrentTime()
	{
		if (IsPlaying && CurrentTime < SimulatedSeconds)
			CurrentTime += getDeltaTimeMs() / 1000.0;
		if (CurrentTime > SimulatedSeconds)
			CurrentTime = SimulatedSeconds;
	}

	void Draw()
	{
		bodiesEuler.Draw(rgb(50, 50, 50));
		bodiesVerlet.Draw(rgb(100, 100, 100));
		bodiesRungeKutta.Draw(colors);
		bodiesRungeKutta.vectorHandler.Draw();

		for (size_t i = 0; i < bodiesRungeKutta.initialPoints.size(); i++)
		{
			col3 color = CurrentBody && *CurrentBody == i ? rgb(50, 200, 50) : rgb(50, 50, 200);
			drawCircle((vec2)bodiesRungeKutta.GetPosition(i, CurrentTime), 0.1f, color);
		}
	}

	bool Update()
	{
		UdpateCurrentTime();

		if (isMousePressed())
		{
			if (CurrentState == State::AddBody)
			{
				auto position = (vec2d)getMousePositionWorld();
				bodiesEuler.AddBody(position, {});
				bodiesVerlet.AddBody(position, {});
				bodiesRungeKutta.AddBody(position, {});
				colors.push_back(Utils::GetRandomColor());

				Simulate();

				CurrentState = State::View;
			}
		}

		if (isMouseReleased())
		{
			if (accumulatedMouseDelta < 0.1f)
			{
				CurrentBody = {};
				auto mousePosition = getMousePositionWorld();
				for (size_t i = 0; i < bodiesRungeKutta.initialPoints.size(); i++)
				{
					vec2 position = (vec2)bodiesRungeKutta.GetPosition(i, CurrentTime);
					if ((position - mousePosition).length() < 0.2f)
					{
						CurrentBody = i;
						break;
					}
				}
			}
			accumulatedMouseDelta = 0.0f;
		}

		if (isMouseDown())
		{
			accumulatedMouseDelta += convertWindowToWorldVector(getMouseDeltaWindow()).length();
		}

		bool result = bodiesRungeKutta.vectorHandler.Update();

		Draw();

		if (result)
		{
			SynchronizeWithRungeKutta();
			Simulate();
		}

		return result;
	}
}
