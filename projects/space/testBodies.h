#pragma once
#include <Magnum2D.h>
#include "point.h"
#include "utils.h"
#include "simulation.h"

extern double SimulationDt;

namespace TestBodies
{
	int32_t TrajectoryPointCount = 300;
	float SimulatedSeconds = 0.0f;
	float CurrentTime = 0.0f;
	bool IsPlaying = false;
	std::optional<size_t> CurrentBody;

	Utils::ClickHandler clickHandler;

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
		size_t AddBody(vec2d position, vec2d velocity, double mass = 1.0)
		{
			initialPoints.emplace_back(position, velocity);
			currentPoints.emplace_back(position, velocity);
			trajectories.push_back({});

			initialPoints.back().setMass(mass);
			currentPoints.back().setMass(mass);

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

			auto newTrajectories = Simulation::Simulate(currentPoints, burns, SimulationDt, time, 0.0, TrajectoryPointCount);
			for (size_t i = 0; i < newTrajectories.size(); i++)
			{
				trajectories[i].positions = std::move(newTrajectories[i].positions);
				trajectories[i].velocities = std::move(newTrajectories[i].velocities);
				trajectories[i].times = std::move(newTrajectories[i].times);
			}
			currentSimulatedSeconds = time;
		}

		void SimulateExtend(double time)
		{
			std::vector<std::vector<BurnPtr>> burns(currentPoints.size());
			auto newTrajectories = Simulation::Simulate(currentPoints, burns, SimulationDt, time, currentSimulatedSeconds, TrajectoryPointCount);
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
			return trajectories[index].positions[trajectories[index].getPoint(time)];
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

	void RefreshEffectiveRadius()
	{
		for (size_t i = 0; i < bodiesRungeKutta.initialPoints.size(); i++)
		{
			bodiesEuler.initialPoints[i].recomputeEffectiveRadius();
			bodiesEuler.currentPoints[i].recomputeEffectiveRadius();
			bodiesVerlet.initialPoints[i].recomputeEffectiveRadius();
			bodiesVerlet.currentPoints[i].recomputeEffectiveRadius();
			bodiesRungeKutta.initialPoints[i].recomputeEffectiveRadius();
			bodiesRungeKutta.currentPoints[i].recomputeEffectiveRadius();
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
		ImGui::Text("Simulated Seconds: %.1f [s]", TestBodies::SimulatedSeconds);
		ImGui::SameLine();
		ImGui::Checkbox("Playing", &TestBodies::IsPlaying);

		if (ImGui::SliderInt("Trajectory Point Count", &TrajectoryPointCount, 100, 1000))
			Simulate();

		static int32_t simulateTime = 10;
		ImGui::InputInt("Time: ", &simulateTime);
		ImGui::SameLine();
		if (ImGui::Button("Simulate"))
			Simulate((double)simulateTime);

		ImGui::SliderFloat("Current Time", &TestBodies::CurrentTime, 0.0f, TestBodies::SimulatedSeconds); ImGui::SameLine(); ImGui::Text("[s]");

		ImGui::RadioButton("View", (int32_t*)&CurrentState, 0); ImGui::SameLine();
		ImGui::RadioButton("Add", (int32_t*)&CurrentState, 1);

		if (CurrentBody)
		{
			auto& trajectory = bodiesRungeKutta.trajectories[*CurrentBody];
			size_t currentIndex = trajectory.getPoint(CurrentTime);
			ImGui::Text("Position"); ImGui::SameLine(100); ImGui::Text("%.3f %.3f [m]", trajectory.positions[currentIndex].x(), trajectory.positions[currentIndex].y());
			ImGui::Text("Velocity"); ImGui::SameLine(100); ImGui::Text("%.3f %.3f [m/s]", trajectory.velocities[currentIndex].x(), trajectory.velocities[currentIndex].y());

			auto& bodyCurrent = bodiesRungeKutta.currentPoints[*CurrentBody];
			auto& bodyInit = bodiesRungeKutta.initialPoints[*CurrentBody];
			float mass = (float)bodyInit.getMass();
			if (ImGui::SliderFloat("Mass", &mass, 0.1f, 10.0f))
			{
				bodyInit.setMass(mass);
				bodyCurrent.setMass(mass);
				Simulate();
			} 
			ImGui::SameLine(); ImGui::Text("[kg]");
		}
	}

	void SetupSolarSystemWip()
	{
		double massSun = 1.989e30;
		double massEarth = 5.972e24;
		double massMoon = 7.348e22;
		double gravitationalConstant = 6.6743e-11;
		double distanceSunEarth = 1.518e11;
		double distanceEarthMoon = 3.844e8;
		double velocityEarth = 2.9722e4;
		double velocityMoon = 1.020e3;

		double massScaler = 1.0e24;
		double distanceScaler = 1.0e10;

		//GravitationalConstant = gravitationalConstant; // *(massScaler / distanceScaler);
		GravitationalConstant = gravitationalConstant * massScaler / (distanceScaler * distanceScaler * distanceScaler);
		SimulationDt = 60 * 60;
		GravityThreshold = 0.001 / distanceScaler;

		auto createBody = [](double positionX, double velocityY, double mass)
		{
			auto position = vec2d(positionX, 0.0);
			auto velocity = vec2d(0.0, velocityY);

			bodiesEuler.AddBody(position, velocity, mass);
			bodiesVerlet.AddBody(position, velocity, mass);
			bodiesRungeKutta.AddBody(position, velocity, mass);
			colors.push_back(Utils::GetRandomColor());
		};

		createBody(0.0, 0.0, massSun / massScaler);
		createBody(distanceSunEarth / distanceScaler, velocityEarth / distanceScaler, massEarth / massScaler);
		createBody((distanceSunEarth + distanceEarthMoon) / distanceScaler, (velocityMoon + velocityEarth) / distanceScaler, massMoon / massScaler);

	}

	void SetupRandomBodies()
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
	}

	void Setup()
	{

		SetupRandomBodies();
		//SetupSolarSystemWip();

		//bodiesEuler.currentPoints[1].initializeCircularOrbit({ 0.0,0.0 }, massSun / massScaler);

		Simulate();

		//Simulate(SimulatedSeconds);
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

		if (CurrentBody)
		{
			float effectiveRadius = bodiesRungeKutta.initialPoints[*CurrentBody].getEffectiveRadius();
			auto& trajectory = bodiesRungeKutta.trajectories[*CurrentBody];
			size_t currentIndex = trajectory.getPoint(CurrentTime);
			auto position = trajectory.positions[currentIndex];

			drawCircleOutline(position, effectiveRadius, rgb(50, 50, 50));
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

		if (clickHandler.IsClick())
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

		clickHandler.Update();

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
