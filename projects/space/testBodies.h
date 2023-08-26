#pragma once
#include <Magnum2D.h>
#include "point.h"
#include "utils.h"
#include "simulation.h"
#include "bodies.h"

extern double SimulationDt;
extern Camera camera;

namespace TestBodies
{
	int32_t TrajectoryPointCount = 300;
	float SimulatedSeconds = 0.0f;
	float CurrentTime = 0.0f;
	bool IsPlaying = false;
	bool IsParentSelect = false;
	bool IsCameraFollow = false;

	enum DrawFlag { DrawFlagEuler = 0x1, DrawFlagVerlet = 0x2, DrawFlagRungeKutta = 0x4 };
	int32_t DrawFlags = DrawFlagEuler | DrawFlagVerlet | DrawFlagRungeKutta;

	std::optional<size_t> CurrentBody;

	Utils::ClickHandler clickHandler;

	enum State : int32_t
	{
		View = 0,
		AddBody
	};
	State CurrentState = State::View;

	using namespace Magnum2D;

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
			bodiesEuler.initialPoints[i].setMass(bodiesRungeKutta.initialPoints[i].getMass());
			bodiesVerlet.initialPoints[i].position = bodiesRungeKutta.initialPoints[i].position;
			bodiesVerlet.initialPoints[i].setVelocity(bodiesRungeKutta.initialPoints[i].velocity);
			bodiesVerlet.initialPoints[i].setMass(bodiesRungeKutta.initialPoints[i].getMass());
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

		static float simulateTime = 1.0f;
		ImGui::InputFloat("Time: ", &simulateTime, 0.5f, 15.0f);
		ImGui::SameLine();
		if (ImGui::Button("Simulate"))
			Simulate((double)simulateTime);

		ImGui::CheckboxFlags("Euler", &DrawFlags, DrawFlagEuler); ImGui::SameLine();
		ImGui::CheckboxFlags("Verlet", &DrawFlags, DrawFlagVerlet); ImGui::SameLine();
		ImGui::CheckboxFlags("RungeKutta", &DrawFlags, DrawFlagRungeKutta);

		ImGui::SliderFloat("Current Time", &TestBodies::CurrentTime, 0.0f, TestBodies::SimulatedSeconds); ImGui::SameLine(); ImGui::Text("[s]");

		ImGui::RadioButton("View", (int32_t*)&CurrentState, 0); ImGui::SameLine();
		ImGui::RadioButton("Add", (int32_t*)&CurrentState, 1);

		if (CurrentBody)
		{
			ImGui::SeparatorText("Body");

			ImGui::Text("Name"); ImGui::SameLine(100); ImGui::Text(bodiesRungeKutta.names[*CurrentBody].c_str());
			ImGui::Text("Parent"); ImGui::SameLine(100); ImGui::Text(bodiesRungeKutta.parents[*CurrentBody] ? bodiesRungeKutta.names[*bodiesRungeKutta.parents[*CurrentBody]].c_str() : "none");
			ImGui::SameLine(100); ImGui::Checkbox("Select", &IsParentSelect);

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
				SynchronizeWithRungeKutta();
				Simulate();
			} 
			ImGui::SameLine(); ImGui::Text("[kg]");
			ImGui::Checkbox("Camera Follow", &IsCameraFollow);
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

		//double massScaler = 1.0;
		//double distanceScaler = 1.0;

		GravitationalConstant = gravitationalConstant * massScaler / (distanceScaler * distanceScaler * distanceScaler);
		SimulationDt = 60 * 60;
		GravityThreshold = 0.001 / distanceScaler;

		auto createBody = [](const char* name, double positionX, double velocityY, double mass)
		{
			auto position = vec2d(positionX, 0.0);
			auto velocity = vec2d(0.0, velocityY);

			bodiesEuler.AddBody(name, position, velocity, mass);
			bodiesVerlet.AddBody(name, position, velocity, mass);
			bodiesRungeKutta.AddBody(name, position, velocity, mass);
			colors.push_back(Utils::GetRandomColor());
		};

		createBody("Sun", 0.0, 0.0, massSun / massScaler);
		createBody("Earth", distanceSunEarth / distanceScaler, velocityEarth / distanceScaler, massEarth / massScaler);
		createBody("Moon", (distanceSunEarth + distanceEarthMoon) / distanceScaler, (velocityMoon + velocityEarth) / distanceScaler, massMoon / massScaler);

	}

	void SetupSolarSystemWip2()
	{
		double pi = 3.141592654;
		double kilometer = 1.0 / (1.5e8);
		double meter = kilometer / (1e3);
		double kilogram = 1.0 / (2e30);
		double second = 1e-7 / pi;
		double minute = 60.0 * second;
		double hour = 60.0 * minute;
		double day = 24.0 * hour;

		double massSun = 1.989e30 * kilogram;
		double massEarth = 5.972e24 * kilogram;
		double massMoon = 7.348e22 * kilogram;
		//double gravitationalConstant = 6.6743e-11;
		double distanceSunEarth = 1.518e11 * meter;
		double distanceEarthMoon = 3.844e8 * meter;
		double velocityEarth = 2.9722e4 * meter / second;
		double velocityMoon = 1.020e3 * meter / second;

		GravitationalConstant = 4.0 * pi * pi;
		SimulationDt = hour;

		auto createBody = [](const char* name, double positionX, double velocityY, double mass)
		{
			auto position = vec2d(positionX, 0.0);
			auto velocity = vec2d(0.0, velocityY);

			bodiesEuler.AddBody(name, position, velocity, mass);
			bodiesVerlet.AddBody(name, position, velocity, mass);
			bodiesRungeKutta.AddBody(name, position, velocity, mass);
			colors.push_back(Utils::GetRandomColor());
		};

		createBody("Sun", 0.0, 0.0, massSun);
		createBody("Earth", distanceSunEarth, velocityEarth, massEarth);
		createBody("Moon", (distanceSunEarth + distanceEarthMoon), (velocityMoon + velocityEarth), massMoon);

	}

	void Setup()
	{
		SetupSolarSystemWip2();

		//bodiesEuler.currentPoints[1].initializeCircularOrbit({ 0.0,0.0 }, massSun / massScaler);
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
		if (DrawFlags & DrawFlagEuler)
			bodiesEuler.Draw(rgb(50, 50, 50));
		if (DrawFlags & DrawFlagVerlet)
			bodiesVerlet.Draw(rgb(100, 100, 100));
		if (DrawFlags & DrawFlagRungeKutta)
			bodiesRungeKutta.Draw(colors);
		bodiesRungeKutta.vectorHandler.Draw();

		for (size_t i = 0; i < bodiesRungeKutta.initialPoints.size(); i++)
		{
			col3 color = CurrentBody && *CurrentBody == i ? rgb(50, 200, 50) : rgb(50, 50, 200);
			drawCircle((vec2)bodiesRungeKutta.GetPosition(i, CurrentTime), Common::GetZoomIndependentSize(0.1f), color);
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
				auto name = Utils::GetRandomString(5);
				bodiesEuler.AddBody(name.c_str(), position, {});
				bodiesVerlet.AddBody(name.c_str(), position, {});
				bodiesRungeKutta.AddBody(name.c_str(), position, {});
				colors.push_back(Utils::GetRandomColor());

				Simulate();

				CurrentState = State::View;
			}
		}

		if (clickHandler.IsClick())
		{
			auto previousCurrentBody = CurrentBody;
			auto clickBody = bodiesRungeKutta.SelectBody(CurrentTime, getMousePositionWorld(), Common::GetZoomIndependentSize(0.2f));

			if (CurrentBody && IsParentSelect)
			{
				if (clickBody)
					bodiesRungeKutta.SetParent(*clickBody, *CurrentBody);
				else
					bodiesRungeKutta.ClearParent(*CurrentBody);
			}
			else
			{
				CurrentBody = clickBody;
			}
			IsParentSelect = false;
			IsCameraFollow = false;
		}

		clickHandler.Update();

		bool result = bodiesRungeKutta.vectorHandler.Update();

		Draw();

		if (result)
		{
			SynchronizeWithRungeKutta();
			Simulate();
		}

		if (IsCameraFollow)
		{
			setCameraCenter(bodiesRungeKutta.GetPosition(*CurrentBody, CurrentTime));
		}

		return result;
	}
}
