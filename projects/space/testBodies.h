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
	float SimulatedTime = 0.0f;
	float CurrentTime = 0.0f;
	bool IsPlaying = false;
	bool IsParentSelect = false;
	bool IsCameraFollow = false;

	enum DrawFlag { DrawFlagEuler = 0x1, DrawFlagVerlet = 0x2, DrawFlagRungeKutta = 0x4, DrawFlagApproximated = 0x8, DrawFlagComputed = 0x10 };
	int32_t DrawFlags = DrawFlagEuler | DrawFlagVerlet | DrawFlagRungeKutta | DrawFlagApproximated | DrawFlagComputed;

	std::optional<size_t> CurrentBody;

	Utils::ClickHandler clickHandler;

	enum State : int32_t
	{
		View = 0,
		AddBody
	};
	State CurrentState = State::View;

	using namespace Magnum2D;

	Bodies bodies;

	void RefreshEffectiveRadius()
	{
		for (auto& body : bodies.bodies)
			body.RecomputeEffectiveRadius();
	}

	void Simulate()
	{
		bodies.SimulateClear(SimulatedTime);
	}

	void Simulate(double time)
	{
		bodies.SimulateExtend(time);
		SimulatedTime += time;
	}

	void Gui()
	{
		if (ImGui::SliderInt("Trajectory Point Count", &TrajectoryPointCount, 100, 1000))
			Simulate();

		ImGui::Text("Simulated days: %.3f [days]", TestBodies::SimulatedTime / Unit::Day);
		ImGui::SameLine();
		ImGui::Checkbox("Playing", &TestBodies::IsPlaying);

		static float simulateDays = 365.0f;
		ImGui::InputFloat("Days: ", &simulateDays, 0.5f, 15.0f);
		ImGui::SameLine();
		if (ImGui::Button("Simulate"))
			Simulate((double)simulateDays * Unit::Day);

		ImGui::CheckboxFlags("Euler", &DrawFlags, DrawFlagEuler); ImGui::SameLine();
		ImGui::CheckboxFlags("Verlet", &DrawFlags, DrawFlagVerlet); ImGui::SameLine();
		ImGui::CheckboxFlags("RungeKutta", &DrawFlags, DrawFlagRungeKutta); ImGui::SameLine();
		ImGui::CheckboxFlags("Approximated", &DrawFlags, DrawFlagApproximated); ImGui::SameLine();
		ImGui::CheckboxFlags("Computed", &DrawFlags, DrawFlagComputed);

		float currentDay = CurrentTime / Unit::Day;
		ImGui::SliderFloat("Current Time", &currentDay, 0.0f, TestBodies::SimulatedTime / Unit::Day); ImGui::SameLine(); ImGui::Text("[days]");
		TestBodies::CurrentTime = currentDay * Unit::Day;

		ImGui::RadioButton("View", (int32_t*)&CurrentState, 0); ImGui::SameLine();
		ImGui::RadioButton("Add", (int32_t*)&CurrentState, 1);

		if (CurrentBody)
		{
			Bodies::Body& body = bodies.bodies[*CurrentBody];
			
			ImGui::SeparatorText("Body");

			ImGui::Text("Name"); ImGui::SameLine(100); ImGui::Text(body.name.c_str());
			ImGui::Text("Parent"); ImGui::SameLine(100); ImGui::Text(body.parent ? bodies.bodies[*body.parent].name.c_str() : "");
			ImGui::SameLine(); ImGui::Checkbox("Select", &IsParentSelect);
			if (body.parent != body.parentSimulation)
			{
				ImGui::SameLine();
				std::string expectedParent = body.parentSimulation ? bodies.bodies[*body.parentSimulation].name : "<empty>";
				ImGui::Text("Expected"); ImGui::SameLine(100); ImGui::Text(expectedParent.c_str());
				ImGui::SameLine();
				if (ImGui::Button("Set"))
					bodies.SetParentUser(*CurrentBody, body.parentSimulation);
			}

			auto& trajectory = body.GetSimulation<PointRungeKutta>().trajectoryGlobal;
			auto position = trajectory.times.empty() ? (vec2)body.initialPosition : trajectory.positions[trajectory.getPoint(CurrentTime)];
			auto velocity = trajectory.times.empty() ? (vec2)body.initialVelocity : trajectory.velocities[trajectory.getPoint(CurrentTime)];

			if (body.parent)
			{
				auto& trajectoryParent = bodies.bodies[*body.parent].GetSimulation<PointRungeKutta>().trajectoryGlobal;
				position -= trajectoryParent.times.empty() ? (vec2)bodies.bodies[*body.parent].initialPosition : trajectoryParent.positions[trajectoryParent.getPoint(CurrentTime)];
				velocity -= trajectoryParent.times.empty() ? (vec2)bodies.bodies[*body.parent].initialVelocity : trajectoryParent.velocities[trajectoryParent.getPoint(CurrentTime)];
			}

			ImGui::Text("Position"); ImGui::SameLine(100); ImGui::Text("%.3f %.3f 10^6 [km]", (float)(position.x() / (Unit::Kilometer * 1e6)), (float)(position.y() / (Unit::Kilometer * 1e6)));
			ImGui::Text("Velocity"); ImGui::SameLine(100); ImGui::Text("%.3f %.3f [km/s]", (float)(velocity.x() * Unit::Second / Unit::Kilometer), (float)(velocity.y() * Unit::Second / Unit::Kilometer));

			float mass = (float)( (body.mass / (Unit::Kilogram * 1e24)));
			if (ImGui::InputFloat("Mass: ", &mass, 0.5f, 15.0f))
			{
				body.SetInitialState(body.initialPosition, body.initialVelocity, mass);
				Simulate();
			} 
			ImGui::SameLine(); ImGui::Text("[10^24 kg]");
			ImGui::Checkbox("Camera Follow", &IsCameraFollow);
		}
	}

	void SetupSolarSystemWip()
	{
		Unit::SetBaseMeter((1.0 / 1.5e8) * 1e-3);
		Unit::SetBaseKilogram(1.0 / 2e30);
		Unit::SetBaseSecond(1e-7 / Utils::Pi);

		GravitationalConstant = 4.0 * Utils::Pi * Utils::Pi;
		SimulationDt = Unit::Hour;

		auto createBody = [](const char* name, vec2d position, vec2d velocity, double mass)
		{
			bodies.AddBody(name, position, velocity, mass);
		};

		//auto data = Utils::ReadJsonFromResource("systems", "solar_system.json");
		auto data = Utils::ReadJsonFromResource("systems", "test_system.json");

		for (auto&[nameJson, body] : data.items())
		{
			const char* name = nameJson.c_str();
			vec2d position = { (double)body["position"]["x"] * Unit::Meter, (double)body["position"]["y"] * Unit::Meter };
			vec2d velocity = { (double)body["velocity"]["x"] * Unit::Meter / Unit::Second, (double)body["velocity"]["y"] * Unit::Meter / Unit::Second };
			double mass = (double)body["mass"] * Unit::Kilogram;

			bodies.AddBody(name, position, velocity, mass);
			bodies.bodies.back().isStar = body.contains("star");
		}
	}

	void Setup()
	{
		SetupSolarSystemWip();

		bodies.vectorHandler.thresholdDistanceZoomIndependent = 0.03f;

		//bodiesEuler.currentPoints[1].initializeCircularOrbit({ 0.0,0.0 }, massSun / massScaler);
	}

	void UdpateCurrentTime()
	{
		if (IsPlaying && CurrentTime < SimulatedTime)
			CurrentTime += (Unit::Day / getDeltaTimeMs()) * 0.1;
		if (CurrentTime > SimulatedTime)
			CurrentTime = SimulatedTime;
	}

	void Draw()
	{
		if (SimulatedTime != 0.0)
		{
			for (size_t i = 0; i < bodies.bodies.size(); i++)
				bodies.bodies[i].SetCurrentTime(CurrentTime);
		}

		bodies.Draw(DrawFlags & DrawFlagEuler, DrawFlags & DrawFlagVerlet, DrawFlags & DrawFlagRungeKutta, DrawFlags & DrawFlagApproximated, DrawFlags & DrawFlagComputed);

		for (size_t i = 0; i < bodies.bodies.size(); i++)
		{
			auto& body = bodies.bodies[i];
			col3 color = CurrentBody && *CurrentBody == i ? rgb(50, 200, 50) : rgb(50, 50, 200);
			drawCircle((vec2)bodies.GetCurrentPosition(i), Common::GetZoomIndependentSize(0.1f), color);

			if (!body.parent || bodies.GetCurrentDistanceToParent(i) > Common::GetZoomIndependentSize(0.85f))
			{
				auto offset = vec2(Common::GetZoomIndependentSize(0.1f), Common::GetZoomIndependentSize(0.1f));
				drawText((vec2)bodies.GetCurrentPosition(i) + offset, body.name, Common::GetZoomIndependentSize(0.5f), rgb(150, 150, 150));
			}
		}

		if (CurrentBody)
		{
			float effectiveRadius = bodies.bodies[*CurrentBody].GetSimulation<PointRungeKutta>().initialPoint.getEffectiveRadius();
			auto position = bodies.GetCurrentPosition(*CurrentBody);

			drawCircleOutline(position, effectiveRadius, rgb(50, 50, 50));
		}
	}

	bool Update()
	{
		UdpateCurrentTime();

		if (IsCameraFollow)
		{
			setCameraCenter(bodies.GetCurrentPosition(*CurrentBody));
		}

		if (isMousePressed())
		{
			if (CurrentState == State::AddBody)
			{
				auto position = (vec2d)getMousePositionWorld();
				auto name = Utils::GetRandomString(5);
				bodies.AddBody(name.c_str(), position, {}, 1e24 * Unit::Kilogram);

				Simulate();

				CurrentState = State::View;
			}
		}

		auto[inputGrabbed, vectorChanged] = bodies.vectorHandler.Update();

		if (vectorChanged && !CurrentBody)
		{
			CurrentBody = bodies.vectorHandlerLastBodyChange;
		}

		if (clickHandler.IsClick())
		{
			auto clickBody = bodies.SelectBody(CurrentTime, getMousePositionWorld(), Common::GetZoomIndependentSize(0.2f));
			if (CurrentBody && IsParentSelect)
			{
				bodies.SetParentUser(*CurrentBody, clickBody);
			}
			else
			{
				CurrentBody = clickBody;
			}
			IsParentSelect = false;
			IsCameraFollow = false;
		}

		clickHandler.Update();

		Draw();

		if (vectorChanged)
		{
			Simulate();
		}

		if (IsCameraFollow)
		{
			inputGrabbed = true;
		}

		return inputGrabbed;
	}
}
