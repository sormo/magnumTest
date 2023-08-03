#pragma once
#include "ship.h"
#include <vector>
#include <optional>

extern double SimulationDt;

namespace TestMassPoint
{
	double SimulationSeconds = 10.0;

	using namespace Magnum2D;

	static std::vector<ShipPtr> ships;
	static std::vector<MassPoint> massPoints;
	static std::optional<size_t> hoverTimepoint;
	static std::optional<size_t> selectTimepoint;

	Ship* currentShip = nullptr;

	static void Simulate(Ship& t)
	{
		t.Simulate(massPoints, SimulationDt, SimulationSeconds, 60);
	}

	static void Simulate()
	{
		for (auto& t : ships)
			Simulate(*t);
	}

	static void Setup()
	{
		auto cameraHsize = getCameraSize() / 2.0f;

		for (int i = 0; i < 20; i++)
		{
			MassPoint m;
			m.position = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
			massPoints.push_back(std::move(m));
		}

		for (int i = 0; i < 2; i++)
		{
			vec2d initPos = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
			ships.emplace_back(std::make_unique<Ship>(initPos));
		}

		Simulate();
	}

	static void UpdateTrajectory(Ship* ship)
	{
		auto updateResult = ship->Update();

		switch (updateResult)
		{
		case UpdateResult::Modified:
			Simulate(*ship);
		case UpdateResult::InputGrab:
			currentShip = ship;
			break;
		default:
			currentShip = nullptr;
		}
	}

	void Gui()
	{
		float seconds = TestMassPoint::SimulationSeconds;
		if (ImGui::SliderFloat("SimulationSeconds", &seconds, 10.0f, 60.0f))
		{
			TestMassPoint::SimulationSeconds = seconds;
			TestMassPoint::Simulate();
		}
	}

	static bool Update()
	{
		if (currentShip)
		{
			UpdateTrajectory(currentShip);
		}

		if (!currentShip)
		{
			for (auto& t : ships)
			{
				UpdateTrajectory(t.get());

				if (currentShip)
					break;
			}
		}

		hoverTimepoint.reset();
		for (auto& t : ships)
		{
			if (t->burnsHandler.burnAddIndex)
			{
				hoverTimepoint = t->burnsHandler.burnAddIndex;
				if (isMousePressed(Mouse::right))
					selectTimepoint = hoverTimepoint;
			}
		}

		for (const auto& m : massPoints)
		{
			drawCircle((vec2)m.position, 0.07f, rgb(66, 135, 245));
			drawCircleOutline((vec2)m.position, m.mass * GravityThreshold, rgb(50, 50, 50));
		}

		// draw timepoints
		for (auto& t : ships)
		{
			if (hoverTimepoint)
				drawCircle(t->trajectoryVerlet.points[*hoverTimepoint], Common::GetZoomIndependentSize(0.06f), rgb(50, 255, 50));
			if (selectTimepoint)
				drawCircle(t->trajectoryVerlet.points[*selectTimepoint], Common::GetZoomIndependentSize(0.06f), rgb(50, 50, 255));
		}

		// draw ships
		for (auto& t : ships)
			t->Draw();

		return currentShip != nullptr;
	}
}
