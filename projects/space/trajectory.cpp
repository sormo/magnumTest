#include "trajectory.h"
#include "utils.h"
#include <algorithm>

using namespace Magnum2D;

static const float GrabBurnCircleRadius = 0.3f;

void Trajectory::simulate(const std::vector<PointMass>& massPoints, float dt, float seconds, int32_t numPoints)
{
	PointVerlet pointVerlet;

	std::tie(points, times, burnPositions) = pointVerlet.simulate(massPoints, burns, dt, seconds, numPoints);

	testPoints.clear();
	PointEuler pointEuler;
	testPoints.push_back(std::get<0>(pointEuler.simulate(massPoints, burns, dt, seconds, numPoints)));
	PointRungeKutta pointRungeKutta(massPoints);
	testPoints.push_back(std::get<0>(pointRungeKutta.simulate(massPoints, burns, dt, seconds, numPoints)));
}

void Trajectory::draw()
{
	drawPolyline(points, rgb(200, 0, 0));
	drawPolyline(testPoints[0], rgb(0, 0, 200));
	drawPolyline(testPoints[1], rgb(0, 200, 0));

	for (size_t i = 0; i < burns.size(); i++)
	{
		Utils::DrawVector(burnPositions[i], burns[i].velocity, rgb(255, 255, 255));
		drawCircleOutline(burnPositions[i] + burns[i].velocity, GrabBurnCircleRadius, rgb(50, 50, 50));
	}
}

void Trajectory::addBurn(float time, const vec2& velocity)
{
	burns.push_back({ time, velocity });
	std::sort(std::begin(burns), std::end(burns), [](const Burn& a, const Burn& b) { return a.time < b.time; });
}

bool Trajectory::handleBurns()
{
	if (Magnum2D::isMousePressed())
	{
		auto position = Magnum2D::getMousePositionWorld();

		for (size_t i = 0; i < burns.size(); i++)
		{
			vec2 offset = position - (burnPositions[i] + burns[i].velocity);

			if (offset.length() < GrabBurnCircleRadius)
			{
				grabbedBurn = i;
				grabOffset = offset;

				return true;
			}
		}
	}
	else if (Magnum2D::isMouseReleased() && grabbedBurn)
	{
		grabbedBurn.reset();

		return true;
	}
	else if (grabbedBurn)
	{
		auto position = Magnum2D::getMousePositionWorld();

		burns[*grabbedBurn].velocity = position - grabOffset - burnPositions[*grabbedBurn];

		return true;
	}

	return false;
}
