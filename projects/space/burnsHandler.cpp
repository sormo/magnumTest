#include "burnsHandler.h"
#include "ship.h"

using namespace Magnum2D;

BurnsHandler::BurnsHandler(Ship* t)
	: ship(t)
{
}

void BurnsHandler::Draw()
{
	vectorHandler.Draw();

	if (burnAddIndex)
		drawCircle(ship->trajectory.points[*burnAddIndex], Common::GetZoomIndependentSize(0.03f), rgb(200, 10, 10));
}

UpdateResult BurnsHandler::Update()
{
	if (vectorHandler.Update())
		return UpdateResult::Modified;

	if (vectorHandler.UpdateHighlight())
		return UpdateResult::InputGrab;

	if (isMousePressed())
	{
		if (burnAddIndex)
		{
			ship->AddBurn(ship->trajectory.times[*burnAddIndex], { 0.0f, 0.0f });
			Update(); // call recursively update to grab the input in vectorHandler, this should also return modified
			vectorHandler.UpdateHighlight();

			return UpdateResult::Modified;
		}
	}
	else
	{
		auto positionMouse = getMousePositionWorld();

		size_t closestPoint = 0;
		if (burnAddIndex)
			closestPoint = ship->trajectory.getClosestPointOnTrajectoryAroundIndex(positionMouse, *burnAddIndex);
		else
			closestPoint = ship->trajectory.getClosestPointOnTrajectory(positionMouse);

		if ((positionMouse - ship->trajectory.points[closestPoint]).length() < 0.1f)
		{
			burnAddIndex = closestPoint;
			return UpdateResult::InputGrab;
		}
		else
		{
			burnAddIndex.reset();
		}
	}

	return UpdateResult::None;
}

void BurnsHandler::NewBurn(Burn* burn)
{
	vec2 from = (vec2)ship->trajectory.points[ship->trajectory.getPoint(burn->time)];
	vec2 to = from + (vec2)burn->velocity;

	auto modifyTo = [this](const vec2& point, void* context)
	{
		Burn* burn = (Burn*)context;
		burn->velocity = (vec2d)point - (vec2d)burn->simulatedPosition;
		return point;
	};

	auto modifyFrom = [this](const vec2& point, void* context)
	{
		Burn* burn = (Burn*)context;
		auto result = ship->trajectory.getClosestPointOnTrajectory(point);

		burn->time = ship->trajectory.times[result];
		burn->simulatedPosition = (vec2d)ship->trajectory.points[result];

		return ship->trajectory.points[result];
	};

	vectorHandler.Push({ from, to, burn, modifyFrom, modifyTo });
}

void BurnsHandler::Refresh()
{
	for (auto& vec : vectorHandler)
	{
		Burn* burn = (Burn*)vec.context;

		vec.from = (vec2)ship->trajectory.points[ship->trajectory.getPoint(burn->time)];
		vec.to = vec.from + (vec2)burn->velocity;
	}
}
