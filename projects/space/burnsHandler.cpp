#include "burnsHandler.h"
#include "trajectory.h"

using namespace Magnum2D;

BurnsHandler::BurnsHandler(Trajectory* t)
	: trajectory(t)
{
}

void BurnsHandler::Draw()
{
	vectorHandler.Draw();

	if (burnAddIndex)
		drawCircle(trajectory->points[*burnAddIndex], 0.03f, rgb(200, 10, 10));
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
			trajectory->addBurn(trajectory->times[*burnAddIndex], { 0.0f,0.0f });
			Update(); // call recursively update to grab the input in vectorHandler, this should also return modified

			return UpdateResult::Modified;
		}
	}
	else
	{
		auto positionMouse = getMousePositionWorld();
		auto closestPoint = trajectory->getClosestPointOnTrajectory(positionMouse);

		if ((positionMouse - trajectory->points[closestPoint]).length() < 0.1f)
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
	vec2 from = (vec2)trajectory->points[trajectory->getPoint(burn->time)];
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
		auto result = trajectory->getClosestPointOnTrajectory(point);

		burn->time = trajectory->times[result];
		burn->simulatedPosition = (vec2d)trajectory->points[result];

		return trajectory->points[result];
	};

	vectorHandler.Push({ from, to, burn, modifyFrom, modifyTo });
}

void BurnsHandler::Refresh()
{
	for (auto& vec : vectorHandler)
	{
		Burn* burn = (Burn*)vec.context;

		vec.from = (vec2)trajectory->points[trajectory->getPoint(burn->time)];
		vec.to = vec.from + (vec2)burn->velocity;
	}
}
