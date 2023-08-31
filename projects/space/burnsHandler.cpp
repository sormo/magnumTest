#include "burnsHandler.h"
#include "ship.h"

using namespace Magnum2D;

BurnsHandler::BurnsHandler(Ship* s, Trajectory* t)
	: ship(s), trajectory(t)
{
}

void BurnsHandler::Draw()
{
	vectorHandler.Draw();

	if (burnAddIndex)
		drawCircle(trajectory->positions[*burnAddIndex], Common::GetZoomIndependentSize(0.03f), rgb(200, 10, 10));
}

UpdateResult BurnsHandler::Update()
{
	auto [inputGrabbed, vectorChanged] = vectorHandler.Update();

	if (vectorChanged)
		return UpdateResult::Modified;

	if (inputGrabbed || vectorHandler.UpdateHighlight())
		return UpdateResult::InputGrab;

	if (isMousePressed())
	{
		if (burnAddIndex)
		{
			ship->AddBurn(trajectory->times[*burnAddIndex], { 0.0f, 0.0f });
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
			closestPoint = trajectory->getClosestPointOnTrajectoryAroundIndex(positionMouse, *burnAddIndex);
		else
			closestPoint = trajectory->getClosestPointOnTrajectory(positionMouse);

		if ((positionMouse - trajectory->positions[closestPoint]).length() < 0.1f)
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
	vec2 from = (vec2)trajectory->positions[trajectory->getPoint(burn->time)];
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
		burn->simulatedPosition = (vec2d)trajectory->positions[result];

		return trajectory->positions[result];
	};

	vectorHandler.Push(from, to, burn, modifyFrom, modifyTo);
}

void BurnsHandler::Refresh()
{
	// TODO rework with Vector ids added later
	for (auto& [vector, data] : vectorHandler)
	{
		Burn* burn = (Burn*)data.context;

		data.from = (vec2)trajectory->positions[trajectory->getPoint(burn->time)];
		data.to = data.from + (vec2)burn->velocity;
	}
}
