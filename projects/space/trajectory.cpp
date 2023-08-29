#include "trajectory.h"
#include "utils.h"
#include "common.h"
#include "simulation.h"
#include <algorithm>
#include <cassert>

using namespace Magnum2D;

static const float GrabBurnCircleRadius = 0.3f;

std::vector<vec2> ConvertToFloat(const std::vector<vec2d>& arr)
{
	std::vector<vec2> result;
	result.reserve(arr.size());
	for (const auto& e : arr)
		result.push_back((vec2)e);
	return result;
}

std::vector<float> ConvertToFloat(const std::vector<double>& arr)
{
	std::vector<float> result;
	result.reserve(arr.size());
	for (const auto& e : arr)
		result.push_back(e);
	return result;
}

size_t Trajectory::getClosestPointOnTrajectory(const Magnum2D::vec2& point)
{
	size_t result = 0;
	float distanceSquared = std::numeric_limits<float>::max();

	for (size_t i = 0; i < positions.size(); i++)
	{
		float distSqr = Utils::DistanceSqr(positions[i], point);
		if (distSqr < distanceSquared)
		{
			distanceSquared = distSqr;
			result = i;
		}
	}

	return result;
}

// This function first decide on direction for search. It will iterate either to the left of previousIndex or to the right based on which point is closer
// to given point. When decided, it will iterate that direction until distance to the point is decreasing.
size_t Trajectory::getClosestPointOnTrajectoryAroundIndex(const vec2& point, size_t previousIndex)
{
	float prevDistSqr = Utils::DistanceSqr(point, (vec2)positions[previousIndex]);
	float leftDistSqr = std::numeric_limits<float>::max(), rightDistSqr = std::numeric_limits<float>::max();

	if (previousIndex != 0)
		leftDistSqr = Utils::DistanceSqr(point, (vec2)positions[previousIndex - 1]);
	if (previousIndex != positions.size() - 1)
		rightDistSqr = Utils::DistanceSqr(point, (vec2)positions[previousIndex + 1]);

	if (prevDistSqr < leftDistSqr && prevDistSqr < rightDistSqr)
		return previousIndex;

	auto iterateCloser = [](size_t index, float distSqr, const vec2& point, const std::vector<vec2>& positions, auto inc)
	{
		size_t indexNext = inc(index, positions.size());
		float distSqrNext = Utils::DistanceSqr(point, (vec2)positions[indexNext]);

		while (distSqrNext < distSqr)
		{
			index = indexNext;
			distSqr = distSqrNext;

			indexNext = inc(index, positions.size());
			distSqrNext = Utils::DistanceSqr(point, (vec2)positions[indexNext]);
		}

		return index;
	};

	if (leftDistSqr < rightDistSqr)
	{
		auto circDec = [](size_t index, size_t size) -> size_t
		{
			if (index == 0)
				return size - 1;
			return index - 1;
		};

		return iterateCloser(previousIndex - 1, leftDistSqr, point, positions, circDec);
	}
	else
	{
		auto circInc = [](size_t index, size_t size) -> size_t
		{
			if (index == size - 1)
				return 0;
			return index + 1;
		};

		return iterateCloser(previousIndex + 1, rightDistSqr, point, positions, circInc);
	}

	return size_t();
}

void Trajectory::draw(size_t fromIndex, size_t toIndex, Magnum2D::col3 color)
{
	Common::DrawPolyline(std::span(std::begin(positions) + fromIndex, toIndex - fromIndex + 1), Common::GetZoomIndependentSize(0.05f), color);

	Utils::DrawCross(positions[fromIndex], Common::GetZoomIndependentSize(0.3f), rgb(200, 200, 200));
}

void Trajectory::draw(double fromTime, double toTime, Magnum2D::col3 color)
{
	if (fromTime == toTime || times.empty())
		return;

	draw(getPoint(fromTime), getPoint(toTime), color);
}

void Trajectory::draw(col3 color)
{
	Common::DrawPolyline(positions, Common::GetZoomIndependentSize(0.05f), color);

	Utils::DrawCross(positions[0], Common::GetZoomIndependentSize(0.3f), rgb(200, 200, 200));
}

size_t Trajectory::getPoint(double time)
{
	if (time >= times.back())
		return times.size() - 1;

	size_t result = 0;

	while (times[result] < time && result != times.size())
		result++;

	assert(result != times.size());

	return result;
}

void Trajectory::extend(Trajectory&& trajectory, size_t fromIndex)
{
	positions.reserve(positions.size() + trajectory.positions.size());
	positions.insert(positions.end(), std::make_move_iterator(trajectory.positions.begin() + fromIndex), std::make_move_iterator(trajectory.positions.end()));

	velocities.reserve(velocities.size() + trajectory.velocities.size());
	velocities.insert(velocities.end(), std::make_move_iterator(trajectory.velocities.begin() + fromIndex), std::make_move_iterator(trajectory.velocities.end()));

	times.reserve(times.size() + trajectory.times.size());
	times.insert(times.end(), std::make_move_iterator(trajectory.times.begin() + fromIndex), std::make_move_iterator(trajectory.times.end()));
}

void Trajectory::extend(const Trajectory& trajectory, size_t fromIndex)
{
	positions.reserve(positions.size() + trajectory.positions.size());
	positions.insert(positions.end(), (trajectory.positions.begin() + fromIndex), (trajectory.positions.end()));

	velocities.reserve(velocities.size() + trajectory.velocities.size());
	velocities.insert(velocities.end(), (trajectory.velocities.begin() + fromIndex), (trajectory.velocities.end()));

	times.reserve(times.size() + trajectory.times.size());
	times.insert(times.end(), (trajectory.times.begin() + fromIndex), (trajectory.times.end()));
}

void Trajectory::clear()
{
	positions.clear();
	velocities.clear();
	times.clear();
}
