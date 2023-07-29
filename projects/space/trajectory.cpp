#include "trajectory.h"
#include "utils.h"
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

Trajectory::Trajectory(const Magnum2D::vec2d& initPos)
	: burnsHandler(this), initialPosistion(initPos)
{
}

void Trajectory::simulate(const std::vector<PointMass>& massPoints, double dt, double seconds, int32_t numPoints)
{
	PointVerlet pointVerlet(initialPosistion);

	auto [pointsd, timesd ] = pointVerlet.simulate(massPoints, burns, dt, seconds, numPoints);

	points = ConvertToFloat(pointsd);
	times = ConvertToFloat(timesd);

	testPoints.clear();
	PointEuler pointEuler(initialPosistion);
	testPoints.push_back(ConvertToFloat(std::get<0>(pointEuler.simulate(massPoints, burns, dt, seconds, numPoints))));
	PointRungeKutta pointRungeKutta(massPoints, initialPosistion);
	testPoints.push_back(ConvertToFloat(std::get<0>(pointRungeKutta.simulate(massPoints, burns, dt, seconds, numPoints))));

	burnsHandler.Refresh();
}

size_t Trajectory::getClosestPointOnTrajectory(const Magnum2D::vec2& point)
{
	size_t result = 0;
	float distanceSquared = std::numeric_limits<float>::max();

	for (size_t i = 0; i < points.size(); i++)
	{
		float distSqr = Utils::DistanceSqr(points[i], point);
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
	float prevDistSqr = Utils::DistanceSqr(point, (vec2)points[previousIndex]);
	float leftDistSqr = std::numeric_limits<float>::max(), rightDistSqr = std::numeric_limits<float>::max();

	if (previousIndex != 0)
		leftDistSqr = Utils::DistanceSqr(point, (vec2)points[previousIndex - 1]);
	if (previousIndex != points.size() - 1)
		rightDistSqr = Utils::DistanceSqr(point, (vec2)points[previousIndex + 1]);

	if (prevDistSqr < leftDistSqr && prevDistSqr < rightDistSqr)
		return previousIndex;

	auto iterateCloser = [](size_t index, float distSqr, const vec2& point, const std::vector<vec2>& points, auto inc)
	{
		size_t indexNext = inc(index, points.size());
		float distSqrNext = Utils::DistanceSqr(point, (vec2)points[indexNext]);

		while (distSqrNext < distSqr)
		{
			index = indexNext;
			distSqr = distSqrNext;

			indexNext = inc(index, points.size());
			distSqrNext = Utils::DistanceSqr(point, (vec2)points[indexNext]);
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

		return iterateCloser(previousIndex - 1, leftDistSqr, point, points, circDec);
	}
	else
	{
		auto circInc = [](size_t index, size_t size) -> size_t
		{
			if (index == size - 1)
				return 0;
			return index + 1;
		};

		return iterateCloser(previousIndex + 1, rightDistSqr, point, points, circInc);
	}

	return size_t();
}

UpdateResult Trajectory::update()
{
	return burnsHandler.Update();
}

void Trajectory::draw()
{
	drawPolyline(points, rgb(200, 0, 0));
	drawPolyline(testPoints[0], rgb(0, 0, 200));
	drawPolyline(testPoints[1], rgb(0, 200, 0));

	Utils::DrawCross((vec2)initialPosistion, Common::GetZoomIndependentSize(0.3f), rgb(200, 200, 200));

	burnsHandler.Draw();
}

size_t Trajectory::getPoint(double time)
{
	size_t result = 0;

	while (times[result] < time && result != times.size())
		result++;

	assert(result != times.size());

	return result;
}

void Trajectory::addBurn(double time, const vec2& velocity)
{
	burns.push_back(std::make_unique<Burn>(time, (vec2d)velocity));

	burnsHandler.NewBurn(burns.back().get());

	// !!! after adding burn we must simulate 
	std::sort(std::begin(burns), std::end(burns), [](const BurnPtr& a, const BurnPtr& b) { return a->time < b->time; });
	points.clear();
	times.clear();
}
