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

UpdateResult Trajectory::update()
{
	return burnsHandler.Update();
}

void Trajectory::draw()
{
	drawPolyline(points, rgb(200, 0, 0));
	drawPolyline(testPoints[0], rgb(0, 0, 200));
	drawPolyline(testPoints[1], rgb(0, 200, 0));

	Utils::DrawCross((vec2)initialPosistion, 0.3f, rgb(200, 200, 200));

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
