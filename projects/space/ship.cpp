#include "ship.h"
#include "simulation.h"
#include <algorithm>

using namespace Magnum2D;

Ship::Ship(const Magnum2D::vec2d& initPos)
	: burnsHandler(this, &trajectoryVerlet), initialPosition(initPos)
{
}

template<typename T>
std::tuple<std::vector<vec2>, std::vector<float>> SimulateHelper(T&& point, const std::vector<MassPoint>& massPoints, const std::vector<BurnPtr>& burns, double dt, double seconds, int32_t numPoints)
{
	auto[points, times] = Simulation::Simulate(point, massPoints, burns, dt, seconds, numPoints);
	return { Utils::ConvertToFloat(points), Utils::ConvertToFloat(times) };
}

void Ship::Simulate(const std::vector<MassPoint>& massPoints, double dt, double seconds, int32_t numPoints)
{
	std::tie(trajectoryEuler.points, trajectoryEuler.times) = SimulateHelper(PointEuler((vec2d)initialPosition), massPoints, burns, dt, seconds, numPoints);
	std::tie(trajectoryVerlet.points, trajectoryVerlet.times) = SimulateHelper(PointVerlet((vec2d)initialPosition), massPoints, burns, dt, seconds, numPoints);
	std::tie(trajectoryRungeKuta.points, trajectoryRungeKuta.times) = SimulateHelper(PointRungeKutta(massPoints, (vec2d)initialPosition), massPoints, burns, dt, seconds, numPoints);

	burnsHandler.Refresh();
}

void Ship::AddBurn(double time, const vec2& velocity)
{
	burns.push_back(std::make_unique<Burn>(time, (vec2d)velocity));

	burnsHandler.NewBurn(burns.back().get());

	// !!! after adding burn we must simulate 
	std::sort(std::begin(burns), std::end(burns), [](const BurnPtr& a, const BurnPtr& b) { return a->time < b->time; });
}

UpdateResult Ship::Update()
{
	return burnsHandler.Update();
}

void Ship::Draw()
{
	trajectoryEuler.draw(rgb(200, 0, 0));
	trajectoryVerlet.draw(rgb(0, 0, 200));
	trajectoryRungeKuta.draw(rgb(0, 200, 0));
	burnsHandler.Draw();
}
