#include "ship.h"
#include <algorithm>

using namespace Magnum2D;

Ship::Ship(const Magnum2D::vec2d& initPos)
	: burnsHandler(this), trajectory(initPos)
{
}

void Ship::Simulate(const std::vector<MassPoint>& massPoints, double dt, double seconds, int32_t numPoints)
{
	trajectory.simulate(massPoints, burns, dt, seconds, numPoints);
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
	trajectory.draw();
	burnsHandler.Draw();
}
