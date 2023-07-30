#include "point.h"

using namespace Magnum2D;

double GravitationalConstant = 0.8;
double GravityThreshold = 10.0;

Point::Point(const Magnum2D::vec2d& pos)
{
	position = pos;
}

vec2d Point::computeAcceleration(const std::vector<MassPoint>& massPoints)
{
	vec2d result;

	for (const auto& p : massPoints)
	{
		if ((p.position - position).length() > mass * GravityThreshold)
			continue;

		result += attractForce(p.position, p.mass) / mass;
	}

	return result;
}

void Point::applyForce(const vec2d& force)
{
	acceleration += (force / mass);
}

vec2d Point::attractForce(vec2d point, double pointMass) const
{
	vec2d dir = (point - position);
	double distance = dir.length();
	dir = dir.normalized();

	return (GravitationalConstant * pointMass * mass / (distance * distance)) * dir;
}

void Point::initializeCircularOrbit(vec2d point, double pointMass)
{
	vec2d vec = point - position;
	vec2d perpendicularDir = vec2d(-vec.y(), vec.x()).normalized();

	double velocitySize = std::sqrt((GravitationalConstant * pointMass) / vec.length());
	setVelocity(perpendicularDir * velocitySize);
}

void PointEuler::step(double dt)
{
	// semi-implicit euler
	velocity += acceleration * dt;
	position += velocity * dt;
}

void PointEuler::setVelocity(const vec2d& vel)
{
	velocity = vel;
}

void PointEuler::addVelocity(const vec2d& vel)
{
	velocity += vel;
}

PointEuler::PointEuler(const Magnum2D::vec2d& pos, const Magnum2D::vec2d& vel)
	: Point(pos)
{
	setVelocity(vel);
}

void PointEuler::reset()
{
	position = acceleration = velocity = { 0.0, 0.0 };
}

PointVerlet::PointVerlet(const Magnum2D::vec2d& pos, const Magnum2D::vec2d& vel)
	: Point(pos)
{
	setVelocity(vel);
}

void PointVerlet::step(double dt)
{
	auto move = position - positionOld;

	positionOld = position;
	position += move + dt * dt * acceleration;

	lastDt = dt;
}

void PointVerlet::setVelocity(const vec2d& vel)
{
	positionOld = position - vel * lastDt;
}

void PointVerlet::addVelocity(const vec2d& vel)
{
	positionOld = positionOld - vel * lastDt;
}

void PointVerlet::reset()
{
	position = acceleration = positionOld = { 0.0, 0.0 };
}

PointRungeKutta::PointRungeKutta(const std::vector<MassPoint>& p, const Magnum2D::vec2d& pos, const Magnum2D::vec2d& vel)
	: Point(pos), massPoints(p)
{
	setVelocity(vel);
}

// https://gafferongames.com/post/integration_basics/
void PointRungeKutta::step(double dt)
{
	struct Derivative
	{
		vec2d dpos;
		vec2d dvel;
	};

	auto evaluate = [&](double dt, const Derivative& d)
	{
		PointRungeKutta state = *this;
		state.position = position + d.dpos * dt;
		state.velocity = velocity + d.dvel * dt;

		Derivative output;
		output.dpos = state.velocity;
		output.dvel = state.computeAcceleration(massPoints);

		return output;
	};

	auto a = evaluate(0.0, {});
	auto b = evaluate(dt * 0.5, a);
	auto c = evaluate(dt * 0.5, b);
	auto d = evaluate(dt, c);

	vec2d dposdt = (1.0 / 6.0) * (a.dpos + 2.0 * (b.dpos + c.dpos) + d.dpos);
	vec2d dveldt = (1.0 / 6.0) * (a.dvel + 2.0 * (b.dvel + c.dvel) + d.dvel);

	position = position + dposdt * dt;
	velocity = velocity + dveldt * dt;
}

void PointRungeKutta::setVelocity(const vec2d& vel)
{
	velocity = vel;
}

void PointRungeKutta::addVelocity(const vec2d& vel)
{
	velocity += vel;
}

void PointRungeKutta::reset()
{
	position = acceleration = velocity = { 0.0, 0.0 };
}
