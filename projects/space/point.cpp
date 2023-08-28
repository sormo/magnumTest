#include "point.h"

using namespace Magnum2D;

double GravitationalConstant = 0.8;
double GravityThreshold = 0.001;

Point::Point(const Magnum2D::vec2d& pos, double m)
{
	position = pos;
	mass = m;
}

void Point::applyForce(const vec2d& force)
{
	acceleration += (force / mass);
}

vec2d Point::attractForce(vec2d point, double pointMass) const
{
	// TODO optimize square length
	vec2d dir = (point - position);
	double distance = dir.length();

	if (distance == 0.0)
		return {};

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

Magnum2D::vec2d PointEuler::getVelocity()
{
	return velocity;
}

PointEuler::PointEuler(const Magnum2D::vec2d& pos, const Magnum2D::vec2d& vel, double mass)
	: Point(pos, mass)
{
	setVelocity(vel);
}

void PointEuler::reset()
{
	position = acceleration = velocity = { 0.0, 0.0 };
}

PointVerlet::PointVerlet(const Magnum2D::vec2d& pos, const Magnum2D::vec2d& vel, double mass)
	: Point(pos, mass)
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

Magnum2D::vec2d PointVerlet::getVelocity()
{
	return position - positionOld;
}

void PointVerlet::reset()
{
	position = acceleration = positionOld = { 0.0, 0.0 };
}

PointRungeKutta::PointRungeKutta(const Magnum2D::vec2d& pos, const Magnum2D::vec2d& vel, double mass)
	: Point(pos, mass)
{
	setVelocity(vel);
}

void PointRungeKutta::step(double dt)
{
	vec2d vel = (1.0 / 6.0) * (k1.velocity + 2.0 * (k2.velocity + k3.velocity) + k4.velocity);
	vec2d acc = (1.0 / 6.0) * (k1.acceleration + 2.0 * (k2.acceleration + k3.acceleration) + k4.acceleration);

	position = position + vel * dt;
	velocity = velocity + acc * dt;
}

void PointRungeKutta::setVelocity(const vec2d& vel)
{
	velocity = vel;
}

void PointRungeKutta::addVelocity(const vec2d& vel)
{
	velocity += vel;
}

Magnum2D::vec2d PointRungeKutta::getVelocity()
{
	return velocity;
}

void PointRungeKutta::reset()
{
	position = acceleration = velocity = { 0.0, 0.0 };
}

MassPoint::MassPoint()
{
	recomputeEffectiveRadius();
}

void MassPoint::recomputeEffectiveRadius()
{
	effectiveRadius = std::sqrt(GravitationalConstant * mass / GravityThreshold);
	effectiveRadiusSqr = effectiveRadius * effectiveRadius;
}

double MassPoint::getMass() const
{
	return mass;
}

void MassPoint::setMass(double newMass)
{
	mass = newMass;
	recomputeEffectiveRadius();
}

double MassPoint::getEffectiveRadius() const
{
	return effectiveRadius;
}

double MassPoint::getEffectiveRadiusSqr() const
{
	return effectiveRadiusSqr;
}
