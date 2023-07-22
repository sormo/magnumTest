#include "point.h"

using namespace Magnum2D;

float GravitationalConstant = 0.8f;
float GravityThreshold = 10.0f;

vec2 Point::computeAcceleration(const std::vector<PointMass>& massPoints)
{
	vec2 result;

	for (const auto& p : massPoints)
	{
		if ((p.position - position).length() > mass * GravityThreshold)
			continue;

		result += attractForce(p.position, p.mass) / mass;
	}

	return result;
}

std::tuple<std::vector<vec2>, std::vector<float>, std::vector<Magnum2D::vec2>> Point::simulate(const std::vector<PointMass>& massPoints, const std::vector<Burn>& burns, float dt, float seconds, int32_t numPoints)
{
	std::vector<vec2> points;
	std::vector<float> times;
	std::vector<Magnum2D::vec2> burnPositions;
	points.reserve(numPoints);
	times.reserve(numPoints);
	burnPositions.reserve(burns.size());

	points.push_back(position);
	times.push_back(0.0f);
	
	int32_t steps = std::ceil(seconds / dt);
	size_t burnIndex = 0;
	
	float accumulatedTime = 0.0f;
	for (int i = 0; i < steps; i++)
	{
		if (burnIndex < burns.size())
		{
			if (accumulatedTime >= burns[burnIndex].time)
			{
				addVelocity(burns[burnIndex].velocity);
				burnPositions.push_back(position);
				burnIndex++;
			}
		}

		acceleration = computeAcceleration(massPoints);

		step(dt);

		accumulatedTime += dt;
		int32_t expectedPoints = accumulatedTime * (float)numPoints / seconds;
		if (expectedPoints < points.size())
		{
			points.push_back(position);
			times.push_back(accumulatedTime);
		}
	}

	return { points, times, burnPositions };
}

void Point::applyForce(const vec2& force)
{
	acceleration += (force / mass);
}

vec2 Point::attractForce(vec2 point, float pointMass) const
{
	vec2 dir = (point - position);
	float distance = dir.length();
	dir = dir.normalized();

	return (GravitationalConstant * pointMass * mass / (distance * distance)) * dir;
}

void Point::initializeCircularOrbit(vec2 point, float pointMass)
{
	vec2 vec = point - position;
	vec2 perpendicularDir = vec2(-vec.y(), vec.x()).normalized();

	float velocitySize = std::sqrtf((GravitationalConstant * pointMass) / vec.length());
	setVelocity(perpendicularDir * velocitySize);
}

void PointEuler::step(float dt)
{
	// semi-implicit euler
	velocity += acceleration * dt;
	position += velocity * dt;
}

void PointEuler::setVelocity(const vec2& vel)
{
	velocity = vel;
}

void PointEuler::addVelocity(const vec2& vel)
{
	velocity += vel;
}

void PointEuler::reset()
{
	position = acceleration = velocity = { 0.0f, 0.0f };
}

void PointVerlet::step(float dt)
{
	auto move = position - positionOld;

	positionOld = position;
	position += move + dt * dt * acceleration;

	lastDt = dt;
}

void PointVerlet::setVelocity(const vec2& vel)
{
	positionOld = position - vel * lastDt;
}

void PointVerlet::addVelocity(const vec2& vel)
{
	positionOld = positionOld - vel * lastDt;
}

void PointVerlet::reset()
{
	position = acceleration = positionOld = { 0.0f, 0.0f };
}

// https://gafferongames.com/post/integration_basics/
void PointRungeKutta::step(float dt)
{
	struct Derivative
	{
		vec2 dpos;
		vec2 dvel;
	};

	auto evaluate = [&](float dt, const Derivative& d)
	{
		PointRungeKutta state = *this;
		state.position = position + d.dpos * dt;
		state.velocity = velocity + d.dvel * dt;

		Derivative output;
		output.dpos = state.velocity;
		output.dvel = state.computeAcceleration(massPoints);

		return output;
	};

	auto a = evaluate(0.0f, {});
	auto b = evaluate(dt * 0.5f, a);
	auto c = evaluate(dt * 0.5f, b);
	auto d = evaluate(dt, c);

	vec2 dposdt = (1.0f / 6.0f) * (a.dpos + 2.0f * (b.dpos + c.dpos) + d.dpos);
	vec2 dveldt = (1.0f / 6.0f) * (a.dvel + 2.0f * (b.dvel + c.dvel) + d.dvel);

	position = position + dposdt * dt;
	velocity = velocity + dveldt * dt;
}

void PointRungeKutta::setVelocity(const Magnum2D::vec2& vel)
{
	velocity = vel;
}

void PointRungeKutta::addVelocity(const Magnum2D::vec2& vel)
{
	velocity += vel;
}

void PointRungeKutta::reset()
{
	position = acceleration = velocity = { 0.0f, 0.0f };
}
