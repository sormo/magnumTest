#pragma once
#include <Magnum2D.h>
#include <vector>

extern float GravitationalConstant;
extern float GravityThreshold;
extern float SimulationDt;

struct PointMass
{
	Magnum2D::vec2 position;
	float mass = 1.0f;
};

struct Burn
{
	float time;
	Magnum2D::vec2 velocity;
};

struct Point
{
	Magnum2D::vec2 position;
	Magnum2D::vec2 acceleration;
	float mass = 1.0f;

	virtual void step(float dt) = 0;
	virtual void setVelocity(const Magnum2D::vec2& vel) = 0;
	virtual void addVelocity(const Magnum2D::vec2& vel) = 0;
	virtual void reset() = 0;

	std::tuple<std::vector<Magnum2D::vec2>, std::vector<float>, std::vector<Magnum2D::vec2>> simulate(const std::vector<PointMass>& points, const std::vector<Burn>& burns, float dt, float seconds, int32_t numPoints);

	void applyForce(const Magnum2D::vec2& force);
	Magnum2D::vec2 attractForce(Magnum2D::vec2 point, float pointMass) const;
	void initializeCircularOrbit(Magnum2D::vec2 point, float pointMass);
	Magnum2D::vec2 computeAcceleration(const std::vector<PointMass>& massPoints);
};

struct PointEuler : public Point
{
	Magnum2D::vec2 velocity;

	void step(float dt) override;
	void setVelocity(const Magnum2D::vec2& vel) override;
	void addVelocity(const Magnum2D::vec2& vel) override;
	void reset() override;
};

struct PointVerlet : public Point
{
	Magnum2D::vec2 positionOld;
	float lastDt = SimulationDt;

	void step(float dt) override;
	void setVelocity(const Magnum2D::vec2& vel) override;
	void addVelocity(const Magnum2D::vec2& vel) override;
	void reset() override;
};

struct PointRungeKutta : public Point
{
	PointRungeKutta(const std::vector<PointMass>& p) : massPoints(p) {}

	Magnum2D::vec2 velocity;
	std::vector<PointMass> massPoints;

	void step(float dt) override;
	void setVelocity(const Magnum2D::vec2& vel) override;
	void addVelocity(const Magnum2D::vec2& vel) override;
	void reset() override;
};
