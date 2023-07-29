#pragma once
#include <Magnum2D.h>
#include <vector>
#include <memory>

extern double GravitationalConstant;
extern double GravityThreshold;
extern double SimulationDt;

struct PointMass
{
	Magnum2D::vec2d position;
	double mass = 1.0;
};

struct Burn
{
	double time;
	Magnum2D::vec2d velocity;
	Magnum2D::vec2d simulatedPosition;
};
using BurnPtr = std::unique_ptr<Burn>;

struct Point
{
	Point(const Magnum2D::vec2d& pos = { 0.0, 0.0 });

	Magnum2D::vec2d position;
	Magnum2D::vec2d acceleration;
	double mass = 1.0f;

	virtual void step(double dt) = 0;
	virtual void setVelocity(const Magnum2D::vec2d& vel) = 0;
	virtual void addVelocity(const Magnum2D::vec2d& vel) = 0;
	virtual void reset() = 0;

	std::tuple<std::vector<Magnum2D::vec2d>, std::vector<double>> simulate(const std::vector<PointMass>& points, const std::vector<BurnPtr>& burns, double dt, double seconds, int32_t numPoints);

	void applyForce(const Magnum2D::vec2d& force);
	Magnum2D::vec2d attractForce(Magnum2D::vec2d point, double pointMass) const;
	void initializeCircularOrbit(Magnum2D::vec2d point, double pointMass);
	Magnum2D::vec2d computeAcceleration(const std::vector<PointMass>& massPoints);
};

struct PointEuler : public Point
{
	PointEuler(const Magnum2D::vec2d& pos = { 0.0, 0.0 }, const Magnum2D::vec2d& vel = { 0.0, 0.0 });

	Magnum2D::vec2d velocity;

	void step(double dt) override;
	void setVelocity(const Magnum2D::vec2d& vel) override;
	void addVelocity(const Magnum2D::vec2d& vel) override;
	void reset() override;
};

struct PointVerlet : public Point
{
	PointVerlet(const Magnum2D::vec2d& pos = { 0.0, 0.0 }, const Magnum2D::vec2d& vel = { 0.0, 0.0 });

	Magnum2D::vec2d positionOld;
	double lastDt = SimulationDt;

	void step(double dt) override;
	void setVelocity(const Magnum2D::vec2d& vel) override;
	void addVelocity(const Magnum2D::vec2d& vel) override;
	void reset() override;
};

struct PointRungeKutta : public Point
{
	PointRungeKutta(const std::vector<PointMass>& p, const Magnum2D::vec2d& pos = { 0.0, 0.0 }, const Magnum2D::vec2d& vel = { 0.0, 0.0 });

	Magnum2D::vec2d velocity;
	std::vector<PointMass> massPoints;

	void step(double dt) override;
	void setVelocity(const Magnum2D::vec2d& vel) override;
	void addVelocity(const Magnum2D::vec2d& vel) override;
	void reset() override;
};
