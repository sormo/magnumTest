#pragma once
#include "utils.h"
#include <Magnum2D.h>
#include <vector>
#include <memory>

extern double GravitationalConstant;
extern double GravityThreshold;
extern double SimulationDt;

struct MassPoint
{
	MassPoint();

	Magnum2D::vec2d position;

	double getMass() const;
	void setMass(double mass);

	double getEffectiveRadius() const;
	void recomputeEffectiveRadius();

protected:

	double mass = 1.0;
	double effectiveRadius = 0.0f;
	double effectiveRadiusSqr = 0.0f;
};

struct Burn
{
	double time;
	Magnum2D::vec2d velocity;
	Magnum2D::vec2d simulatedPosition;
};
using BurnPtr = std::unique_ptr<Burn>;

struct Point : public MassPoint
{
	Point(const Magnum2D::vec2d& pos = { 0.0, 0.0 });

	Magnum2D::vec2d acceleration;

	// thisIndex is index in massPoints which must be skipped when computing acceleration
	template<class T>
	void prepareStep(const std::vector<T>& massPoints, float, size_t thisIndex = -1)
	{
		Magnum2D::vec2d acc;
		for (const auto& m : massPoints)
			acc += attractForce(m.position, m.getMass()) / mass;
		acceleration = computeAcceleration(massPoints, thisIndex);
	}

	virtual void step(double dt) = 0;
	virtual void setVelocity(const Magnum2D::vec2d& vel) = 0;
	virtual void addVelocity(const Magnum2D::vec2d& vel) = 0;
	virtual Magnum2D::vec2d getVelocity() = 0;
	virtual void reset() = 0;

	void applyForce(const Magnum2D::vec2d& force);
	Magnum2D::vec2d attractForce(Magnum2D::vec2d point, double pointMass) const;
	void initializeCircularOrbit(Magnum2D::vec2d point, double pointMass);

	template<class T>
	Magnum2D::vec2d computeAcceleration(const std::vector<T>& massPoints, size_t thisIndex = -1)
	{
		Magnum2D::vec2d result;

		for (size_t i = 0; i < massPoints.size(); i++)
		{
			if (i == thisIndex)
				continue;

			if (Utils::DistanceSqr(massPoints[i].position, position) > effectiveRadiusSqr)
				continue;

			result += attractForce(massPoints[i].position, massPoints[i].getMass()) / mass;
		}

		return result;
	}
};

struct PointEuler : public Point
{
	PointEuler(const Magnum2D::vec2d& pos = { 0.0, 0.0 }, const Magnum2D::vec2d& vel = { 0.0, 0.0 });

	Magnum2D::vec2d velocity;

	void step(double dt) override;
	void setVelocity(const Magnum2D::vec2d& vel) override;
	void addVelocity(const Magnum2D::vec2d& vel) override;
	Magnum2D::vec2d getVelocity() override;
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
	Magnum2D::vec2d getVelocity() override;
	void reset() override;
};

struct PointRungeKutta : public Point
{
	PointRungeKutta(const Magnum2D::vec2d& pos = { 0.0, 0.0 }, const Magnum2D::vec2d& vel = { 0.0, 0.0 });

	Magnum2D::vec2d velocity;

	Magnum2D::vec2d dposdt;
	Magnum2D::vec2d dveldt;

	template<class T>
	void prepareStep(const std::vector<T>& massPoints, float dt, size_t thisIndex = -1)
	{
		struct Derivative
		{
			Magnum2D::vec2d dpos;
			Magnum2D::vec2d dvel;
		};

		auto evaluate = [&](double dt, const Derivative& d)
		{
			PointRungeKutta state = *this;
			state.position = position + d.dpos * dt;
			state.velocity = velocity + d.dvel * dt;

			Derivative output;
			output.dpos = state.velocity;
			output.dvel = state.computeAcceleration(massPoints, thisIndex);

			return output;
		};

		auto a = evaluate(0.0, {});
		auto b = evaluate(dt * 0.5, a);
		auto c = evaluate(dt * 0.5, b);
		auto d = evaluate(dt, c);

		dposdt = (1.0 / 6.0) * (a.dpos + 2.0 * (b.dpos + c.dpos) + d.dpos);
		dveldt = (1.0 / 6.0) * (a.dvel + 2.0 * (b.dvel + c.dvel) + d.dvel);
	}

	void step(double dt) override;
	void setVelocity(const Magnum2D::vec2d& vel) override;
	void addVelocity(const Magnum2D::vec2d& vel) override;
	Magnum2D::vec2d getVelocity() override;
	void reset() override;
};
