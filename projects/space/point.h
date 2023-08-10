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
	double getEffectiveRadiusSqr() const;
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
			// TODO
			//if (Utils::DistanceSqr(massPoints[i].position, position) > massPoints[i].getEffectiveRadiusSqr())
			//	continue;

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

	Magnum2D::vec2d positionTemp;

	struct State
	{
		Magnum2D::vec2d velocity;
		Magnum2D::vec2d acceleration;
	};
	State k1;
	State k2;
	State k3;
	State k4;

	// TODO fix somehow
	Magnum2D::vec2d attractForceTemp(Magnum2D::vec2d pointPosition, double pointMass) const
	{
		// TODO optimize square length
		Magnum2D::vec2d dir = (pointPosition - positionTemp);
		double distance = dir.length();

		if (distance == 0.0)
			return {};

		dir = dir.normalized();

		return ((GravitationalConstant * pointMass * mass / (distance * distance))) * dir;
	}
	Magnum2D::vec2d computeAccelerationTemp(const std::vector<PointRungeKutta>& massPoints, size_t thisIndex)
	{
		Magnum2D::vec2d result;

		for (size_t i = 0; i < massPoints.size(); i++)
		{
			if (i == thisIndex)
				continue;

			//if (Utils::DistanceSqr(massPoints[i].position, position) > massPoints[i].getEffectiveRadiusSqr())
			//	continue;

			result += attractForceTemp(massPoints[i].positionTemp, massPoints[i].getMass()) / mass;
		}

		return result;
	}

	void stepK1Begin(double)
	{
		// the points starts at the beginning
		positionTemp = position;
	}

	void stepK1End(const std::vector<PointRungeKutta>& massPoints, double, size_t thisIndex)
	{
		k1.acceleration = computeAccelerationTemp(massPoints, thisIndex);
		k1.velocity = velocity;
	}

	void stepK2Begin(double dt)
	{
		// move each point to temporary position at dt/2 using the velocity from previous step
		positionTemp = position + k1.velocity * (dt / 2.0);
	}

	void stepK2End(const std::vector<PointRungeKutta>& massPoints, double dt, size_t thisIndex)
	{
		// when points are at new position, fill in the state
		// we will use acceleration from the beginning to modify the velicity, because it was affecting velocity for dt/2 time
		k2.acceleration = computeAccelerationTemp(massPoints, thisIndex);
		k2.velocity = velocity + k1.acceleration * (dt / 2.0);
	}

	void stepK3Begin(double dt)
	{
		positionTemp = position + k2.velocity * (dt / 2.0);
	}

	void stepK3End(const std::vector<PointRungeKutta>& massPoints, double dt, size_t thisIndex)
	{
		// here not sure what acceleration to use
		k3.acceleration = computeAccelerationTemp(massPoints, thisIndex);
		k3.velocity = velocity + k1.acceleration * (dt / 2.0);
	}

	void stepK4Begin(double dt)
	{
		positionTemp = position + k3.velocity * dt;
	}

	void stepK4End(const std::vector<PointRungeKutta>& massPoints, double dt, size_t thisIndex)
	{
		k4.acceleration = computeAccelerationTemp(massPoints, thisIndex);
		k4.velocity = velocity + k1.acceleration * dt;
	}

	void step(double dt) override;
	void setVelocity(const Magnum2D::vec2d& vel) override;
	void addVelocity(const Magnum2D::vec2d& vel) override;
	Magnum2D::vec2d getVelocity() override;
	void reset() override;
};
