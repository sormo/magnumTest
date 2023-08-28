#pragma once
#include <Magnum2D.h>
#include "vectorHandler.h"
#include "simulation.h"
#include <set>

namespace TestBodies
{
	extern int32_t TrajectoryPointCount;
	extern float CurrentTime;
}

using namespace Magnum2D;

struct Bodies
{
	const float ForceDrawFactor = 0.2f;

	size_t AddBody(const char* name, vec2d position, vec2d velocity, double mass = 1.0);

	void SimulateClear(double time);
	void SimulateExtend(double time);

	void Draw(bool euler, bool verlet, bool rungeKutta);

	vec2 GetPosition(size_t index, double time);

	std::optional<size_t> SelectBody(double time, const vec2& selectPosition, float selectRadius);

	void SetParent(size_t parent, size_t child);
	void ClearParent(size_t child);

	double simulatedTime = 0.0;

	template<typename T>
	struct BodySimulation
	{
		T initialPoint;
		T currentPoint;

		Trajectory trajectoryGlobal;
		Trajectory trajectoryParent;

		void Clear()
		{
			trajectoryGlobal.clear();
			trajectoryParent.clear();
			currentPoint = initialPoint;
		}
	};

	struct Body
	{
		std::string name;
		std::optional<size_t> parent;
		std::set<size_t> childs;

		vec2d initialPosition;
		vec2d initialVelocity;
		double mass;

		col3 color;

		BodySimulation<PointEuler> simulationEuler;
		BodySimulation<PointVerlet> simulationVerlet;
		BodySimulation<PointRungeKutta> simulationRK4;

		template<class T> BodySimulation<T>& GetSimulation() {}
		template<> BodySimulation<PointEuler>& GetSimulation() { return simulationEuler; }
		template<> BodySimulation<PointVerlet>& GetSimulation() { return simulationVerlet; }
		template<> BodySimulation<PointRungeKutta>& GetSimulation() { return simulationRK4; }

		void SetInitialState(const vec2d& position, const vec2d& velocity, double mass)
		{
			initialPosition = position;
			initialVelocity = velocity;
			mass = mass;
			simulationEuler.initialPoint = PointEuler(initialPosition, initialVelocity, mass);
			simulationVerlet.initialPoint = PointVerlet(initialPosition, initialVelocity, mass);
			simulationRK4.initialPoint = PointRungeKutta(initialPosition, initialVelocity, mass);
		}

		void RecomputeEffectiveRadius()
		{
			simulationEuler.initialPoint.recomputeEffectiveRadius();
			simulationEuler.currentPoint.recomputeEffectiveRadius();
			simulationVerlet.initialPoint.recomputeEffectiveRadius();
			simulationVerlet.currentPoint.recomputeEffectiveRadius();
			simulationRK4.initialPoint.recomputeEffectiveRadius();
			simulationRK4.currentPoint.recomputeEffectiveRadius();
		}
	};

	std::vector<Body> bodies;

	VectorHandler vectorHandler;
};
