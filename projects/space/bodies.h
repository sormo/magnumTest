#pragma once
#include <Magnum2D.h>
#include "vectorHandler.h"
#include "simulation.h"
#include "conicfit/conicApproximation.h"
#include <set>

namespace TestBodies
{
	extern int32_t TrajectoryPointCount;
	extern float CurrentTime;
}

using namespace Magnum2D;

struct Bodies
{
	static const float ForceDrawFactor;

	size_t AddBody(const char* name, vec2d position, vec2d velocity, double mass = 1.0);

	void SimulateClear(double time);
	void SimulateExtend(double time);
	// this is optimization, we will simulate just few bodies hoping that changed trajectories won't affect
	// other bodies much
	void Resimulate(std::set<size_t> bodies);
	void Resimulate(size_t body);

	void Draw(bool euler, bool verlet, bool rungeKutta, bool approximated, bool computed);

	vec2 GetPosition(size_t index, double time);
	vec2 GetCurrentPosition(size_t index);
	float GetCurrentDistanceToParent(size_t index);

	std::optional<size_t> SelectBody(double time, const vec2& selectPosition, float selectRadius);
	size_t GetBodyOfGrab(VectorHandler::Vector grab);

	void SetParentUser(size_t child, std::optional<size_t> parent);
	void SetParentSimulation(size_t child, std::optional<size_t> parent);
	void SetParentCommon(size_t child, std::optional<size_t> parent);

	void SetParentInternal(size_t child, size_t parent);
	void ClearParentInternal(size_t child);


	double simulatedTime = 0.0;

	template<typename T>
	struct BodySimulation
	{
		T initialPoint;
		T currentPoint;

		Trajectory trajectoryGlobal;
		Trajectory trajectoryParent;

		size_t currentIndex = 0;
		void SetCurrentIndex(double currentTime)
		{
			currentIndex = trajectoryGlobal.getPoint(currentTime);
		}

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
		// current parent
		std::optional<size_t> parent;
		std::set<size_t> childs;
		// parent computed by simulation
		std::optional<size_t> parentSimulation;

		vec2d initialPosition;
		vec2d initialVelocity;
		double mass;

		bool isStar = false;

		col3 color;
		VectorHandler::Vector initialVector;

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
			this->mass = mass;
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

		void SetCurrentTime(double time)
		{
			simulationEuler.SetCurrentIndex(time);
			simulationVerlet.SetCurrentIndex(time);
			simulationRK4.SetCurrentIndex(time);
		}

		bool HasCorrectParent()
		{
			return parent == parentSimulation;
		}

		struct Conic
		{
			vec2 position;
			float rotation;
			std::vector<vec2> points;
		};

		Conic conicApproximatedFromPoints;
		Conic conicComputedFromParent;
	};

	void ComputeParents(std::vector<Body>& bodies);
	void ComputeConics(std::vector<Body>& bodies);

	void DrawConic(const Magnum2D::vec2& parentPosition, Body::Conic& conic, float width, const Magnum2D::col3& color);

	void SimulateClearInternal(double time, std::set<size_t> indices);

	std::vector<Body> bodies;

	VectorHandler vectorHandler;
};
