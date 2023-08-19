#pragma once
#include "point.h"
#include "trajectory.h"
#include <vector>

namespace Simulation
{
	using namespace Magnum2D;

	template<typename T>
	void ApplyBurnIfNeeded(T& point, const std::vector<BurnPtr>& burns, size_t& currentBurn, double accumulatedTime)
	{
		if (currentBurn == burns.size())
			return;

		Burn* burn = burns[currentBurn].get();

		if (accumulatedTime >= burn->time)
		{
			point.addVelocity(burn->velocity);
			burn->simulatedPosition = point.position;
			currentBurn++;
		}
	}

	template<typename T>
	void ApplyBurns(std::vector<T>& points, const std::vector<std::vector<BurnPtr>>& burns, std::vector<size_t>& burnIndex, double accumulatedTime)
	{
		for (size_t j = 0; j < points.size(); j++)
		{
			ApplyBurnIfNeeded(points[j], burns[j], burnIndex[j], accumulatedTime);
		}
	}

	// TODO this template can be called instead of runge-kutta
	template<typename T>
	std::vector<Trajectory> Simulate(std::vector<T>& points, const std::vector<std::vector<BurnPtr>>& burns, double dt, double seconds, double timeOffset = 0.0, int32_t numPoints = 60)
	{
		std::vector<Trajectory> result(points.size(), Trajectory{ {} });

		for (auto& t : result)
		{
			t.positions.reserve(numPoints);
			t.velocities.reserve(numPoints);
			t.times.reserve(numPoints);
		}

		for (size_t i = 0; i < points.size(); i++)
		{
			result[i].positions.push_back((vec2)points[i].position);
			result[i].velocities.push_back((vec2)points[i].getVelocity());
			result[i].times.push_back(0.0f);
		}

		int32_t steps = std::ceil(seconds / dt);
		std::vector<size_t> burnIndex(burns.size(), 0);

		std::vector<double> accelerations(points.size(), 0.0);

		double accumulatedTime = 0.0;
		for (int i = 0; i < steps; i++)
		{
			// apply burns
			ApplyBurns(points, burns, burnIndex, accumulatedTime);

			// update accelerations
			for (size_t j = 0; j < points.size(); j++)
			{
				points[j].prepareStep(points, dt, j);
			}

			// update velocities and positions
			for (size_t j = 0; j < points.size(); j++)
			{
				points[j].step(dt);
			}

			accumulatedTime += dt;

			// update trajectories
			int32_t expectedPoints = (accumulatedTime * (double)numPoints) / seconds;
			if (expectedPoints > result[0].times.size())
			{
				for (size_t j = 0; j < points.size(); j++)
				{
					result[j].positions.push_back((vec2)points[j].position);
					result[j].velocities.push_back((vec2)points[j].getVelocity());
					result[j].times.push_back(timeOffset + accumulatedTime);
				}
			}
		}

		return result;
	}

	static std::vector<Trajectory> Simulate(std::vector<PointRungeKutta>& points, const std::vector<std::vector<BurnPtr>>& burns, double dt, double seconds, double timeOffset, int32_t numPoints)
	{
		std::vector<Trajectory> result(points.size(), Trajectory{ {} });

		for (auto& t : result)
		{
			t.positions.reserve(numPoints);
			t.velocities.reserve(numPoints);
			t.times.reserve(numPoints);
		}

		for (size_t i = 0; i < points.size(); i++)
		{
			result[i].positions.push_back((vec2)points[i].position);
			result[i].velocities.push_back((vec2)points[i].getVelocity());
			result[i].times.push_back(0.0f);
		}

		int32_t steps = std::ceil(seconds / dt);
		std::vector<size_t> burnIndex(burns.size(), 0);

		std::vector<double> accelerations(points.size(), 0.0);

		double accumulatedTime = 0.0;
		for (int i = 0; i < steps; i++)
		{
			// apply burns
			ApplyBurns(points, burns, burnIndex, accumulatedTime);

			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK1Begin(dt);
			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK1End(points, dt, j);

			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK2Begin(dt);
			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK2End(points, dt, j);

			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK3Begin(dt);
			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK3End(points, dt, j);

			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK4Begin(dt);
			for (size_t j = 0; j < points.size(); j++)
				points[j].stepK4End(points, dt, j);

			// update velocities and positions
			for (size_t j = 0; j < points.size(); j++)
				points[j].step(dt);

			accumulatedTime += dt;

			// update trajectories
			int32_t expectedPoints = (accumulatedTime * (double)numPoints) / seconds;
			if (expectedPoints > result[0].times.size())
			{
				for (size_t j = 0; j < points.size(); j++)
				{
					result[j].positions.push_back((vec2)points[j].position);
					result[j].velocities.push_back((vec2)points[j].getVelocity());
					result[j].times.push_back(timeOffset + accumulatedTime);
				}
			}
		}

		return result;
	}

	// simulate single point given static mass points
	template<typename T>
	std::tuple<std::vector<vec2d>, std::vector<double>> Simulate(T& point, const std::vector<MassPoint>& massPoints, const std::vector<BurnPtr>& burns, double dt, double seconds, int32_t numPoints)
	{
		std::vector<vec2d> points;
		std::vector<double> times;
		points.reserve(numPoints);
		times.reserve(numPoints);

		points.push_back(point.position);
		times.push_back(0.0f);

		int32_t steps = std::ceil(seconds / dt);
		size_t burnIndex = 0;

		double accumulatedTime = 0.0;
		for (int i = 0; i < steps; i++)
		{
			if (burnIndex < burns.size())
			{
				if (accumulatedTime >= burns[burnIndex]->time)
				{
					point.addVelocity(burns[burnIndex]->velocity);
					burns[burnIndex]->simulatedPosition = point.position;
					burnIndex++;
				}
			}

			point.prepareStep(massPoints, dt);

			point.step(dt);

			accumulatedTime += dt;
			int32_t expectedPoints = (accumulatedTime * (double)numPoints) / seconds;
			if (expectedPoints > times.size())
			{
				points.push_back(point.position);
				times.push_back(accumulatedTime);
			}
		}

		return { points, times };
	}
}
