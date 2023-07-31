#pragma once
#include "point.h"
#include "trajectory.h"
#include <vector>

namespace Simulation
{
	using namespace Magnum2D;

	template<typename T>
	std::vector<Trajectory> Simulate(std::vector<T>& points, const std::vector<std::vector<BurnPtr>>& burns, double dt, double seconds, int32_t numPoints)
	{
		std::vector<Trajectory> result(points.size(), Trajectory{ {} });

		for (auto& t : result)
		{
			t.points.reserve(numPoints);
			t.times.reserve(numPoints);
		}

		for (size_t i = 0; i < points.size(); i++)
		{
			result[i].points.push_back((vec2)points[i].position);
			result[i].times.push_back(0.0f);
		}

		int32_t steps = std::ceil(seconds / dt);
		std::vector<size_t> burnIndex(burns.size(), 0);

		std::vector<double> accelerations(points.size(), 0.0);

		double accumulatedTime = 0.0;
		for (int i = 0; i < steps; i++)
		{
			// apply burns
			for (size_t j = 0; j < burns.size(); j++)
			{
				if (burnIndex[j] < burns[j].size())
				{
					Burn* burn = burns[j][burnIndex[j]].get();

					if (accumulatedTime >= burn->time)
					{
						points[j].addVelocity(burn->velocity);
						burn->simulatedPosition = points[j].position;
						burnIndex[j]++;
					}
				}
			}

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
			//int32_t expectedPoints = (accumulatedTime * (double)numPoints) / seconds;
			//if (expectedPoints > points.size())
			{
				for (size_t j = 0; j < points.size(); j++)
				{
					result[j].points.push_back((vec2)points[j].position);
					result[j].times.push_back(accumulatedTime);
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
			//int32_t expectedPoints = (accumulatedTime * (double)numPoints) / seconds;
			//if (expectedPoints > points.size())
			{
				points.push_back(point.position);
				times.push_back(accumulatedTime);
			}
		}

		return { points, times };
	}
}
