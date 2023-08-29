#include "bodies.h"

size_t Bodies::AddBody(const char* name, vec2d position, vec2d velocity, double mass)
{
	Body newBody;
	newBody.name = name;
	newBody.initialPosition = position;
	newBody.initialVelocity = velocity;
	newBody.mass = mass;

	newBody.simulationRK4.initialPoint = decltype(newBody.simulationRK4.initialPoint)(position, velocity, mass);
	newBody.simulationEuler.initialPoint = decltype(newBody.simulationEuler.initialPoint)(position, velocity, mass);
	newBody.simulationVerlet.initialPoint = decltype(newBody.simulationVerlet.initialPoint)(position, velocity, mass);

	newBody.simulationRK4.currentPoint = newBody.simulationRK4.initialPoint;
	newBody.simulationEuler.currentPoint = newBody.simulationEuler.initialPoint;
	newBody.simulationVerlet.currentPoint = newBody.simulationVerlet.initialPoint;

	newBody.color = Utils::GetRandomColor();

	bodies.push_back(std::move(newBody));

	vec2 from = (vec2)position;
	vec2 to = (vec2)(position + velocity * ForceDrawFactor);
	VectorHandler::OnChange onFromChange = [this, index = bodies.size() - 1](const vec2& v, void*)
		{
			bodies[index].SetInitialState((vec2d)v, bodies[index].initialVelocity, bodies[index].mass);
			return v;
		};
	VectorHandler::OnChange onToChange = [this, index = bodies.size() - 1](const vec2& v, void*)
		{
			vec2d velocity = (vec2d)(v / ForceDrawFactor) - bodies[index].initialPosition;

			bodies[index].SetInitialState(bodies[index].initialPosition, velocity, bodies[index].mass);
			return v;
		};

	vectorHandler.Push(from, to, nullptr, onFromChange, onToChange);

	return bodies.size() - 1;
}

template<class T>
void ProcessTrajectoriesParentRecursive(size_t body, std::vector<Bodies::Body>& bodies)
{
	auto& simulation = bodies[body].GetSimulation<T>();
	auto& trajectoryBody = simulation.trajectoryGlobal;
	auto& trajectoryParent = bodies[*bodies[body].parent].GetSimulation<T>().trajectoryParent;

	simulation.trajectoryParent.clear();

	for (size_t i = 0; i < simulation.trajectoryGlobal.positions.size(); i++)
	{
		simulation.trajectoryParent.positions.push_back(trajectoryBody.positions[i] - trajectoryParent.positions[i]);
		simulation.trajectoryParent.velocities.push_back(trajectoryBody.velocities[i] - trajectoryParent.velocities[i]);
	}
	simulation.trajectoryParent.times = trajectoryBody.times;

	for (size_t child : bodies[body].childs)
		ProcessTrajectoriesParentRecursive<T>(child, bodies);
}

template<class T>
void ProcessTrajectoriesParent(std::vector<Bodies::Body>& bodies)
{
	for (size_t i = 0; i < bodies.size(); i++)
	{
		if (bodies[i].parent)
			continue;
		// from the root
		bodies[i].GetSimulation<T>().trajectoryParent = bodies[i].GetSimulation<T>().trajectoryGlobal;

		for (auto child : bodies[i].childs)
			ProcessTrajectoriesParentRecursive<T>(child, bodies);
	}
}

template<class T>
void Simulate(double time, double simulatedTime, std::vector<Bodies::Body>& bodies)
{
	std::vector<std::vector<BurnPtr>> burns(bodies.size());
	std::vector<T> points;
	points.reserve(bodies.size());

	for (auto& body : bodies)
		points.push_back(body.GetSimulation<T>().currentPoint);

	auto newTrajectories = Simulation::Simulate(points, burns, SimulationDt, time, simulatedTime, TestBodies::TrajectoryPointCount);
	for (size_t i = 0; i < newTrajectories.size(); i++)
	{
		bodies[i].GetSimulation<T>().currentPoint = std::move(points[i]);
		bodies[i].GetSimulation<T>().trajectoryGlobal.extend(std::move(newTrajectories[i]), 0);
		//bodies[i].GetSimulation<T>().trajectoryGlobal.positions = std::move(newTrajectories[i].positions);
		//bodies[i].GetSimulation<T>().trajectoryGlobal.velocities = std::move(newTrajectories[i].velocities);
		//bodies[i].GetSimulation<T>().trajectoryGlobal.times = std::move(newTrajectories[i].times);
	}

	ProcessTrajectoriesParent<T>(bodies);
}

template<class T>
void SimulateClear(double time, std::vector<Bodies::Body>& bodies)
{
	for (auto& body : bodies)
		body.GetSimulation<T>().Clear();

	Simulate<T>(time, 0.0, bodies);
}

void Bodies::SimulateClear(double time)
{
	::SimulateClear<PointEuler>(time, bodies);
	::SimulateClear<PointVerlet>(time, bodies);
	::SimulateClear<PointRungeKutta>(time, bodies);

	simulatedTime = time;
}

void Bodies::SimulateExtend(double time)
{
	::Simulate<PointEuler>(time, simulatedTime, bodies);
	::Simulate<PointVerlet>(time, simulatedTime, bodies);
	::Simulate<PointRungeKutta>(time, simulatedTime, bodies);

	simulatedTime += time;
}

void Bodies::Draw(bool euler, bool verlet, bool rungeKutta)
{
	vectorHandler.Draw();
	
	if (simulatedTime == 0.0)
		return;

	for (auto& body : bodies)
	{
		if (body.parent)
		{
			auto index = bodies[*body.parent].GetSimulation<PointRungeKutta>().currentIndex;
			auto currentParentGlobalPosition = bodies[*body.parent].GetSimulation<PointRungeKutta>().trajectoryGlobal.positions[index];
			setTransform({ currentParentGlobalPosition, 0.0f });
		}

		if (euler)
			body.GetSimulation<PointEuler>().trajectoryParent.draw(0, body.GetSimulation<PointEuler>().currentIndex, rgb(50, 50, 50));
		if (verlet)
			body.GetSimulation<PointVerlet>().trajectoryParent.draw(0, body.GetSimulation<PointVerlet>().currentIndex, rgb(100, 100, 100));
		if (rungeKutta)
			body.GetSimulation<PointRungeKutta>().trajectoryParent.draw(0, body.GetSimulation<PointRungeKutta>().currentIndex, body.color);

		setTransform({});
	}
}

vec2 Bodies::GetPosition(size_t index, double time)
{
	if (time == 0.0)
		return (vec2)bodies[index].initialPosition;

	auto positionIndex = bodies[index].GetSimulation<PointRungeKutta>().trajectoryGlobal.getPoint(time);
	return bodies[index].GetSimulation<PointRungeKutta>().trajectoryGlobal.positions[positionIndex];
}

vec2 Bodies::GetCurrentPosition(size_t index)
{
	if (bodies[index].GetSimulation<PointRungeKutta>().trajectoryGlobal.positions.empty())
		return (vec2)bodies[index].initialPosition;

	return bodies[index].GetSimulation<PointRungeKutta>().trajectoryGlobal.positions[bodies[index].GetSimulation<PointRungeKutta>().currentIndex];
}

std::optional<size_t> Bodies::SelectBody(double time, const vec2& selectPosition, float selectRadius)
{
	for (size_t i = 0; i < bodies.size(); i++)
	{
		auto bodyPosition = GetPosition(i, time);
		if ((bodyPosition - selectPosition).length() < selectRadius)
			return i;
	}
	return {};
}

void Bodies::SetParent(size_t parent, size_t child)
{
	bodies[child].parent = parent;
	bodies[parent].childs.insert(child);

	ProcessTrajectoriesParent<PointRungeKutta>(bodies);
	ProcessTrajectoriesParent<PointVerlet>(bodies);
	ProcessTrajectoriesParent<PointEuler>(bodies);
}

void Bodies::ClearParent(size_t child)
{
	if (!bodies[child].parent)
		return;

	size_t parent = *bodies[child].parent;

	bodies[child].parent = {};
	bodies[parent].childs.erase(child);
}
