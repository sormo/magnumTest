#include "bodies.h"
#include "common.h"

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

	ComputeParents();
	ComputeConics();
}

void Bodies::SimulateExtend(double time)
{
	::Simulate<PointEuler>(time, simulatedTime, bodies);
	::Simulate<PointVerlet>(time, simulatedTime, bodies);
	::Simulate<PointRungeKutta>(time, simulatedTime, bodies);

	simulatedTime += time;

	ComputeParents();
	ComputeConics();
}

void Bodies::DrawConic(const vec2& parentPosition, Body::Conic& conic, float width, const Magnum2D::col3& color)
{
	setTransform({ conic.position + parentPosition, conic.rotation });
	Common::DrawPolyline(conic.points, Common::GetZoomIndependentSize(width), color);
	setTransform({});
}

void Bodies::Draw(bool euler, bool verlet, bool rungeKutta)
{
	vectorHandler.Draw();
	
	if (simulatedTime == 0.0)
		return;

	for (auto& body : bodies)
	{
		auto parentPosition = body.parent ? GetCurrentPosition(*body.parent) : vec2{};

		if (body.parent)
		{
			setTransform({ parentPosition, 0.0f });
		}

		if (euler)
			body.GetSimulation<PointEuler>().trajectoryParent.draw(0, body.GetSimulation<PointEuler>().currentIndex, rgb(50, 50, 50));
		if (verlet)
			body.GetSimulation<PointVerlet>().trajectoryParent.draw(0, body.GetSimulation<PointVerlet>().currentIndex, rgb(100, 100, 100));
		if (rungeKutta)
			body.GetSimulation<PointRungeKutta>().trajectoryParent.draw(0, body.GetSimulation<PointRungeKutta>().currentIndex, body.color);

		setTransform({});

		DrawConic(parentPosition, body.conicApproximatedFromPoints, 0.03f, rgb(80, 80, 80));

		if (body.parent)
			DrawConic(parentPosition, body.conicComputedFromParent, 0.03f, rgb(150, 80, 80));
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

float Bodies::GetCurrentDistanceToParent(size_t index)
{
	if (!bodies[index].parent)
		return {};
	auto& sim = bodies[index].GetSimulation<PointRungeKutta>();
	return sim.trajectoryParent.positions[sim.currentIndex].length();
}

template<class T>
std::vector<double> GetEccentricities(Bodies::Body& a, Bodies::Body& b)
{
	Trajectory& traja = a.GetSimulation<T>().trajectoryGlobal;
	Trajectory& trajb = b.GetSimulation<T>().trajectoryGlobal;

	const size_t n = traja.positions.size();
	double ma = a.mass, mb = b.mass;
	double rm = (ma * mb) / (ma + mb);

	double G = GravitationalConstant;

	std::vector<double> E(n);
	for (size_t i = 0; i < n; ++i)
	{
		vec2d velocity = (vec2d)traja.velocities[i] - (vec2d)trajb.velocities[i];

		double v = velocity.length();
		double vt = Utils::RotateVector(velocity, Utils::GetAngle(velocity)).y();
		double r = (traja.positions[i] - trajb.positions[i]).length();
		double energy = 0.5 * rm * v * v - (G * ma * mb) / r;

		double angularMomentum = rm * r * vt; // use tangential part of velocity
		double semilatusRectum = pow(angularMomentum, 2) / (rm * G * ma * mb);
		double eccentricity = sqrt(1 + (2.0 * energy * pow(angularMomentum, 2)) / (rm * pow(G * ma * mb, 2)));

		E[i] = eccentricity;
	}

	return E;
}

void Bodies::ComputeParents()
{
	// constant of allowed deviation of eccentricity for elliptical orbit
	static const double PeriodicOrbitFocusSigma = 1.0;

	for (size_t child = 0; child < bodies.size(); ++child)
	{
		using EccentricityParentChildPair = std::pair<double, std::pair<size_t, size_t>>;
		std::vector<EccentricityParentChildPair> values;

		for (size_t parent = 0; parent < bodies.size(); ++parent)
		{
			if (parent == child || bodies[parent].mass < bodies[child].mass)
				continue;

			double value = Utils::GetMeanDeviation(GetEccentricities<PointRungeKutta>(bodies[child], bodies[parent]));
			values.push_back({ value, { parent, child } });
		}

		if (!values.empty())
		{
			std::sort(std::begin(values), std::end(values));

			auto value = values[0].first;
			auto pair = values[0].second;
			
			if (values.size() > 1 && bodies[pair.first].isStar && values[1].first < PeriodicOrbitFocusSigma)
			{
				value = values[1].first;
				pair = values[1].second;
			}

			if (value < PeriodicOrbitFocusSigma)
				SetParent(pair.first, pair.second);
		}
	}
}

Bodies::Body::Conic Bodies::CreateConicFromApproximation(const ConicApproximation::Conic& conic)
{
	Body::Conic result;

	switch (conic.GetType())
	{
	case ConicApproximation::Conic::Type::ellipse:
	{
		auto ellipse = conic.GetEllipse();
		result.points = Utils::ConvertToFloat(Utils::GenerateEllipsePoints(ellipse.radius.x(), ellipse.radius.y()));
		// TODO we are using polyline, possibly better solution will be scaled circle
		result.points.push_back(result.points[0]);
		result.position = (vec2)ellipse.position;
		result.rotation = (float)ellipse.angle;
	}
	break;
	case ConicApproximation::Conic::Type::hyperbola:
	{
		auto hyperbola = conic.GetHyperbola();
		result.points = Utils::ConvertToFloat(Utils::GenerateHyperbolaPoints(hyperbola.radius.x(), hyperbola.radius.y()));
		result.position = (vec2)hyperbola.position;
		result.rotation = (float)hyperbola.angle;
	}
	break;
	}

	return result;
}

void Bodies::ComputeConics()
{
	for (auto& body : bodies)
	{
		// TODO
		std::vector<vec2d> positions(body.GetSimulation<PointRungeKutta>().trajectoryParent.positions.size());
		for (size_t i = 0; i < positions.size(); i++)
			positions[i] = (vec2d)body.GetSimulation<PointRungeKutta>().trajectoryParent.positions[i];

		if (!positions.empty())
		{
			body.conicApproximatedFromPoints = CreateConicFromApproximation(ConicApproximation::ApproximateConic(positions));
		}
		if (body.parent)
		{
			auto initialPositionParentRelative = body.initialPosition - bodies[*body.parent].initialPosition;
			auto initialVelocityParentRelative = body.initialVelocity - bodies[*body.parent].initialVelocity;

			auto computedConic = ConicApproximation::ComputeConic(bodies[*body.parent].mass, body.mass, initialPositionParentRelative, initialVelocityParentRelative);
			body.conicComputedFromParent = CreateConicFromApproximation(computedConic);
		}
	}
}
