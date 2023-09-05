#include "bodies.h"
#include "common.h"
#include "simulationBodies.h"

const float Bodies::ForceDrawFactor = 0.02f;

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
		vec2d vparent = bodies[index].parent ? bodies[*bodies[index].parent].initialVelocity : vec2d{};
		vec2d vp = bodies[index].initialPosition + (bodies[index].initialVelocity - vparent) * ForceDrawFactor;
		vec2d change = (vec2d)v - vp;

		bodies[index].SetInitialState(bodies[index].initialPosition, bodies[index].initialVelocity + change / ForceDrawFactor, bodies[index].mass);
		return v;
	};

	bodies.back().initialVector = vectorHandler.Push(from, to, nullptr, onFromChange, onToChange);

	return bodies.size() - 1;
}

void Bodies::SimulateClear(double time)
{
	std::set<size_t> indices;
	for (size_t i = 0; i < bodies.size(); i++)
		indices.insert(i);

	SimulateClearInternal(time, indices);
}

void Bodies::SimulateExtend(double time)
{
	auto euler = SimulationBodies<PointEuler>(bodies);
	euler.SimulateExtend(time, simulatedTime);
	auto verlet = SimulationBodies<PointVerlet>(bodies);
	verlet.SimulateExtend(time, simulatedTime);
	auto runge = SimulationBodies<PointRungeKutta>(bodies);
	runge.SimulateExtend(time, simulatedTime);

	simulatedTime += time;

	for (auto [child, parent] : runge.ComputeParents())
		SetParentSimulation(child, parent);

	euler.ProcessTrajectoriesParent();
	verlet.ProcessTrajectoriesParent();
	runge.ProcessTrajectoriesParent();

	runge.ComputeConics();
}

void Bodies::Resimulate(std::set<size_t> bodyIndices)
{
	if (simulatedTime == 0.0)
		return;

	SimulateClearInternal(simulatedTime, bodyIndices);
}

void Bodies::Resimulate(size_t body)
{
	std::set<size_t> indices;

	auto accs = SimulationBodies<PointRungeKutta>(bodies).ComputeInitialAccelerationsToBody(body);
	for (size_t i = 0; i < bodies.size(); i++)
	{
		if (accs[i].length() > 0.1)
			indices.insert(i);
	}
	indices.insert(body);

	Resimulate(indices);
}

void Bodies::SimulateClearInternal(double time, std::set<size_t> indices)
{
	auto euler = SimulationBodies<PointEuler>(bodies, indices);
	euler.SimulateClear(time);
	auto verlet = SimulationBodies<PointVerlet>(bodies, indices);
	verlet.SimulateClear(time);
	auto runge = SimulationBodies<PointRungeKutta>(bodies, indices);
	runge.SimulateClear(time);
	
	simulatedTime = time;

	for (auto [child, parent] : runge.ComputeParents())
		SetParentSimulation(child, parent);

	euler.ProcessTrajectoriesParent();
	verlet.ProcessTrajectoriesParent();
	runge.ProcessTrajectoriesParent();

	runge.ComputeConics();
}

void Bodies::DrawConic(const vec2& parentPosition, Body::Conic& conic, float width, const Magnum2D::col3& color)
{
	setTransform({ conic.position + parentPosition, conic.rotation });
	Common::DrawPolyline(conic.points, Common::GetZoomIndependentSize(width), color);
	setTransform({});
}

void Bodies::Draw(bool euler, bool verlet, bool rungeKutta, bool approximated, bool computed)
{
	vectorHandler.Draw();
	
	if (simulatedTime == 0.0)
		return;

	for (size_t i = 0; i < bodies.size(); i++)
	{
		auto& body = bodies[i];

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

		if (approximated)
			DrawConic(parentPosition, body.conicApproximatedFromPoints, 0.03f, rgb(80, 80, 80));

		// computed trajectory is wrong if parent is not correct
		if (body.parent && computed)
			DrawConic(parentPosition, body.conicComputedFromParent, 0.03f, !body.HasCorrectParent() ? rgb(80, 10, 10) : rgb(150, 80, 80));
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

void SetVelocityVectorSizeToParent(std::vector<Bodies::Body>& bodies, size_t body, VectorHandler& vectorHandler)
{
	vec2d parentVelocity = bodies[body].parent ? bodies[*bodies[body].parent].initialVelocity : vec2d{};
	vec2 to = (vec2)(bodies[body].initialPosition + (bodies[body].initialVelocity - parentVelocity) * Bodies::ForceDrawFactor);
	vectorHandler.SetTo(bodies[body].initialVector, to);
}

void Bodies::SetParentCommon(size_t child, std::optional<size_t> parent)
{
	if (bodies[child].parent)
		ClearParentInternal(child);

	if (parent)
		SetParentInternal(child, *parent);
	else
		ClearParentInternal(child);

	SetVelocityVectorSizeToParent(bodies, child, vectorHandler);
}

// Make sure that child is not within the subtree of body.
void ClearChildFromBodyRecursive(Bodies& bodies, Bodies::Body& body, size_t child)
{
	// If child is a child of body, clear it.
	if (body.childs.contains(child))
		bodies.SetParentUser(child, {});
	for (auto c : body.childs)
		ClearChildFromBodyRecursive(bodies, bodies.bodies[c], child);
}

void Bodies::SetParentUser(size_t child, std::optional<size_t> parent)
{
	// When setting the parent from user, we expect that velocity vector is
	// not grabbed. 
	// There are two possibilities, either user is setting the parent to expected
	// one or wrong one. In both cases proceed with change.
	if (bodies[child].parent == parent)
		return;

	// We may stack overflow if there will be a loop in the tree. This means that if
	// parent is actually a child of the child below we will recurse endlessly.
	// To fix this, we need to break the loop.
	// TODO This will call recursively SetParentUser, may be done a lot of work
	if (parent)
		ClearChildFromBodyRecursive(*this, bodies[child], *parent);

	SetParentCommon(child, parent);

	// When parent is set by user we need to update trajectories and conics for the child.
	SimulationBodies<PointRungeKutta>(bodies).ProcessTrajectoriesParentRecursive(child);
	SimulationBodies<PointVerlet>(bodies).ProcessTrajectoriesParentRecursive(child);
	SimulationBodies<PointEuler>(bodies).ProcessTrajectoriesParentRecursive(child);

	SimulationBodies<PointRungeKutta>(bodies).ComputeConicRecursive(child);
}

void Bodies::SetParentSimulation(size_t child, std::optional<size_t> parent)
{
	bodies[child].parentSimulation = parent;
	// When setting parent from simulation, we know this is correct parent.
	// Reason not setting it as current parent is that user has grabbed
	// child's vector and is modifying it. We keep the current parent in that case.
	if (vectorHandler.IsGrab(bodies[child].initialVector))
	{
		return;
	}

	SetParentCommon(child, parent);
}

void Bodies::SetParentInternal(size_t child, size_t parent)
{
	bodies[child].parent = parent;
	bodies[parent].childs.insert(child);
}

void Bodies::ClearParentInternal(size_t child)
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

size_t Bodies::GetBodyOfGrab(VectorHandler::Vector grab)
{
	for (size_t i = 0; i < bodies.size(); i++)
	{
		if (bodies[i].initialVector == grab)
			return i;
	}
	return (size_t)-1;
}
