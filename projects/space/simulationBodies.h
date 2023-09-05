#include <vector>
#include <map>
#include "bodies.h"


template<class T>
struct SimulationBodies
{
    SimulationBodies(std::vector<Bodies::Body>& bodies, std::set<size_t> indices)
        : bodies(bodies), indices(indices)
    {
    }

    SimulationBodies(std::vector<Bodies::Body>& bodies)
        : bodies(bodies), indices(indices)
    {
        for (size_t i = 0; i < bodies.size(); i++)
            indices.insert(i);
    }

    std::vector<Bodies::Body>& bodies;
    // indices to bodies vector, should represent bodies included in simulation
    std::set<size_t> indices;

    void SimulateClear(double time)
    {
        for (auto& index : indices)
            bodies[index].GetSimulation<T>().Clear();

        SimulateExtend(time, 0.0);
    }

    void SimulateExtend(double time, double simulatedTime)
    {
        std::vector<std::vector<BurnPtr>> burns(indices.size());
        std::vector<T> points;
        points.reserve(indices.size());

        std::vector<size_t> resultIndices;
        resultIndices.reserve(indices.size());

        for (auto& index : indices)
        {
            points.push_back(bodies[index].GetSimulation<T>().currentPoint);
            resultIndices.push_back(index);
        }

        auto newTrajectories = Simulation::Simulate(points, burns, SimulationDt, time, simulatedTime, TestBodies::TrajectoryPointCount);
        for (size_t i = 0; i < newTrajectories.size(); i++)
        {
            bodies[resultIndices[i]].GetSimulation<T>().currentPoint = std::move(points[i]);
            bodies[resultIndices[i]].GetSimulation<T>().trajectoryGlobal.extend(std::move(newTrajectories[i]), 0);
            bodies[resultIndices[i]].GetSimulation<T>().trajectoryParent.extend(std::move(newTrajectories[i]), 0);
        }
    }

    void ProcessTrajectoriesParentRecursive(size_t body)
    {
        auto& simulation = bodies[body].GetSimulation<T>();
        auto& trajectoryGlobal = simulation.trajectoryGlobal;
        auto& trajectoryParent = simulation.trajectoryParent;

        if (bodies[body].parent)
        {
            auto& trajectoryOfParent = bodies[*bodies[body].parent].GetSimulation<T>().trajectoryParent;
            trajectoryParent.clear();
            for (size_t i = 0; i < simulation.trajectoryGlobal.positions.size(); i++)
            {
                trajectoryParent.positions.push_back(trajectoryGlobal.positions[i] - trajectoryOfParent.positions[i]);
                trajectoryParent.velocities.push_back(trajectoryGlobal.velocities[i] - trajectoryOfParent.velocities[i]);
            }
        }
        else
        {
            trajectoryParent = trajectoryGlobal;
        }

        simulation.trajectoryParent.times = trajectoryGlobal.times;

        for (size_t child : bodies[body].childs)
        {
            if (indices.contains(child))
                ProcessTrajectoriesParentRecursive(child);
        }
    }

    void ProcessTrajectoriesParent(size_t parent)
    {
        for (auto child : bodies[parent].childs)
        {
            if (indices.contains(child))
                ProcessTrajectoriesParentRecursive(child);
        }
    }

    void ProcessTrajectoriesParent()
    {
        for (size_t i = 0; i < bodies.size(); i++)
        {
            if (bodies[i].parent)
                continue;
            // from the root
            if (indices.contains(i))
                bodies[i].GetSimulation<T>().trajectoryParent = bodies[i].GetSimulation<T>().trajectoryGlobal;

            for (auto child : bodies[i].childs)
            {
                if (indices.contains(child))
                    ProcessTrajectoriesParentRecursive(child);
            }
        }
    }

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
            //double semilatusRectum = pow(angularMomentum, 2) / (rm * G * ma * mb);
            double eccentricity = sqrt(1 + (2.0 * energy * pow(angularMomentum, 2)) / (rm * pow(G * ma * mb, 2)));

            E[i] = eccentricity;
        }

        return E;
    }

    std::map<size_t, std::optional<size_t>> ComputeParents()
    {
        std::map<size_t, std::optional<size_t>> result;

        // constant of allowed deviation of eccentricity for elliptical orbit
        static const double PeriodicOrbitFocusSigma = 1.0;

        for (auto child : indices)
        {
            using EccentricityParents = std::pair<double, size_t>;
            std::vector<EccentricityParents> values;

            for (auto parent : indices)
            {
                if (parent == child || bodies[parent].mass < bodies[child].mass)
                    continue;

                double value = Utils::GetMeanDeviation(GetEccentricities(bodies[child], bodies[parent]));
                values.push_back({ value, parent });
            }

            bool parentSet = false;

            if (!values.empty())
            {
                std::sort(std::begin(values), std::end(values));

                auto value = values[0].first;
                auto parent = values[0].second;

                if (values.size() > 1 && bodies[parent].isStar && values[1].first < PeriodicOrbitFocusSigma)
                {
                    value = values[1].first;
                    parent = values[1].second;
                }

                if (value < PeriodicOrbitFocusSigma)
                {
                    result.insert({ child, parent });
                    parentSet = true;
                }
            }

            if (!parentSet)
                result.insert({ child, {} });
        }

        return result;
    }

    Bodies::Body::Conic CreateConicFromApproximation(const ConicApproximation::Conic& conic)
    {
        Bodies::Body::Conic result;

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

    void ComputeConic(Bodies::Body& body)
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

    void ComputeConicRecursive(size_t body)
    {
        ComputeConic(bodies[body]);

        for (auto child : bodies[body].childs)
        {
            if (indices.contains(child))
                ComputeConicRecursive(child);
        }
    }

    void ComputeConics()
    {
        for (auto index : indices)
            ComputeConic(bodies[index]);
    }

    std::vector<vec2d> ComputeInitialAccelerationsToBody(size_t body)
    {
        std::vector<vec2d> result;

        for (auto index : indices)
        {
            if (index == body)
            {
                result.push_back({ 0.0, 0.0 });
                continue;
            }

            auto acc = bodies[body].GetSimulation<T>().initialPoint.attractForce(bodies[index].GetSimulation<T>().initialPoint.position, bodies[index].mass) / bodies[body].mass;
            result.push_back(acc);
        }

        return result;
    }
};
