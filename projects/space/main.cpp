#include <Magnum2D.h>
#include <imgui.h>
#include "utils.h"
#include "camera.h"
#include "trajectory.h"

using namespace Magnum2D;

Camera camera;

double SimulationDt = 0.01f;
double SimulationSeconds = 10.0f;

const col3 Color1 = rgb(66, 135, 245);

std::vector<TrajectoryPtr> trajectories;
std::vector<PointMass> massPoints;
std::optional<size_t> hoverTimepoint;
std::optional<size_t> selectTimepoint;

Trajectory* currentTrajectory = nullptr;

void simulate(Trajectory& t)
{
	t.simulate(massPoints, SimulationDt, SimulationSeconds, 60);
}

void setup()
{
	camera.Setup();

	auto cameraHsize = getCameraSize() / 2.0f;

	for (int i = 0; i < 20; i++)
	{
		PointMass m;
		m.position = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
		massPoints.push_back(std::move(m));
	}

	for (int i = 0; i < 2; i++)
	{
		vec2d initPos = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
		trajectories.emplace_back(std::make_unique<Trajectory>(initPos));
	}

	for (auto& t : trajectories)
		simulate(*t);

	//pointEuler.position = vec2(5.0f, 0.0f);
	//pointEuler.initializeCircularOrbit({ 0,0 }, centerMass);
	//pointVerlet.position = vec2(-5.0f, 0.0f);
	//pointVerlet.initializeCircularOrbit({ 0,0 }, centerMass);
}

void gui()
{
	static bool is_open = false;

	ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Test", &is_open))
	{
		ImGui::End();
		return;
	}

	ImGui::Text("Time %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::Text("Delta time %.3f ms", getDeltaTimeMs());
	ImGui::Text("Time %.3f s", getTimeMs() / 1000.0f);
	if (ImGui::CollapsingHeader("Window"))
	{
		ImGui::Text("Camera Size %.1f %.1f", getCameraSize().x(), getCameraSize().y());
		ImGui::Text("Camera Center %.1f %.1f", getCameraCenter().x(), getCameraCenter().y());
		ImGui::Text("Canvas Size %.1f %.1f", getWindowSize().x(), getWindowSize().y());

		auto mousePosition = getMousePositionWindow();
		auto mousePositionWorld = convertWindowToWorld(mousePosition);
		auto mouseDelta = getMouseDeltaWindow();
		ImGui::Text("Mouse Window %.1f %.1f", mousePosition.x(), mousePosition.y());
		ImGui::Text("Mouse World %.1f %.1f", mousePositionWorld.x(), mousePositionWorld.y());
		ImGui::Text("Mouse Window Delta %.1f %.1f", mouseDelta.x(), mouseDelta.y());
		ImGui::Text("Mouse Scroll %.1f", getMouseScroll());
	}

	if (ImGui::CollapsingHeader("Simulation"))
	{
		bool resimulate = false;

		static float GravitationalConstantF = GravitationalConstant, GravityThresholdF = GravityThreshold, SimulationDtF = SimulationDt, SimulationSecondsF = SimulationSeconds;
		auto SliderDouble = [&](const char* name, double& target, float &tmp, double min, double max)
		{
			if (ImGui::SliderFloat(name, &tmp, (float)min, (float)max))
			{
				target = tmp;
				resimulate = true;
			}
		};

		SliderDouble("Gravity Constant", GravitationalConstant, GravitationalConstantF, 0.01, 1.0);
		SliderDouble("Gravity Threshold", GravityThreshold, GravityThresholdF, 0.5, 10.0);
		SliderDouble("Simulation Dt", SimulationDt, SimulationDtF, 0.001, 0.1);
		SliderDouble("Simulation Seconds", SimulationSeconds, SimulationSecondsF, 10.0, 60.0);

		if (resimulate)
		{
			for (auto& t : trajectories)
				simulate(*t);
		}
	}

	//ImGui::ShowDemoWindow();

	ImGui::End();
}

void drawCoordinateLines()
{
	vec2 bottomLeft = getCameraCenter() - getCameraSize() / 2.0f;
	vec2 upperRight = getCameraCenter() + getCameraSize() / 2.0f;

	std::vector<vec2> points;
	for (float x = std::ceil(bottomLeft.x()); x < std::ceil(upperRight.x()); x += 1.0f)
	{
		points.push_back({ x, bottomLeft.y() });
		points.push_back({ x, upperRight.y() });
	}
	for (float y = std::ceil(bottomLeft.y()); y < std::ceil(upperRight.y()); y += 1.0f)
	{
		points.push_back({ bottomLeft.x(), y });
		points.push_back({ upperRight.x(), y });
	}

	drawLines(points, rgb(50, 50, 50));
}

void updateTrajectory(Trajectory* trajectory)
{
	auto trajectoryResult = trajectory->update();

	switch (trajectoryResult)
	{
	case UpdateResult::Modified:
		simulate(*trajectory);
	case UpdateResult::InputGrab:
		currentTrajectory = trajectory;
		break;
	default:
		currentTrajectory = nullptr;
	}
}

void draw()
{
	gui();

	if (currentTrajectory)
	{
		updateTrajectory(currentTrajectory);
	}

	if (!currentTrajectory)
	{
		for (auto& t : trajectories)
		{
			updateTrajectory(t.get());

			if (currentTrajectory)
				break;
		}
	}

	hoverTimepoint.reset();
	for (auto& t : trajectories)
	{
		if (t->burnsHandler.burnAddIndex)
		{
			hoverTimepoint = t->burnsHandler.burnAddIndex;
			if (isMousePressed(Mouse::right))
				selectTimepoint = hoverTimepoint;
		}
	}

	camera.Update(currentTrajectory == nullptr);

	drawCoordinateLines();

	for (const auto& m : massPoints)
	{
		drawCircle((vec2)m.position, 0.07f, Color1); 
		drawCircleOutline((vec2)m.position, m.mass * GravityThreshold, rgb(50,50,50));
	}

	// draw timepoints
	for (auto& t : trajectories)
	{
		if (hoverTimepoint)
			drawCircle(t->points[*hoverTimepoint], Common::GetZoomIndependentSize(0.06f), rgb(50, 255, 50));
		if (selectTimepoint)
			drawCircle(t->points[*selectTimepoint], Common::GetZoomIndependentSize(0.06f), rgb(50, 50, 255));
	}

	// draw trajectories
	for (auto& t : trajectories)
		t->draw();
}
