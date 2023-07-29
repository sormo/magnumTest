#include <Magnum2D.h>
#include <imgui.h>
#include "utils.h"
#include "trajectory.h"

using namespace Magnum2D;

double SimulationDt = 0.01f;
double SimulationSeconds = 10.0f;

const col3 Color1 = rgb(66, 135, 245);

Trajectory trajectory;

std::vector<PointMass> massPoints;

void simulate()
{
	trajectory.simulate(massPoints, SimulationDt, SimulationSeconds, 60);
}

void setup()
{
	auto cameraHsize = getCameraSize() / 2.0f;

	for (int i = 0; i < 20; i++)
	{
		PointMass m;
		m.position = (vec2d)Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
		massPoints.push_back(std::move(m));
	}

	simulate();

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
			simulate();
	}

	//ImGui::ShowDemoWindow();

	ImGui::End();
}

void cameraControl()
{
	if (isMouseDown())
	{
		auto mouseDeltaWorld = convertWindowToWorldVector(getMouseDeltaWindow());
		auto cameraCenter = getCameraCenter();

		setCameraCenter(cameraCenter - mouseDeltaWorld);
	}

	auto scrollX = getMouseScroll();
	auto cameraSize = getCameraSize();
	float scrollY = (cameraSize.y() / cameraSize.x()) * scrollX;
	auto newCameraSize = getCameraSize() + vec2{ scrollX, scrollY };

	if (newCameraSize.x() < 0.1f || newCameraSize.y() < 0.1f)
		return;

	setCameraSize(newCameraSize);
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

void draw()
{
	gui();

	if (auto trajectoryResult = trajectory.update(); trajectoryResult == UpdateResult::Modified)
	{
		simulate();
	}
	else if (trajectoryResult == UpdateResult::None)
	{
		cameraControl();
	}

	drawCoordinateLines();

	for (const auto& m : massPoints)
	{
		drawCircle((vec2)m.position, 0.07f, Color1); 
		drawCircleOutline((vec2)m.position, m.mass * GravityThreshold, rgb(50,50,50));
	}

	trajectory.draw();
}
