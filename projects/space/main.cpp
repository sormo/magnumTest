#include <Magnum2D.h>
#include <imgui.h>
#include "utils.h"
#include "trajectory.h"

using namespace Magnum2D;

float SimulationDt = 0.01f;
float SimulationSeconds = 10.0f;

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
		m.position = Utils::GetRandomPosition(-cameraHsize.x(), cameraHsize.x(), -cameraHsize.y(), cameraHsize.y());
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
		resimulate |= ImGui::SliderFloat("Gravity Constant", &GravitationalConstant, 0.01f, 1.0f);
		resimulate |= ImGui::SliderFloat("Gravity Threshold", &GravityThreshold, 0.5f, 10.0f);
		resimulate |= ImGui::SliderFloat("Simulation Dt", &SimulationDt, 0.001f, 1.0f);
		resimulate |= ImGui::SliderFloat("Simulation Seconds", &SimulationSeconds, 10.0f, 60.0f);
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

	bool isMouseGrabbed = false;

	if (trajectory.handleBurns())
	{
		isMouseGrabbed = true;
		simulate();
	}
	else if (isMousePressed())
	{
		auto positionMouse = getMousePositionWorld();
		for (size_t i = 0; i < trajectory.points.size(); i++)
		{
			if ((positionMouse - trajectory.points[i]).length() < 0.1f)
			{
				trajectory.addBurn(trajectory.times[i], { 0.0f,0.0f });
				simulate();
				isMouseGrabbed = true;
				break;
			}
		}
	}

	if (!isMouseGrabbed)
	{
		cameraControl();
	}

	drawCoordinateLines();

	for (const auto& m : massPoints)
	{
		drawCircle(m.position, 0.07f, Color1); 
		drawCircleOutline(m.position, m.mass * GravityThreshold, rgb(50,50,50));
	}

	trajectory.draw();
}
