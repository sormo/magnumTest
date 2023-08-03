#include <Magnum2D.h>
#include <imgui.h>
#include "utils.h"
#include "camera.h"
#include "testMassPoint.h"
#include "testBodies.h"

using namespace Magnum2D;

Camera camera;

double SimulationDt = 0.01f;
extern double TestMassPoint::SimulationSeconds = 10.0f;
extern float TestBodies::SimulatedSeconds;
extern bool TestBodies::IsPlaying;

enum TestType : int32_t
{
	MassPoints,
	Bodies

};
TestType CurrentTest = TestType::Bodies;

void setup()
{
	camera.Setup();

	TestMassPoint::Setup();
	TestBodies::Setup();

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

	if (ImGui::CollapsingHeader("Performance"))
	{
		ImGui::Text("Time"); ImGui::SameLine(100); ImGui::Text("%.3f ms/frame(% .1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Text("Delta time"); ImGui::SameLine(100); ImGui::Text("%.3f ms", getDeltaTimeMs());
		ImGui::Text("Time"); ImGui::SameLine(100); ImGui::Text("%.3f s", getTimeMs() / 1000.0f);
	}

	if (ImGui::CollapsingHeader("Window"))
	{
		ImGui::Text("Camera Size"); ImGui::SameLine(150); ImGui::Text("%.1f %.1f", getCameraSize().x(), getCameraSize().y());
		ImGui::Text("Camera Center"); ImGui::SameLine(150); ImGui::Text("%.1f %.1f", getCameraCenter().x(), getCameraCenter().y());
		ImGui::Text("Canvas Size"); ImGui::SameLine(150); ImGui::Text("%.1f %.1f", getWindowSize().x(), getWindowSize().y());

		auto mousePosition = getMousePositionWindow();
		auto mousePositionWorld = convertWindowToWorld(mousePosition);
		auto mouseDelta = getMouseDeltaWindow();
		ImGui::Text("Mouse Window"); ImGui::SameLine(150); ImGui::Text("%.1f %.1f", mousePosition.x(), mousePosition.y());
		ImGui::Text("Mouse World"); ImGui::SameLine(150); ImGui::Text("%.1f %.1f", mousePositionWorld.x(), mousePositionWorld.y());
		ImGui::Text("Mouse Window Delta"); ImGui::SameLine(150); ImGui::Text("%.1f %.1f", mouseDelta.x(), mouseDelta.y());
		ImGui::Text("Mouse Scroll"); ImGui::SameLine(150); ImGui::Text("%.1f", getMouseScroll());
	}

	if (ImGui::CollapsingHeader("Simulation"))
	{
		bool resimulate = false;

		static float GravitationalConstantF = GravitationalConstant, GravityThresholdF = GravityThreshold, SimulationDtF = SimulationDt, SimulationSecondsF = TestMassPoint::SimulationSeconds;
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

		if (resimulate)
		{
			TestMassPoint::Simulate();
			TestBodies::Simulate();
		}
	}

	if (ImGui::CollapsingHeader("Tests"))
	{
		ImGui::Combo("Test", (int*)&CurrentTest, "MassPoints\0Bodies\0");

		if (ImGui::BeginTabBar("##tabs", ImGuiTabBarFlags_None))
		{
			if (ImGui::BeginTabItem("MassPoints"))
			{
				float seconds = TestMassPoint::SimulationSeconds;
				if (ImGui::SliderFloat("SimulationSeconds", &seconds, 10.0f, 60.0f))
				{
					TestMassPoint::SimulationSeconds = seconds;
					TestMassPoint::Simulate();
				}
				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Bodies"))
			{
				ImGui::Text("Simulated Seconds: %.1f", TestBodies::SimulatedSeconds);
				ImGui::SameLine();
				if (ImGui::Button("Simulate"))
					TestBodies::Simulate(10.0); 
				ImGui::SameLine();
				ImGui::Checkbox("Playing", &TestBodies::IsPlaying);

				ImGui::SliderFloat("Current Time", &TestBodies::CurrentTime, 0.0f, TestBodies::SimulatedSeconds);

				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
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

void draw()
{
	drawCoordinateLines();
	gui();

	bool allowCameraMove = true;

	switch (CurrentTest)
	{
	case TestType::MassPoints:
		allowCameraMove = !TestMassPoint::Update();
		break;
	case TestType::Bodies:
		allowCameraMove = !TestBodies::Update();
		break;
	}

	camera.Update(allowCameraMove);

}
