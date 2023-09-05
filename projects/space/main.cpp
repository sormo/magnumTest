#include <Magnum2D.h>
#include <imgui.h>
#include "utils.h"
#include "camera.h"
#include "testMassPoint.h"
#include "testBodies.h"

using namespace Magnum2D;

Camera camera;

double SimulationDt = 0.01f;

enum class TestType : int32_t
{
	TestMassPoints,
	TestBodies

};
TestType CurrentTest = TestType::TestBodies;

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
		ImGui::Checkbox("Antialised Lines", &Common::IsAntialisedLinesEnabled);
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
		bool refreshEffectiveRadius = false;

		//float GravityThresholdF = GravityThreshold;
		float GravitationalConstantF = GravitationalConstant, SimulationDtF = SimulationDt / Unit::Hour;
		auto SliderDouble = [&](const char* name, double& target, float &tmp, double min, double max, double factor)
		{
			if (ImGui::SliderFloat(name, &tmp, (float)min, (float)max))
			{
				target = tmp * factor;
				return true;
			}
			return false;
		};

		resimulate |= refreshEffectiveRadius |= SliderDouble("Gravity Constant", GravitationalConstant, GravitationalConstantF, 0.1, 100.0, 1.0); ImGui::SameLine(); ImGui::Text("[]");
		//resimulate |= refreshEffectiveRadius |= SliderDouble("Gravity Threshold", GravityThreshold, GravityThresholdF, 0.0005, 0.01, 1.0); ImGui::SameLine(); ImGui::Text("[m/s^2]");
		resimulate |= SliderDouble("Simulation Dt", SimulationDt, SimulationDtF, 0.1, 24.0, Unit::Hour); ImGui::SameLine(); ImGui::Text("[hours]");

		if (refreshEffectiveRadius)
		{
			TestMassPoint::RefreshEffectiveRadius();
			TestBodies::RefreshEffectiveRadius();
		}

		if (resimulate)
		{
			//TestMassPoint::Simulate();
			TestBodies::Resimulate();
		}
	}

	if (ImGui::CollapsingHeader("Tests"))
	{
		ImGui::Combo("Test", (int*)&CurrentTest, "MassPoints\0Bodies\0");

		switch (CurrentTest)
		{
		case TestType::TestMassPoints:
			TestMassPoint::Gui();
			break;
		case TestType::TestBodies:
			TestBodies::Gui();
			break;
		}
	}

	//ImGui::ShowDemoWindow();
	ImGui::End();
}

void drawGui()
{
	gui();
}

void draw()
{
	//setTransform({ {4.0f, 5.0}, 0.0f });
	//drawCircle({ 0.0f, 1.0f }, 0.5f, rgb(10, 10, 200));
	//drawCircleOutline2({ 0.0f, 1.0f }, 0.5f, 0.3f, rgb(10, 10, 200));
	//drawRectangle({ 0.0f, 1.0f }, 1.0f, 2.0f, rgb(10, 10, 200));
	//drawPolygon({ {0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f} }, rgb(10, 10, 200));
	//std::vector<vec2> points = { {0.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 2.0f} };
	//drawPolyline(points, rgb(10, 10, 200));
	//drawPolyline2(points, 0.3f, rgb(10, 10, 200));
	//setTransform({});

	camera.Draw();

	bool allowCameraMove = true;

	switch (CurrentTest)
	{
	case TestType::TestMassPoints:
		allowCameraMove = !TestMassPoint::Update();
		break;
	case TestType::TestBodies:
		allowCameraMove = !TestBodies::Update();
		break;
	}

	camera.Update(allowCameraMove);
}
