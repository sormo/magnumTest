#include "Application2D.h"
#include <imgui.h>

void setup()
{
	Application2D::setCameraCenter({ 0.0f, 0.0f });
	Application2D::setCameraSize({ 32.0f, 24.0f });
}

void test_window()
{
	static bool is_open = false;

	if (!ImGui::Begin("Test", &is_open))
	{
		ImGui::End();
		return;
	}

	ImGui::Text("Time %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::Text("Camera Size %.1f %.1f", Application2D::getCameraSize().x(), Application2D::getCameraSize().y());
	ImGui::Text("Camera Center %.1f %.1f", Application2D::getCameraCenter().x(), Application2D::getCameraCenter().y());

	auto mouse_position = Application2D::getMousePositionWindow(), mouse_world_position = Application2D::convertWindowToCamera(mouse_position);
	ImGui::Text("Mouse Window %.1f %.1f", mouse_position.x(), mouse_position.y());
	ImGui::Text("Mouse World %.1f %.1f", mouse_world_position.x(), mouse_world_position.y());

	ImGui::End();
}

void draw()
{
	ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
	test_window();

	//Application2D::setCameraCenter({ 5.0f,5.0f });
	Application2D::drawCircle({ 0.0f, 0.0f }, 1.0f, Application2D::rgb(255, 255, 255));
	Application2D::drawCircleOutline({ 5.0f, 0.0f }, 0.5f, Application2D::rgb(0, 128, 255));

	Application2D::drawRectangle({ 0.0f, 5.0f }, 1.0f, 1.0f, Application2D::rgb(0, 128, 255));
	Application2D::drawRectangle({ 0.0f, 10.0f }, 1.0f, 2.0f, Application2D::rgb(0, 128, 255));
	Application2D::drawRectangle({ 0.0f, 15.0f }, 5.0f * 0.0174533f, 0.5f, 1.0f, Application2D::rgb(0, 128, 255));

	//Application2D::drawPolygon({ {0.0f, -5.0f}, {5.0f, -5.0f}, {12.0f, 0.0f} }, Application2D::rgb(0, 128, 255));
	Application2D::drawPolyline({ {0.0f, -5.0f}, {5.0f, -5.0f}, {12.0f, 0.0f} }, Application2D::rgb(0, 128, 255));
}
