#include <Magnum2D.h>
#include <imgui.h>

using namespace Magnum2D;

void setup()
{

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

void draw()
{
	gui();

	cameraControl();

	drawCircle({ 0,0 }, 1.0f, rgb(0, 0, 255));
}
