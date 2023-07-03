#include <Magnum2D.h>
#include "application.h"
#include "globals.h"
#include "utils.h"
#include <imgui.h>
#include <algorithm>
#include <array>

void testWindow()
{
	static bool is_open = false;

	ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Test", &is_open))
	{
		ImGui::End();
		return;
	}

	//ImGui::ShowDemoWindow();

	ImGui::Text("Time %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	if (ImGui::CollapsingHeader("Window"))
	{
		ImGui::Text("Camera Size %.1f %.1f", Magnum2D::getCameraSize().x(), Magnum2D::getCameraSize().y());
		ImGui::Text("Camera Center %.1f %.1f", Magnum2D::getCameraCenter().x(), Magnum2D::getCameraCenter().y());
		ImGui::Text("Canvas Size %.1f %.1f", Magnum2D::getWindowSize().x(), Magnum2D::getWindowSize().y());

		auto mousePosition = Magnum2D::getMousePositionWindow();
		auto moustPositionWorld = Magnum2D::convertWindowToCamera(mousePosition);
		ImGui::Text("Mouse Window %.1f %.1f", mousePosition.x(), mousePosition.y());
		ImGui::Text("Mouse World %.1f %.1f", moustPositionWorld.x(), moustPositionWorld.y());
	}

	if (ImGui::CollapsingHeader("Simmulation"))
	{
		ImGui::SliderFloat("Time Delta", &Globals::RopeSimmulationDelta, 0.001f, 0.03f);
		ImGui::SliderFloat("Gravity", &Globals::Gravity.y(), -15.0f, -0.1f);
		ImGui::SliderInt("Constraint Iterations", &Globals::RopeConstraintIterations, 1, 40);
	}

	if (ImGui::CollapsingHeader("Rope"))
	{
		if (ImGui::SliderFloat("Node Distance", &Globals::RopeNodeDistance, 0.1f, 2.0f))
		{
			g_app->SetupRope();
		}

		ImGui::Combo("Interaction", (int*)&Globals::Interaction, "Cut\0Grab\0Attract\0");
	}

	ImGui::End();
}

void setup()
{
	Magnum2D::setCameraCenter({ 0.0f, 0.0f });
	Magnum2D::setCameraSize({ 32.0f, 24.0f });

	Globals::Interaction = Globals::InteractionGrab;

	g_app = std::make_unique<Application>();
}

bool moveMovable()
{
	static std::optional<Movable*> grabbedMovable;
	static Magnum2D::vec2 grabOffset;

	if (Magnum2D::isMousePressed())
	{
		auto position = Magnum2D::getMousePositionCamera();

		for (auto& r : g_app->rectangles)
		{
			if (r.IsInside(position))
			{
				grabbedMovable = &r;
				grabOffset = position - r.center;
				
				return true;
			}
		}

		for (auto& r : g_app->circles)
		{
			if (r.IsInside(position))
			{
				grabbedMovable = &r;
				grabOffset = position - r.center;

				return true;
			}
		}
	}
	else if (Magnum2D::isMouseReleased() && grabbedMovable)
	{
		grabbedMovable.reset();

		return true;
	}
	else if (grabbedMovable)
	{
		auto position = Magnum2D::getMousePositionCamera();

		(*grabbedMovable)->SetCenter(position - grabOffset);

		return true;
	}

	return false;
}

void cutRope()
{
	static std::optional<Magnum2D::vec2> cutPosition;

	if (Magnum2D::isMousePressed())
	{
		cutPosition = Magnum2D::getMousePositionCamera();
	}
	else if (Magnum2D::isMouseReleased())
	{
		cutPosition.reset();
	}
	else if (cutPosition)
	{
		auto position = Magnum2D::getMousePositionCamera();

		for (auto& node : g_app->rope.nodes)
		{
			for (auto it = node.childs.begin(); it != node.childs.end();)
			{
				if (utils::doLineSegmentsIntersect(*cutPosition, position, node.position, ((Rope::RopeNode&)(*it)).position))
					it = node.childs.erase(it);
				else
					it++;
			}
		}
		cutPosition = position;
	}
}

Rope::RopeNode* getClosestNode(const Magnum2D::vec2& point)
{
	float distance = std::numeric_limits<float>::max();
	Rope::RopeNode* result = nullptr;

	for (auto& node : g_app->rope.nodes)
	{
		auto nodeDistance = (node.position - point).length();
		if (nodeDistance < distance)
		{
			distance = nodeDistance;
			result = &node;
		}
	}

	return result;
}

void grabRope()
{
	static std::optional<Rope::RopeNode*> grabbedNode;

	if (Magnum2D::isMousePressed())
	{
		grabbedNode = getClosestNode(Magnum2D::getMousePositionCamera());
		(*grabbedNode)->mass = 0.0f;
	}
	else if (Magnum2D::isMouseReleased())
	{
		(*grabbedNode)->mass = 1.0f;
		grabbedNode = std::nullopt;
	}

	if (grabbedNode)
	{
		(*grabbedNode)->position = Magnum2D::getMousePositionCamera();
		(*grabbedNode)->positionOld = (*grabbedNode)->position;
	}
}

void draw()
{
	testWindow();

	//if (Magnum2D::isKeyDown('a') || Magnum2D::isKeyPressed('s'))
	{
		g_app->rope.SimmulateStep();
		for (int32_t i = 0; i < Globals::RopeConstraintIterations; i++)
			g_app->rope.ApplyConstraints();
	}

	for (auto& r : g_app->rectangles)
		Magnum2D::drawRectangle(r.center, r.size.x(), r.size.y(), Magnum2D::rgb(50, 50, 50));

	for (auto& c : g_app->circles)
		Magnum2D::drawCircle(c.center, c.radius, Magnum2D::rgb(50, 50, 50));

	g_app->rope.Draw();

	if (!moveMovable())
	{
		switch (Globals::Interaction)
		{
		case Globals::InteractionCut:
			cutRope();
			break;
		case Globals::InteractionGrab:
			grabRope();
			break;
		}
	}
}


