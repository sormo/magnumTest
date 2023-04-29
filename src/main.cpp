#include "Application2D.h"
#include "utils.h"
#include <imgui.h>
#include <algorithm>
#include <array>

const float RopeNodeDistance = 0.1f;
float RopeSimmulationDelta = 0.01f;
const int32_t RopeConstraintIterations = 50;
const app2d::vec2 Gravity = app2d::vec2(0.0f, -9.89f);

struct Rectangle
{
	static Rectangle fromCenterSize(app2d::vec2&& center, app2d::vec2&& size)
	{
		return { center, size, center - size / 2.0f, center + size / 2.0f };
	}

	app2d::vec2 center;
	app2d::vec2 size;

	app2d::vec2 min;
	app2d::vec2 max;

	bool isInside(const app2d::vec2& p)
	{
		return p.x() >= min.x() && p.x() <= max.x() && p.y() >= min.y() && p.y() <= max.y();
	}

	void setCenter(app2d::vec2&& c)
	{
		center = c;
		min = c - size / 2.0f;
		max = c + size / 2.0f;
	}
};

struct Line
{
	app2d::vec2 p1;
	app2d::vec2 p2;
};

void testWindow()
{
	static bool is_open = false;

	if (!ImGui::Begin("Test", &is_open))
	{
		ImGui::End();
		return;
	}

	//ImGui::ShowDemoWindow();

	ImGui::Text("Time %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::Text("Camera Size %.1f %.1f", app2d::getCameraSize().x(), app2d::getCameraSize().y());
	ImGui::Text("Camera Center %.1f %.1f", app2d::getCameraCenter().x(), app2d::getCameraCenter().y());

	auto mouse_position = app2d::getMousePositionWindow(), mouse_world_position = app2d::convertWindowToCamera(mouse_position);
	ImGui::Text("Mouse Window %.1f %.1f", mouse_position.x(), mouse_position.y());
	ImGui::Text("Mouse World %.1f %.1f", mouse_world_position.x(), mouse_world_position.y());

	ImGui::Text("Simmulation Delta"); ImGui::SameLine(); ImGui::SliderFloat("##", & RopeSimmulationDelta, 0.001f, 0.015f);
	//ImGui::SliderFloat("Simmulation Delta", &RopeSimmulationDelta, 0.05f, 0.15f);

	ImGui::End();
}

struct RopeNode
{
	RopeNode(app2d::vec2& p) : position(p), positionOld(p) {}
	app2d::vec2& position;
	app2d::vec2 positionOld;
    float mass = 1.0f;
};

struct Rope
{
	std::vector<app2d::vec2> positions;
	std::vector<RopeNode> nodes;
};

Rope createRope(app2d::vec2 p1, app2d::vec2 p2)
{
	Rope rope;

	rope.positions = utils::generatePoints(p1, p2, RopeNodeDistance);
	for (auto& p : rope.positions)
		rope.nodes.emplace_back(p);

	return rope;
}

void drawRope(Rope& rope)
{
	for (const auto& p : rope.positions)
		app2d::drawCircle(p, 0.05f, app2d::rgb(0, 128, 255));
}

// ---

Rope g_rope;

std::vector<Rectangle> g_rectangles;
std::vector<Line> g_lines;


void applyCollisions(app2d::vec2& from, app2d::vec2& to)
{
	for (auto& rect : g_rectangles)
	{
		// if "from" is inside of rectangle, move it to closest edge
		if (rect.isInside(from))
		{
			auto closestPoint = utils::getClosestPointOnEdge(from, rect.min, rect.max);

			from = closestPoint;
			to = closestPoint;
		}

		if (auto p = utils::aabbRaycast(from, to, rect.min, rect.max))
		{
			to = *p;
			break;
		}
	}
}

void simmulateStep(Rope& rope)
{
	for (auto& node : rope.nodes)
	{
		if (node.mass == 0.0f)
			continue;

		auto move = node.position - node.positionOld;

		node.positionOld = node.position;
		node.position += move + RopeSimmulationDelta * RopeSimmulationDelta * Gravity;

		applyCollisions(node.positionOld, node.position);
	}
}

void applyConstraints(Rope& rope)
{
	for (size_t i = 0; i < rope.nodes.size() - 1; i++)
	{
		RopeNode& node1 = rope.nodes[i];
		RopeNode& node2 = rope.nodes[i + 1];

        float im1 = node1.mass == 0.0f ? 0.0f : 1.0f / node1.mass;
        float im2 = node2.mass == 0.0f ? 0.0f : 1.0f / node2.mass;
        float mult1 = im1 / (im1 + im2), mult2 = im2 / (im1 + im2);
        
		app2d::vec2 diff = node1.position - node2.position;
        
		float dist = diff.length();
        float difference = 0.0f;
        if (dist > 0.0f)
            difference = (dist - RopeNodeDistance) / dist;

        diff *= 0.5f * difference;

        node1.position -= diff * mult1;
        node2.position += diff * mult2;
	}
}

void setup()
{
	app2d::setCameraCenter({ 0.0f, 0.0f });
	app2d::setCameraSize({ 32.0f, 24.0f });

	g_rope = createRope({ 0.0f, 10.0f }, { -14.0f, 4.0f });
	g_rope.nodes[0].mass = 0.0f;

	g_rectangles.push_back(Rectangle::fromCenterSize({0.0f, 0.0f}, { 4.0f, 2.0f }));
}

std::optional<app2d::vec2> linePoint1;

void lineAdd()
{
	static app2d::vec2 mousePressed;

	if (app2d::isMousePressed())
	{
		mousePressed = app2d::getMousePositionCamera();
	}
	else if (app2d::isMouseDown())
	{
		app2d::drawPolyline({ mousePressed, app2d::getMousePositionCamera() }, { 10,255,20 });
	}
	else if (app2d::isMouseReleased())
	{
		g_lines.emplace_back(mousePressed, app2d::getMousePositionCamera());
	}
}

void moveRect()
{
	static std::optional<Rectangle*> grabbedRect;
	static app2d::vec2 grabOffset;

	if (app2d::isMousePressed())
	{
		auto position = app2d::getMousePositionCamera();

		for (auto& r : g_rectangles)
		{
			if (r.isInside(position))
			{
				grabbedRect = &r;
				grabOffset = position - r.center;
				break;
			}
		}
	}
	else if (app2d::isMouseReleased())
	{
		grabbedRect.reset();
	}
	else if (grabbedRect)
	{
		auto position = app2d::getMousePositionCamera();

		(*grabbedRect)->setCenter(position - grabOffset);
	}
}

void draw()
{
	ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
	testWindow();

	//if (app2d::isKeyDown('a') || app2d::isKeyPressed('s'))
	{
		simmulateStep(g_rope);
		for (int32_t i = 0; i < RopeConstraintIterations; i++)
			applyConstraints(g_rope);
	}

	drawRope(g_rope);
	
	for (auto& l : g_lines)
	{
		app2d::drawPolyline({ l.p1, l.p2 }, app2d::rgb(10, 255, 20));

		for (auto& r : g_rectangles)
			if (auto intersection = utils::aabbRaycast(l.p1, l.p2, r.min, r.max))
				app2d::drawCircle(*intersection, 0.1f, app2d::rgb(255, 0, 0));
	}

	for (auto& r : g_rectangles)
		app2d::drawRectangle(r.center, r.size.x(), r.size.y(), app2d::rgb(50, 50, 50));

	//lineAdd();
	moveRect();
}


