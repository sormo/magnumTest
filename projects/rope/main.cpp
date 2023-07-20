#include <Magnum2D.h>
#include "utils.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <queue>
#include <set>

float RopeNodeDistance = 0.5f;
float RopeSimmulationDelta = 0.01f;
int32_t RopeConstraintIterations = 2;
Magnum2D::vec2 Gravity = Magnum2D::vec2(0.0f, -9.89f);

enum 
{
	InteractionCut = 0,
	InteractionGrab = 1,
	InteractionAttract = 2,

} Interaction;

void setupRope();

struct Rectangle
{
	static Rectangle fromCenterSize(const Magnum2D::vec2& center, const Magnum2D::vec2& size)
	{
		return { center, size, center - size / 2.0f, center + size / 2.0f };
	}

	static Rectangle fromMinMax(const Magnum2D::vec2& min, const Magnum2D::vec2& max)
	{
		return { min + (max - min) / 2.0f, max - min, min, max };
	}

	Magnum2D::vec2 center;
	Magnum2D::vec2 size;

	Magnum2D::vec2 min;
	Magnum2D::vec2 max;

	bool isInside(const Magnum2D::vec2& p)
	{
		return p.x() >= min.x() && p.x() <= max.x() && p.y() >= min.y() && p.y() <= max.y();
	}

	void setCenter(Magnum2D::vec2&& c)
	{
		center = c;
		min = c - size / 2.0f;
		max = c + size / 2.0f;
	}
};

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
		auto moustPositionWorld = Magnum2D::convertWindowToWorld(mousePosition);
		ImGui::Text("Mouse Window %.1f %.1f", mousePosition.x(), mousePosition.y());
		ImGui::Text("Mouse World %.1f %.1f", moustPositionWorld.x(), moustPositionWorld.y());
	}

	if (ImGui::CollapsingHeader("Simmulation"))
	{
		ImGui::SliderFloat("Time Delta", &RopeSimmulationDelta, 0.001f, 0.015f);
		ImGui::SliderFloat("Gravity", &Gravity.y(), -15.0f, -0.1f);
		ImGui::SliderInt("Constraint Iterations", &RopeConstraintIterations, 1, 40);
	}

	if (ImGui::CollapsingHeader("Rope"))
	{
		if (ImGui::SliderFloat("Node Distance", &RopeNodeDistance, 0.1f, 2.0f))
		{
			setupRope();
		}

		ImGui::Combo("Interaction", (int*)&Interaction, "Cut\0Grab\0Attract\0");
	}

	ImGui::End();
}

struct RopeNode
{
	RopeNode(Magnum2D::vec2& p) : position(p), positionOld(p) {}
	Magnum2D::vec2& position;
	Magnum2D::vec2 positionOld;
    float mass = 1.0f;

	std::vector<std::reference_wrapper<RopeNode>> childs;
};

struct Rope
{
	std::vector<Magnum2D::vec2> positions;
	std::vector<RopeNode> nodes;

	size_t pointsX;
	size_t pointsY;
};

Rope createRope(Magnum2D::vec2 p1, Magnum2D::vec2 p2)
{
	Rope rope;

	rope.positions = utils::generatePoints(p1, p2, RopeNodeDistance);
	for (size_t i = 0; i < rope.positions.size(); i++)
		rope.nodes.emplace_back(rope.positions[i]);

	for (size_t i = 0; i < rope.nodes.size(); i++)
	{
		if (i > 0)
			rope.nodes[i].childs.push_back(rope.nodes[i - 1]);
		if (i < rope.nodes.size() - 1)
			rope.nodes[i].childs.push_back(rope.nodes[i + 1]);
	}

	rope.pointsX = 1;
	rope.pointsY = rope.positions.size();

	return rope;
}

Rope createRectRope(Magnum2D::vec2 rectMin, Magnum2D::vec2 rectMax)
{
	Rope rope;

	rope.pointsX = (rectMax.x() - rectMin.x()) / RopeNodeDistance;
	rope.pointsY = (rectMax.y() - rectMin.y()) / RopeNodeDistance;

	rope.positions.resize(rope.pointsX * rope.pointsY);

	for (size_t x = 0; x < rope.pointsX; x++)
	{
		for (size_t y = 0; y < rope.pointsY; y++)
		{
			rope.positions[x * rope.pointsY + y] = Magnum2D::vec2(rectMin.x() + x * RopeNodeDistance, rectMin.y() + y * RopeNodeDistance);
			rope.nodes.emplace_back(rope.positions[x * rope.pointsY + y]);
		}
	}

	for (size_t x = 0; x < rope.pointsX; x++)
	{
		for (size_t y = 0; y < rope.pointsY; y++)
		{
			if (x > 0)
				rope.nodes[x * rope.pointsY + y].childs.push_back(rope.nodes[(x-1) * rope.pointsY + y]);
			if (x < rope.pointsX - 1)
				rope.nodes[x * rope.pointsY + y].childs.push_back(rope.nodes[(x + 1) * rope.pointsY + y]);
			if (y > 0)
				rope.nodes[x * rope.pointsY + y].childs.push_back(rope.nodes[x * rope.pointsY + y - 1]);
			if (y < rope.pointsY - 1)
				rope.nodes[x * rope.pointsY + y].childs.push_back(rope.nodes[x * rope.pointsY + y + 1]);
		}
	}

	for (size_t x = 0; x < rope.pointsX; x++)
		rope.nodes[x * rope.pointsY + rope.pointsY - 1].mass = 0.0f;

	return rope;
}

void drawRope(Rope& rope)
{
	for (const auto& p : rope.positions)
		Magnum2D::drawCircle(p, 0.05f, Magnum2D::rgb(0, 128, 255));

	std::set<const RopeNode*> visited;
	std::vector<Magnum2D::vec2> linePoints;

	std::queue< const RopeNode*> nodes;
	for (const RopeNode& node : rope.nodes)
	{
		if (visited.contains(&node))
			continue;
		nodes.push(&node);

		while (!nodes.empty())
		{
			const RopeNode* node = nodes.front();
			nodes.pop();

			if (visited.contains(node))
				continue;
			visited.insert(node);

			for (const RopeNode& child : node->childs)
			{
				linePoints.push_back(node->position);
				linePoints.push_back(child.position);

				nodes.push(&child);
			}
		}
	}

	Magnum2D::drawLines(linePoints, Magnum2D::rgb(0, 128, 255));
}

// ---

Rope g_rope;

std::vector<Rectangle> g_rectangles;

void applyCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to)
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
	Magnum2D::vec2 mousePosition = Magnum2D::getMousePositionWorld();

	for (auto& node : rope.nodes)
	{
		if (node.mass == 0.0f)
			continue;

		auto move = node.position - node.positionOld;
		Magnum2D::vec2 acceleration = Gravity;

		if (Interaction == InteractionAttract && Magnum2D::isMouseDown())
		{
			acceleration += (mousePosition - node.position);
		}

		node.positionOld = node.position;
		node.position += move + RopeSimmulationDelta * RopeSimmulationDelta * acceleration;

		applyCollisions(node.positionOld, node.position);
	}
}

void applyConstraints(Rope& rope)
{
	for (RopeNode& node1 : rope.nodes)
	{
		for (RopeNode& node2 : node1.childs)
		{
			float im1 = node1.mass == 0.0f ? 0.0f : 1.0f / node1.mass;
			float im2 = node2.mass == 0.0f ? 0.0f : 1.0f / node2.mass;
			float mult1 = im1 + im2 == 0.0f ? 0.0f : im1 / (im1 + im2), mult2 = im1 + im2 == 0.0f ? 0.0f : im2 / (im1 + im2);

			Magnum2D::vec2 diff = node1.position - node2.position;

			float dist = diff.length();
			float difference = 0.0f;
			if (dist > 0.0f)
				difference = (dist - RopeNodeDistance) / dist;

			diff *= 0.5f * difference;

			node1.position -= diff * mult1;
			node2.position += diff * mult2;
		}
	}
}

Magnum2D::vec2 getWindowRelative(const Magnum2D::vec2& relative)
{
	auto windowSize = Magnum2D::getWindowSize();

	return { windowSize.x() * relative.x(), windowSize.y() * relative.y() };
}

Magnum2D::vec2 getWindowRelativeCamera(const Magnum2D::vec2& relative)
{
	return Magnum2D::convertWindowToWorld(getWindowRelative(relative));
}

void setupRope()
{
	//g_rope = createRope({ 0.0f, 10.0f }, { -14.0f, 4.0f });
	//g_rope.nodes[0].mass = 0.0f;

	g_rope = createRectRope(getWindowRelativeCamera({ 0.1f, 0.18f }), getWindowRelativeCamera({ 0.9f, 0.98f }));
}

void setupRectangle()
{
	float offset = 0.01f * Magnum2D::getWindowSize().x();
	Magnum2D::vec2 min = Magnum2D::convertWindowToWorld({ offset, offset });
	Magnum2D::vec2 max = getWindowRelativeCamera({ 0.08f, 0.08f });
	g_rectangles.push_back(Rectangle::fromMinMax(min, max));
}

void setup()
{
	Magnum2D::setCameraCenter({ 0.0f, 0.0f });
	Magnum2D::setCameraSize({ 32.0f, 24.0f });

	setupRope();
	setupRectangle();
}

bool moveRect()
{
	static std::optional<Rectangle*> grabbedRect;
	static Magnum2D::vec2 grabOffset;

	if (Magnum2D::isMousePressed())
	{
		auto position = Magnum2D::getMousePositionWorld();

		for (auto& r : g_rectangles)
		{
			if (r.isInside(position))
			{
				grabbedRect = &r;
				grabOffset = position - r.center;
				
				return true;
			}
		}
	}
	else if (Magnum2D::isMouseReleased() && grabbedRect)
	{
		grabbedRect.reset();

		return true;
	}
	else if (grabbedRect)
	{
		auto position = Magnum2D::getMousePositionWorld();

		(*grabbedRect)->setCenter(position - grabOffset);

		return true;
	}

	return false;
}

void cutRope()
{
	static std::optional<Magnum2D::vec2> cutPosition;

	if (Magnum2D::isMousePressed())
	{
		cutPosition = Magnum2D::getMousePositionWorld();
	}
	else if (Magnum2D::isMouseReleased())
	{
		cutPosition.reset();
	}
	else if (cutPosition)
	{
		auto position = Magnum2D::getMousePositionWorld();

		for (auto& node : g_rope.nodes)
		{
			for (auto it = node.childs.begin(); it != node.childs.end();)
			{
				if (utils::doLineSegmentsIntersect(*cutPosition, position, node.position, ((RopeNode&)(*it)).position))
					it = node.childs.erase(it);
				else
					it++;
			}
		}
		cutPosition = position;
	}
}

RopeNode* getClosestNode(const Magnum2D::vec2& point)
{
	float distance = std::numeric_limits<float>::max();
	RopeNode* result = nullptr;

	for (auto& node : g_rope.nodes)
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
	static std::optional<RopeNode*> grabbedNode;

	if (Magnum2D::isMousePressed())
	{
		grabbedNode = getClosestNode(Magnum2D::getMousePositionWorld());
		(*grabbedNode)->mass = 0.0f;
	}
	else if (Magnum2D::isMouseReleased())
	{
		(*grabbedNode)->mass = 1.0f;
		grabbedNode = std::nullopt;
	}

	if (grabbedNode)
	{
		(*grabbedNode)->position = Magnum2D::getMousePositionWorld();
		(*grabbedNode)->positionOld = (*grabbedNode)->position;
	}
}

void draw()
{
	testWindow();

	//if (Magnum2D::isKeyDown('a') || Magnum2D::isKeyPressed('s'))
	{
		simmulateStep(g_rope);
		for (int32_t i = 0; i < RopeConstraintIterations; i++)
			applyConstraints(g_rope);
	}

	drawRope(g_rope);

	for (auto& r : g_rectangles)
		Magnum2D::drawRectangle(r.center, r.size.x(), r.size.y(), Magnum2D::rgb(50, 50, 50));

	if (!moveRect())
	{
		switch (Interaction)
		{
		case InteractionCut:
			cutRope();
			break;
		case InteractionGrab:
			grabRope();
			break;
		}
	}
}


