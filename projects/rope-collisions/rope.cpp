#include "rope.h"
#include "utils.h"
#include "globals.h"
#include "collisions.h"
#include <Magnum2D.h>
#include <set>
#include <queue>

void Rope::SimmulateStep()
{
	Magnum2D::vec2 mousePosition = Magnum2D::getMousePositionCamera();

	for (auto& node : nodes)
	{
		if (node.mass == 0.0f)
			continue;

		auto move = node.position - node.positionOld;
		Magnum2D::vec2 acceleration = Globals::Gravity;

		if (Globals::Interaction == Globals::InteractionAttract && Magnum2D::isMouseDown())
		{
			acceleration += (mousePosition - node.position);
		}

		node.positionOld = node.position;
		node.position += move + Globals::RopeSimmulationDelta * Globals::RopeSimmulationDelta * acceleration;

		Collisions::applyCollisions(node.positionOld, node.position);
	}
}

void Rope::ApplyConstraints()
{
	for (RopeNode& node1 : nodes)
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
				difference = (dist - node1.distance) / dist;

			diff *= 0.5f * difference;

			node1.position -= diff * mult1;
			node2.position += diff * mult2;
		}
	}

	for (auto& n : nodes)
		Collisions::applyCollisions(n.position);
}

Rope::Rope(Magnum2D::vec2 p1, Magnum2D::vec2 p2)
{
	auto points = utils::generatePoints(p1, p2, Globals::RopeNodeDistance);
	for (size_t i = 0; i < points.size(); i++)
		nodes.emplace_back(points[i], Globals::RopeNodeDistance);

	for (size_t i = 0; i < nodes.size(); i++)
	{
		if (i > 0)
			nodes[i].childs.push_back(nodes[i - 1]);
		if (i < nodes.size() - 1)
			nodes[i].childs.push_back(nodes[i + 1]);
	}
}

void Rope::Draw()
{
	for (const auto& n : nodes)
		Magnum2D::drawCircle(n.position, 0.05f, Magnum2D::rgb(0, 128, 255));

	std::set<const RopeNode*> visited;
	std::vector<Magnum2D::vec2> linePoints;

	std::queue< const RopeNode*> processNodes;
	for (const RopeNode& parent : nodes)
	{
		if (visited.contains(&parent))
			continue;
		processNodes.push(&parent);

		while (!processNodes.empty())
		{
			const RopeNode* node = processNodes.front();
			processNodes.pop();

			if (visited.contains(node))
				continue;
			visited.insert(node);

			for (const RopeNode& child : node->childs)
			{
				linePoints.push_back(node->position);
				linePoints.push_back(child.position);

				processNodes.push(&child);
			}
		}
	}

	Magnum2D::drawLines(linePoints, Magnum2D::rgb(0, 128, 255));
}
