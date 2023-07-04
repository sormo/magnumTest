#pragma once
#include <Magnum2D.h>


struct Rope
{
	Rope(Magnum2D::vec2 p1, Magnum2D::vec2 p2);

	void SimmulateStep();
	void ApplyConstraints();

	void Draw();

	struct RopeNode
	{
		RopeNode(Magnum2D::vec2& p, float d) : position(p), positionOld(p), distance(d) {}
		Magnum2D::vec2 position;
		Magnum2D::vec2 positionOld;
		float mass = 1.0f;
		float distance = 1.0f; // distance to childs

		std::vector<std::reference_wrapper<RopeNode>> childs;
	};

	std::vector<RopeNode> nodes;
};

