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
		RopeNode(Magnum2D::vec2& p) : position(p), positionOld(p) {}
		Magnum2D::vec2& position;
		Magnum2D::vec2 positionOld;
		float mass = 1.0f;

		std::vector<std::reference_wrapper<RopeNode>> childs;
	};

	std::vector<Magnum2D::vec2> positions;
	std::vector<RopeNode> nodes;

	size_t pointsX;
	size_t pointsY;
};

