#pragma once
#include <vector>
#include <optional>
#include <Magnum2D.h>

namespace utils
{
	std::vector<Magnum2D::vec2> generatePoints(Magnum2D::vec2 start, Magnum2D::vec2 end, float distance);
	std::optional<Magnum2D::vec2> aabbRaycast(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2, const Magnum2D::vec2& rectMin, const Magnum2D::vec2& rectMax);

	Magnum2D::vec2 getClosestPointOnEdge(const Magnum2D::vec2& p, const Magnum2D::vec2& rectMin, const Magnum2D::vec2& rectMax);

	bool doLineSegmentsIntersect(const Magnum2D::vec2& p1, const Magnum2D::vec2& q1, const Magnum2D::vec2& p2, const Magnum2D::vec2& q2);
}
