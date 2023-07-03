#pragma once
#include <vector>
#include <optional>
#include <Magnum2D.h>

namespace utils
{
	std::vector<Magnum2D::vec2> generatePoints(Magnum2D::vec2 start, Magnum2D::vec2 end, float distance);
	std::optional<Magnum2D::vec2> aabbRaycast(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2, const Magnum2D::vec2& rectMin, const Magnum2D::vec2& rectMax);

	// return the closest point on edge of rectangle if p is inside of rectangle
	Magnum2D::vec2 getClosestPointOnEdge(const Magnum2D::vec2& p, const Magnum2D::vec2& rectMin, const Magnum2D::vec2& rectMax);
	// return closest point on circle if p is inside of circle
	Magnum2D::vec2 getClosestPointOnCircle(const Magnum2D::vec2& p, const Magnum2D::vec2& center, float radius);

	bool doLineSegmentsIntersect(const Magnum2D::vec2& p1, const Magnum2D::vec2& q1, const Magnum2D::vec2& p2, const Magnum2D::vec2& q2);

	Magnum2D::vec2 getWindowRelative(const Magnum2D::vec2& relative);
	Magnum2D::vec2 getWindowRelativeCamera(const Magnum2D::vec2& relative);

	Magnum2D::vec2 rotate(const Magnum2D::vec2& p, float degrees);

	bool isPointInsidePolygon(const Magnum2D::vec2& point, const std::vector<Magnum2D::vec2>& polygon);
	Magnum2D::vec2 findClosestPointOnEdge(const Magnum2D::vec2& insidePoint, const std::vector<Magnum2D::vec2>& polygon);
	std::optional<Magnum2D::vec2> findClosestPointOnEdge(const Magnum2D::vec2& insidePoint, const Magnum2D::vec2& outsidePoint, const std::vector<Magnum2D::vec2>& polygon);
}
