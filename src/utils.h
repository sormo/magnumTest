#pragma once
#include <vector>
#include <optional>
#include "Application2D.h"

namespace utils
{
	std::vector<app2d::vec2> generatePoints(app2d::vec2 start, app2d::vec2 end, float distance);
	std::optional<app2d::vec2> aabbRaycast(const app2d::vec2& p1, const app2d::vec2& p2, const app2d::vec2& rectMin, const app2d::vec2& rectMax);

	app2d::vec2 getClosestPointOnEdge(const app2d::vec2& p, const app2d::vec2& rectMin, const app2d::vec2& rectMax);

	bool doLineSegmentsIntersect(const app2d::vec2& p1, const app2d::vec2& q1, const app2d::vec2& p2, const app2d::vec2& q2);
}
