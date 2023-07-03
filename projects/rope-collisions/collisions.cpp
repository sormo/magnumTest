#include "collisions.h"
#include "application.h"
#include "utils.h"

namespace Collisions
{
	void applyRectangleCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to)
	{
		for (const auto& rect : g_app->rectangles)
		{
			// if "from" is inside of rectangle, move it to closest edge
			if (rect.IsInside(from))
			{
				auto closestPoint = utils::getClosestPointOnEdge(from, rect.min, rect.max);

				from = closestPoint;
				to = closestPoint;
			}
			// if "to" is inside, raycast from->to and get intersection with rectangle
			else if (rect.IsInside(to))
			{
				if (auto p = utils::aabbRaycast(from, to, rect.min, rect.max))
				{
					to = *p;
					break;
				}
			}
		}
	}

	void applyCircleCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to)
	{
		for (const auto& circ : g_app->circles)
		{
			if (circ.IsInside(from))
			{
				auto closestPoint = utils::getClosestPointOnCircle(from, circ.center, circ.radius);

				from = closestPoint;
				to = closestPoint;
			}
			else if (circ.IsInside(to))
			{
				to = utils::getClosestPointOnCircle(from, circ.center, circ.radius);
				break;
			}
		}
	}

	void applyCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to)
	{
		applyRectangleCollisions(from, to);
		applyCircleCollisions(from, to);
	}

	void applyCollisions(Magnum2D::vec2& point)
	{
		for (const auto& rect : g_app->rectangles)
		{
			if (rect.IsInside(point))
			{
				point = utils::getClosestPointOnEdge(point, rect.min, rect.max);
				break;
			}
		}

		for (const auto& circ : g_app->circles)
		{
			if (circ.IsInside(point))
			{
				point = utils::getClosestPointOnCircle(point, circ.center, circ.radius);
				break;
			}
		}
	}
}
