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
				auto localPoint = rect.ConvertToLocal(from);
				auto closestPoint = utils::getClosestPointOnEdge(localPoint, -rect.hsize, rect.hsize);
				auto globalPoint = rect.ConvertToGlobal(closestPoint);

				from = globalPoint;
				to = globalPoint;
			}
			// if "to" is inside, raycast from->to and get intersection with rectangle
			else if (rect.IsInside(to))
			{
				auto localFrom = rect.ConvertToLocal(from);
				auto localTo = rect.ConvertToLocal(to);
				
				if (auto p = utils::aabbRaycast(localFrom, localTo, -rect.hsize, rect.hsize))
				{
					to = rect.ConvertToGlobal(*p);
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

	void applyPolygonCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to)
	{
		for (const auto& poly : g_app->polygons)
		{
			// if "from" is inside of rectangle, move it to closest edge
			if (poly.IsInside(from))
			{
				auto closestPoint = utils::findClosestPointOnEdge(from, poly.pointsMoved);

				from = closestPoint;
				to = closestPoint;
			}
			// if "to" is inside, raycast from->to and get intersection with rectangle
			else if (poly.IsInside(to))
			{
				if (auto p = utils::findClosestPointOnEdge(from, to, poly.pointsMoved))
				{
					to = *p;
					break;
				}
			}
		}
	}

	void applyCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to)
	{
		applyRectangleCollisions(from, to);
		applyCircleCollisions(from, to);
		applyPolygonCollisions(from, to);
	}

	void applyCollisions(Magnum2D::vec2& point)
	{
		for (const auto& rect : g_app->rectangles)
		{
			if (rect.IsInside(point))
			{
				auto localPoint = rect.ConvertToLocal(point);
				auto closestPoint = utils::getClosestPointOnEdge(localPoint, -rect.hsize, rect.hsize);
				auto globalPoint = rect.ConvertToGlobal(closestPoint);

				point = globalPoint;
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

		for (const auto& poly : g_app->polygons)
		{
			if (poly.IsInside(point))
			{
				point = utils::findClosestPointOnEdge(point, poly.pointsMoved);
				break;
			}
		}

	}
}
