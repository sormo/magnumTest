#include "utils.h"
#include <array>
#include <algorithm>

namespace utils
{
    std::vector<app2d::vec2> generatePoints(app2d::vec2 start, app2d::vec2 end, float distance)
    {
        app2d::vec2 direction = (end - start).normalized();
        float totalDistance = (end - start).length();
        int numPoints = std::ceilf(totalDistance / distance);

        std::vector<app2d::vec2> points(numPoints);

        for (int i = 0; i < numPoints; i++)
        {
            // Calculate the position of the point
            app2d::vec2 position = start + direction * i * distance;
            // If this is the last point and it's too close to the end point,
            // move it closer to the end point
            if (i == numPoints - 1 && (position - end).length() < distance / 2.0f)
            {
                position = end;
            }
            // Add the point to the array
            points[i] = position;
        }
        // Return the array of points
        return points;
    }

    std::optional<app2d::vec2> aabbRaycast(const app2d::vec2& p1, const app2d::vec2& p2, const app2d::vec2& rectMin, const app2d::vec2& rectMax)
    {
        // Completely outside.
        if ((p1.x() <= rectMin.x() && p2.x() <= rectMin.x()) || 
            (p1.y() <= rectMin.y() && p2.y() <= rectMin.y()) || 
            (p1.x() >= rectMax.x() && p2.x() >= rectMax.x()) || 
            (p1.y() >= rectMax.y() && p2.y() >= rectMax.y()))
            return {};

        // y = mx + q
        float m = (p2.y() - p1.y()) / (p2.x() - p1.x());
        float q = p1.y() - m * p1.x();

        // now we will plug in the lines of the rectangle and compute intersection
        float x = 0.0f, y = 0.0f;

        // origin point is above the rectangle, it can intersect only
        if (p1.y() > rectMax.y())
        {
            // horizontal y = rectMax.y()
            x = (rectMax.y() - q) / m;
            if (x > rectMin.x() && x < rectMax.x())
                return app2d::vec2{ x, rectMax.y() };

            // vertical x = rectMin.x()
            y = m * rectMin.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return app2d::vec2{ rectMin.x(), y };

            // vertical x = rectMax.x()
            y = m * rectMax.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return app2d::vec2{ rectMax.x(), y };
        }
        else if (p1.y() < rectMin.y())
        {
            // horizontal y = rectMin.y()
            x = (rectMin.y() - q) / m;
            if (x > rectMin.x() && x < rectMax.x())
                return app2d::vec2{ x, rectMin.y() };

            // vertical x = rectMin.x()
            y = m * rectMin.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return app2d::vec2{ rectMin.x(), y };

            // vertical x = rectMax.x()
            y = m * rectMax.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return app2d::vec2{ rectMax.x(), y };
        }
        else
        {
            // origin point is with y values of rectangle, it can only intersect vertical lines
            if (p1.x() < rectMin.x())
            {
                // vertical x = rectMin.x()
                y = m * rectMin.x() + q;
                if (y > rectMin.y() && y < rectMax.y())
                    return app2d::vec2{ rectMin.x(), y };
            }
            else
            {
                // vertical x = rectMax.x()
                y = m * rectMax.x() + q;
                if (y > rectMin.y() && y < rectMax.y())
                    return app2d::vec2{ rectMax.x(), y };
            }
        }

        //// horizontal y = rectMin.y()
        //x = (rectMin.y() - q) / m;
        //if (x > rectMin.x() && x < rectMax.x())
        //    return app2d::vec2{ x, rectMin.y() };

        //// horizontal y = rectMax.y()
        //x = (rectMax.y() - q) / m;
        //if (x > rectMin.x() && x < rectMax.x())
        //    return app2d::vec2{ x, rectMax.y() };

        //// vertical x = rectMin.x()
        //y = m * rectMin.x() + q;
        //if (y > rectMin.y() && y < rectMax.y())
        //    return app2d::vec2{ rectMin.x(), y };

        //// vertical x = rectMax.x()
        //y = m * rectMax.x() + q;
        //if (y > rectMin.y() && y < rectMax.y())
        //    return app2d::vec2{ rectMax.x(), y };

        return {};
    }

    app2d::vec2 getClosestPointOnEdge(const app2d::vec2& p, const app2d::vec2& rectMin, const app2d::vec2& rectMax)
    {
        std::array<std::pair<float, app2d::vec2>, 4> edges{{ { p.x() - rectMin.x(), { rectMin.x(), p.y() }},
                                                             { rectMax.x() - p.x(), { rectMax.x(), p.y() }},
                                                             { p.y() - rectMin.y(), { p.x(), rectMin.y() }},
                                                             { rectMax.y() - p.y(), { p.x(), rectMax.y() }} } };
        std::sort(edges.begin(), edges.end(), [](auto a, auto b) { return a.first < b.first; });

        return edges[0].second;
    }
}
