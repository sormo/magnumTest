#include "utils.h"
#include <array>
#include <algorithm>

namespace utils
{
    std::vector<Magnum2D::vec2> generatePoints(Magnum2D::vec2 start, Magnum2D::vec2 end, float distance)
    {
        Magnum2D::vec2 direction = (end - start).normalized();
        float totalDistance = (end - start).length();
        int numPoints = std::ceilf(totalDistance / distance);

        std::vector<Magnum2D::vec2> points(numPoints);

        for (int i = 0; i < numPoints; i++)
        {
            // Calculate the position of the point
            Magnum2D::vec2 position = start + direction * i * distance;
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

    std::optional<Magnum2D::vec2> aabbRaycast(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2, const Magnum2D::vec2& rectMin, const Magnum2D::vec2& rectMax)
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
                return Magnum2D::vec2{ x, rectMax.y() };

            // vertical x = rectMin.x()
            y = m * rectMin.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return Magnum2D::vec2{ rectMin.x(), y };

            // vertical x = rectMax.x()
            y = m * rectMax.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return Magnum2D::vec2{ rectMax.x(), y };
        }
        else if (p1.y() < rectMin.y())
        {
            // horizontal y = rectMin.y()
            x = (rectMin.y() - q) / m;
            if (x > rectMin.x() && x < rectMax.x())
                return Magnum2D::vec2{ x, rectMin.y() };

            // vertical x = rectMin.x()
            y = m * rectMin.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return Magnum2D::vec2{ rectMin.x(), y };

            // vertical x = rectMax.x()
            y = m * rectMax.x() + q;
            if (y > rectMin.y() && y < rectMax.y())
                return Magnum2D::vec2{ rectMax.x(), y };
        }
        else
        {
            // origin point is with y values of rectangle, it can only intersect vertical lines
            if (p1.x() < rectMin.x())
            {
                // vertical x = rectMin.x()
                y = m * rectMin.x() + q;
                if (y > rectMin.y() && y < rectMax.y())
                    return Magnum2D::vec2{ rectMin.x(), y };
            }
            else
            {
                // vertical x = rectMax.x()
                y = m * rectMax.x() + q;
                if (y > rectMin.y() && y < rectMax.y())
                    return Magnum2D::vec2{ rectMax.x(), y };
            }
        }

        //// horizontal y = rectMin.y()
        //x = (rectMin.y() - q) / m;
        //if (x > rectMin.x() && x < rectMax.x())
        //    return Magnum2D::vec2{ x, rectMin.y() };

        //// horizontal y = rectMax.y()
        //x = (rectMax.y() - q) / m;
        //if (x > rectMin.x() && x < rectMax.x())
        //    return Magnum2D::vec2{ x, rectMax.y() };

        //// vertical x = rectMin.x()
        //y = m * rectMin.x() + q;
        //if (y > rectMin.y() && y < rectMax.y())
        //    return Magnum2D::vec2{ rectMin.x(), y };

        //// vertical x = rectMax.x()
        //y = m * rectMax.x() + q;
        //if (y > rectMin.y() && y < rectMax.y())
        //    return Magnum2D::vec2{ rectMax.x(), y };

        return {};
    }

    Magnum2D::vec2 getClosestPointOnEdge(const Magnum2D::vec2& p, const Magnum2D::vec2& rectMin, const Magnum2D::vec2& rectMax)
    {
        std::array<std::pair<float, Magnum2D::vec2>, 4> edges{{ { p.x() - rectMin.x(), { rectMin.x(), p.y() }},
                                                                { rectMax.x() - p.x(), { rectMax.x(), p.y() }},
                                                                { p.y() - rectMin.y(), { p.x(), rectMin.y() }},
                                                                { rectMax.y() - p.y(), { p.x(), rectMax.y() }} } };

        std::sort(edges.begin(), edges.end(), [](auto a, auto b) { return a.first < b.first; });

        return edges[0].second;
    }

    Magnum2D::vec2 getClosestPointOnCircle(const Magnum2D::vec2& p, const Magnum2D::vec2& center, float radius)
    {
        return (p - center).normalized() * radius + center;
    }

    // Function to find orientation of three points
    int orientation(const Magnum2D::vec2& p, const Magnum2D::vec2& q, const Magnum2D::vec2& r)
    {
        float val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
        if (fabs(val) < 1e-9) 
            return 0;  // collinear
        return (val > 0) ? 1 : 2; // 1 for clockwise, 2 for counterclockwise
    }

    // Function to check if a point lies within a line segment
    bool isWithinSegment(const Magnum2D::vec2& p, const Magnum2D::vec2& q, const Magnum2D::vec2& r)
    {
        if (q.x() <= fmax(p.x(), r.x()) && q.x() >= fmin(p.x(), r.x()) && q.y() <= fmax(p.y(), r.y()) && q.y() >= fmin(p.y(), r.y()))
            return true;
        return false;
    }

    // Function to check if two line segments intersect
    bool doLineSegmentsIntersect(const Magnum2D::vec2& p1, const Magnum2D::vec2& q1, const Magnum2D::vec2& p2, const Magnum2D::vec2& q2)
    {
        // Find the four orientations needed for general and special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special cases
        if (o1 == 0 && isWithinSegment(p1, p2, q1)) return true;  // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o2 == 0 && isWithinSegment(p1, q2, q1)) return true;  // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o3 == 0 && isWithinSegment(p2, p1, q2)) return true;  // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o4 == 0 && isWithinSegment(p2, q1, q2)) return true;  // p2, q2 and q1 are collinear and q1 lies on segment p2q2

        return false; // Doesn't fall in any of the above cases
    }

    Magnum2D::vec2 getWindowRelative(const Magnum2D::vec2& relative)
    {
        auto windowSize = Magnum2D::getWindowSize();

        return { windowSize.x() * relative.x(), windowSize.y() * relative.y() };
    }

    Magnum2D::vec2 getWindowRelativeCamera(const Magnum2D::vec2& relative)
    {
        return Magnum2D::convertWindowToCamera(getWindowRelative(relative));
    }

    Magnum2D::vec2 rotate(const Magnum2D::vec2& p, float degrees)
    {
        float radians = degrees * 0.0174533f;

        return { p.x() * std::cos(radians) - p.y() * sin(radians),  p.x() * std::sin(radians) + p.y() * cos(radians) };
    }

    struct Line {
        Magnum2D::vec2 start;
        Magnum2D::vec2 end;
    };

    bool isPointInsidePolygon(const Magnum2D::vec2& point, const std::vector<Magnum2D::vec2>& polygon)
    {
        int numIntersections = 0;
        Line ray;
        ray.start = point;
        ray.end.x() = std::numeric_limits<float>::infinity(); // Extend the ray infinitely horizontally

        for (size_t i = 0; i < polygon.size(); ++i)
        {
            const Magnum2D::vec2& p1 = polygon[i];
            const Magnum2D::vec2& p2 = polygon[(i + 1) % polygon.size()];

            if ((p1.y() > point.y() && p2.y() <= point.y()) || (p2.y() > point.y() && p1.y() <= point.y()))
            {
                float xIntersection = (p1.x() + (point.y() - p1.y()) / (p2.y() - p1.y()) * (p2.x() - p1.x()));

                if (xIntersection > point.x())
                    ++numIntersections;
            }
        }

        return (numIntersections % 2 == 1);
    }

    Magnum2D::vec2 findClosestPointOnEdge(const Magnum2D::vec2& insidePoint, const std::vector<Magnum2D::vec2>& polygon)
    {
        float minDistance = std::numeric_limits<float>::max();
        Magnum2D::vec2 closestPoint;

        for (size_t i = 0; i < polygon.size(); ++i)
        {
            const Magnum2D::vec2& p1 = polygon[i];
            const Magnum2D::vec2& p2 = polygon[(i + 1) % polygon.size()];

            float dx = p2.x() - p1.x();
            float dy = p2.y() - p1.y();
            float l2 = dx * dx + dy * dy;

            float t = std::max(0.0f, std::min(1.0f, ((insidePoint.x() - p1.x()) * dx + (insidePoint.y() - p1.y()) * dy) / l2));

            float closestX = p1.x() + t * dx;
            float closestY = p1.y() + t * dy;

            float distance = std::sqrt((insidePoint.x() - closestX) * (insidePoint.x() - closestX) + (insidePoint.y() - closestY) * (insidePoint.y() - closestY));

            if (distance < minDistance)
            {
                minDistance = distance;
                closestPoint.x() = closestX;
                closestPoint.y() = closestY;
            }
        }

        return closestPoint;
    }

    // Function to calculate the cross product of two vectors
    float crossProduct(const Magnum2D::vec2& a, const Magnum2D::vec2& b, const Magnum2D::vec2& c)
    {
        return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    }

    // Function to find intersection points between a line segment and a polygon
    std::vector<Magnum2D::vec2> findIntersectionPoints(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2, const std::vector<Magnum2D::vec2>& polygon)
    {
        std::vector<Magnum2D::vec2> intersectionPoints;

        for (int i = 0; i < polygon.size(); i++)
        {
            int j = (i + 1) % polygon.size();
            Line polygonSegment = { polygon[i], polygon[j] };

            if (doLineSegmentsIntersect(p1, p2, polygonSegment.start, polygonSegment.end))
            {
                float cp1 = crossProduct(p1, p2, polygonSegment.start);
                float cp2 = crossProduct(p1, p2, polygonSegment.end);

                if (std::abs(cp1) > 1e-9 || std::abs(cp2) > 1e-9)
                {
                    float t = cp1 / (cp1 - cp2);
                    float intersectionX = polygonSegment.start.x() + (polygonSegment.end.x() - polygonSegment.start.x()) * t;
                    float intersectionY = polygonSegment.start.y() + (polygonSegment.end.y() - polygonSegment.start.y()) * t;
                    intersectionPoints.push_back({ intersectionX, intersectionY });
                }
            }
        }

        return intersectionPoints;
    }

    std::optional<Magnum2D::vec2> findClosestPointOnEdge(const Magnum2D::vec2& insidePoint, const Magnum2D::vec2& outsidePoint, const std::vector<Magnum2D::vec2>& polygon)
    {
        auto points = findIntersectionPoints(insidePoint, outsidePoint, polygon);

        return points.empty() ? std::nullopt : std::optional<Magnum2D::vec2>(points[0]);
    }

    float angle(const Magnum2D::vec2& p)
    {
        return std::atan2f(p.y(), p.x()) * 57.2958f;
    }
}