#include "common.h"
#include "camera.h"

extern Camera camera;

namespace Common
{
	using namespace Magnum2D;

	bool IsAntialisedLinesEnabled = true;

	float GetZoomIndependentSize(float size)
	{
		return size * camera.zoomFactor;
	}

	void DrawPolyline(std::span<vec2> points, float width, col3 color)
	{
		if (IsAntialisedLinesEnabled)
			Magnum2D::drawPolyline2(points, width, color);
		else
			Magnum2D::drawPolyline(points, color);
	}

	void DrawCircleOutline(vec2 center, float radius, float width, col3 color)
	{
		if (IsAntialisedLinesEnabled)
			Magnum2D::drawCircleOutline2(center, radius, width, color);
		else
			Magnum2D::drawCircleOutline(center, radius, color);
	}

	void DrawLines(std::vector<vec2>&& points, float width, col3 color)
	{
		if (IsAntialisedLinesEnabled)
			Magnum2D::drawLines2(points, width, color);
		else
			Magnum2D::drawLines(points, color);
	}
}
