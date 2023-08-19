#pragma once
#include <Magnum2D.h>

enum class UpdateResult
{
	None,
	Modified,
	InputGrab,
};

namespace Common
{
	extern bool IsAntialisedLinesEnabled;

	float GetZoomIndependentSize(float size);

	void DrawPolyline(std::span<Magnum2D::vec2> points, float width, Magnum2D::col3 color);
	void DrawCircleOutline(Magnum2D::vec2 center, float radius, float width, Magnum2D::col3 color);
	void DrawLines(std::vector<Magnum2D::vec2>&& points, float width, Magnum2D::col3 color);
}
