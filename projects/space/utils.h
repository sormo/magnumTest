#pragma once
#include <Magnum2D.h>

namespace Utils
{
	const float Deg2Rad = 0.0174533f;

	Magnum2D::vec2 GetRandomPosition(float xmin, float xmax, float ymin, float ymax); // ranges are inclusive
	Magnum2D::col3 GetRandomColor();
	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians);
	float DistanceSqr(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2);

	void DrawVector(const Magnum2D::vec2& position, const Magnum2D::vec2& vector, const Magnum2D::col3& color);
	void DrawCross(const Magnum2D::vec2& position, float size, const Magnum2D::col3& color);

	std::vector<Magnum2D::vec2> ConvertToFloat(const std::vector<Magnum2D::vec2d>& arr);
	std::vector<float> ConvertToFloat(const std::vector<double>& arr);
}