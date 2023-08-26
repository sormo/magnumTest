#pragma once
#include <Magnum2D.h>

namespace Utils
{
	const float Deg2Rad = 0.0174533f;

	Magnum2D::vec2 GetRandomPosition(float xmin, float xmax, float ymin, float ymax); // ranges are inclusive
	Magnum2D::col3 GetRandomColor();
	std::string GetRandomString(size_t chars);

	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians);
	float DistanceSqr(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2);
	double DistanceSqr(const Magnum2D::vec2d& p1, const Magnum2D::vec2d& p2);
	float LenghtSqr(const Magnum2D::vec2& p);
	double LenghtSqr(const Magnum2D::vec2d& p);

	void DrawVector(const Magnum2D::vec2& position, const Magnum2D::vec2& vector, const Magnum2D::col3& color);
	void DrawCross(const Magnum2D::vec2& position, float size, const Magnum2D::col3& color);

	std::vector<Magnum2D::vec2> ConvertToFloat(const std::vector<Magnum2D::vec2d>& arr);
	std::vector<float> ConvertToFloat(const std::vector<double>& arr);

	struct ClickHandler
	{
		static constexpr float MouseDeltaSqrThreshold = 0.15f * 0.15f;

		bool IsClick();
		void Update();

	private:
		// how much distance was mouse moved while pressed
		float accumulatedMouseDelta = 0.0f;

	};
}
