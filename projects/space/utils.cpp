#include "utils.h"
#include "common.h"
#include <random>

using namespace Magnum2D;

std::random_device g_randDev;
std::mt19937 g_rand(g_randDev());

namespace Utils
{
	vec2 GetRandomPosition(float xmin, float xmax, float ymin, float ymax)
	{
		std::uniform_real_distribution<float> distx(xmin, xmax);
		std::uniform_real_distribution<float> disty(ymin, ymax);

		return { (float)distx(g_rand), (float)disty(g_rand) };
	}

	col3 GetRandomColor()
	{
		return rgb(rand() % 255, rand() % 255 , rand() % 255);
	}

	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians)
	{
		return { std::cos(radians) * vector.x() - std::sin(radians) * vector.y(), std::sin(radians) * vector.x() + std::cos(radians) * vector.y() };
	}

	void DrawVector(const vec2& position, const vec2& vector, const Magnum2D::col3& color)
	{
		static const float ArrowAngle = 10 * Deg2Rad;
		static const float ArrowSize = 0.2f;
		static const float GrabCircleRadius = 0.3f;

		vec2 destPosition = position + vector;
		
		const float LineWidth = Common::GetZoomIndependentSize(0.03f);

		Common::DrawLines({ position, destPosition }, LineWidth, color);

		auto arrowDir = (-vector).normalized();
		auto arrowLeft = RotateVector(arrowDir, ArrowAngle) * Common::GetZoomIndependentSize(ArrowSize);
		auto arrowRight = RotateVector(arrowDir, -ArrowAngle) * Common::GetZoomIndependentSize(ArrowSize);

		Common::DrawLines({ destPosition, destPosition + arrowLeft }, LineWidth, color);
		Common::DrawLines({ destPosition, destPosition + arrowRight }, LineWidth, color);
	}

	void DrawCross(const Magnum2D::vec2& position, float size, const Magnum2D::col3& color)
	{
		const float LineWidth = Common::GetZoomIndependentSize(0.03f);
		float hsize = size / 2.0f;

		Common::DrawLines({ position - vec2(0.0f, hsize), position + vec2(0.0f, hsize) }, LineWidth, color);
		Common::DrawLines({ position - vec2(hsize, 0.0f), position + vec2(hsize, 0.0f) }, LineWidth, color);
	}

	std::vector<vec2> ConvertToFloat(const std::vector<vec2d>& arr)
	{
		std::vector<vec2> result;
		result.reserve(arr.size());
		for (const auto& e : arr)
			result.push_back((vec2)e);
		return result;
	}

	std::vector<float> ConvertToFloat(const std::vector<double>& arr)
	{
		std::vector<float> result;
		result.reserve(arr.size());
		for (const auto& e : arr)
			result.push_back(e);
		return result;
	}

	float DistanceSqr(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2)
	{
		float dx = p1.x() - p2.x();
		float dy = p1.y() - p2.y();

		return dx * dx + dy * dy;
	}

	double DistanceSqr(const Magnum2D::vec2d& p1, const Magnum2D::vec2d& p2)
	{
		double dx = p1.x() - p2.x();
		double dy = p1.y() - p2.y();

		return dx * dx + dy * dy;
	}

	float LenghtSqr(const Magnum2D::vec2& p)
	{
		return p.x() * p.x() + p.y() * p.y();
	}

	double LenghtSqr(const Magnum2D::vec2d& p)
	{
		return p.x() * p.x() + p.y() * p.y();
	}

	bool ClickHandler::IsClick()
	{
		return Magnum2D::isMouseReleased() && accumulatedMouseDelta < MouseDeltaSqrThreshold;
	}

	void ClickHandler::Update()
	{
		if (isMouseDown())
		{
			accumulatedMouseDelta += LenghtSqr(convertWindowToWorldVector(getMouseDeltaWindow()));
		}

		if (isMouseReleased())
		{
			accumulatedMouseDelta = 0.0f;
		}
	}

	std::string GetRandomString(size_t chars)
	{
		std::string result(chars, '\0');

		for (size_t i = 0; i < chars; i++)
			result[i] = rand() % 25 + 97;

		return result;
	}
}

namespace Unit
{
	double Second = 1.0;
	double Minute = 60.0 * Second;
	double Hour = 60.0 * Minute;
	double Day = 24.0 * Hour;
	double Month = 31.0 * Day;
	double Year = 365 * Day;

	// mass
	double Kilogram = 1.0;
	double Ton = 10e3 * Kilogram;
	double Megaton = 10e6 * Ton;

	// length
	double Meter = 1.0;
	double Kilometer = 10e3 * Meter;
	double AU = 149597870700 * Meter;

	void SetBaseSecond(double base)
	{
		Second = base;
		Minute = 60.0 * Second;
		Hour = 60.0 * Minute;
		Day = 24.0 * Hour;
		Month = 31.0 * Day;
		Year = 365 * Day;
	}

	void SetBaseKilogram(double base)
	{
		Kilogram = base;
		Ton = 10e3 * Kilogram;
		Megaton = 10e6 * Ton;
	}

	void SetBaseMeter(double base)
	{
		Meter = base;
		Kilometer = 10e3 * Meter;
		AU = 149597870700 * Meter;
	}
}
