#include "utils.h"
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

	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians)
	{
		return { std::cos(radians) * vector.x() - std::sin(radians) * vector.y(), std::sin(radians) * vector.x() + std::cos(radians) * vector.y() };
	}

	void DrawVector(const vec2& position, const vec2& vector, const Magnum2D::col3& color)
	{
		static const float ArrowAngle = 10 * Deg2Rad;
		static const float ArrowSize = 0.1f;
		static const float GrabCircleRadius = 0.3f;

		vec2 destPosition = position + vector;
		
		drawLines({ position, destPosition }, color);

		auto arrowDir = (-vector).normalized();
		auto arrowLeft = RotateVector(arrowDir, ArrowAngle) * ArrowSize;
		auto arrowRight = RotateVector(arrowDir, -ArrowAngle) * ArrowSize;

		drawLines({ destPosition, destPosition + arrowLeft }, color);
		drawLines({ destPosition, destPosition + arrowRight }, color);
	}

	void DrawCross(const Magnum2D::vec2& position, float size, const Magnum2D::col3& color)
	{
		float hsize = size / 2.0f;

		drawLines({ position - vec2(0.0f, hsize), position + vec2(0.0f, hsize) }, color);
		drawLines({ position - vec2(hsize, 0.0f), position + vec2(hsize, 0.0f) }, color);
	}

	float DistanceSqr(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2)
	{
		float dx = p1.x() - p2.x();
		float dy = p1.y() - p2.y();

		return dx * dx + dy * dy;
	}
}
