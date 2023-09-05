#include "utils.h"
#include "common.h"
#include <random>
#include <Corrade/Utility/Resource.h>

using namespace Magnum2D;

std::random_device g_randDev;
std::mt19937 g_rand(g_randDev());

extern double GravitationalConstant;

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

	vec2 RotateVector(const vec2& vector, float radians)
	{
		return { std::cos(radians) * vector.x() - std::sin(radians) * vector.y(), std::sin(radians) * vector.x() + std::cos(radians) * vector.y() };
	}

	vec2d RotateVector(const vec2d& vector, double radians)
	{
		return { std::cos(radians) * vector.x() - std::sin(radians) * vector.y(), std::sin(radians) * vector.x() + std::cos(radians) * vector.y() };
	}

	double GetAngle(const vec2d& vector)
	{
		return atan2(vector.y(), vector.x());
	}

	void DrawVector(const vec2& position, const vec2& vector, const col3& color)
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

	void DrawCross(const vec2& position, float size, const col3& color)
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

	float DistanceSqr(const vec2& p1, const vec2& p2)
	{
		float dx = p1.x() - p2.x();
		float dy = p1.y() - p2.y();

		return dx * dx + dy * dy;
	}

	double DistanceSqr(const vec2d& p1, const vec2d& p2)
	{
		double dx = p1.x() - p2.x();
		double dy = p1.y() - p2.y();

		return dx * dx + dy * dy;
	}

	float LenghtSqr(const vec2& p)
	{
		return p.x() * p.x() + p.y() * p.y();
	}

	double LenghtSqr(const vec2d& p)
	{
		return p.x() * p.x() + p.y() * p.y();
	}

	bool ClickHandler::IsClick()
	{
		return isMouseReleased() && accumulatedMouseDelta < Common::GetZoomIndependentSize(MouseDeltaSqrThreshold);
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

	nlohmann::json ReadJsonFromResource(std::string_view group, std::string_view file)
	{
		Corrade::Utility::Resource resource(group.data());

		auto data = resource.getString(file.data());

		return nlohmann::json::parse((std::string)data);
	}

	double GetMeanDeviation(const std::vector<double>& data)
	{
		double mean = 0.0;
		mean = std::accumulate(std::begin(data), std::end(data), 0.0);
		mean /= (double)data.size();

		double accumulatedMeanDistances = 0.0;
		for (auto e : data)
			accumulatedMeanDistances += fabs(e - mean);

		return accumulatedMeanDistances / (double)data.size();
	}

	bool SigmaCompare(double a, double b, double sigma)
	{
		return std::fabs(a - b) <= sigma;
	}

	double ComputeDpt(double a, double b, double theta)
	{
		double dpt_sin = pow(a * sin(theta), 2.0f);
		double dpt_cos = pow(b * cos(theta), 2.0f);

		return sqrt(dpt_sin + dpt_cos);
	}

	std::vector<vec2d> GenerateEllipsePoints(double a, double b)
	{
		static const double EllipsePoints = 300.0;

		std::vector<vec2d> points;
		double theta = 0.0;
		double twoPi = Utils::Pi * 2.0;
		double deltaTheta = 0.0001f;
		double numIntegrals = round(twoPi / deltaTheta);
		double circ = 0.0;
		double dpt = 0.0;

		/* integrate over the elipse to get the circumference */
		for (int i = 0; i < numIntegrals; i++)
		{
			theta += i * deltaTheta;
			dpt = ComputeDpt(a, b, theta);
			circ += dpt;
		}

		int nextPoint = 0;
		double run = 0.0;

		theta = 0.0;

		for (int i = 0; i < numIntegrals; i++)
		{
			theta += deltaTheta;
			double subIntegral = EllipsePoints * run / circ;
			if ((int)subIntegral >= nextPoint)
			{
				double x = a * cos(theta);
				double y = b * sin(theta);
				points.push_back({ x, y });
				nextPoint++;
			}
			run += ComputeDpt(a, b, theta);
		}

		return points;
	}

	std::vector<vec2d> GenerateHyperbolaPoints(double a, double b)
	{
		std::vector<vec2d> result;

		// generate two halfs of hyperbola

		for (double theta = -40.0f; theta < 40.0f; theta += 0.1f)
		{
			double x = a * cosh(theta);
			double y = b * sinh(theta);
			result.push_back({ x, y });
		}

		for (double theta = -40.0f; theta < 40.0f; theta += 0.1f)
		{
			double x = -a * cosh(theta);
			double y = -b * sinh(theta);
			result.push_back({ x, y });
		}

		return result;
	}

	bool IsLeft(const vec2d& a, const vec2d& b, const vec2d& c)
	{
		return (c.x() - a.x()) * (b.y() - a.y()) - (c.y() - a.y()) * (b.x() - a.x()) < 0;
	}

	vec2d VelocityForCircularOrbit(const vec2d& point, const vec2d& center, double centerMass, bool left)
	{
		vec2d vec = center - point;
		vec2d perpendicularDirection;

		if (left)
			perpendicularDirection = vec2d(vec.y(), -vec.x()).normalized();
		else
			perpendicularDirection = vec2d(-vec.y(), vec.x()).normalized();

		double velocitySize = std::sqrt((GravitationalConstant * centerMass) / vec.length());
		return perpendicularDirection * velocitySize;
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
	double Ton = 1e3 * Kilogram;
	double Megaton = 1e6 * Ton;

	// length
	double Meter = 1.0;
	double Kilometer = 1e3 * Meter;
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
		Ton = 1e3 * Kilogram;
		Megaton = 1e6 * Ton;
	}

	void SetBaseMeter(double base)
	{
		Meter = base;
		Kilometer = 1e3 * Meter;
		AU = 149597870700 * Meter;
	}
}
