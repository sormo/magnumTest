#pragma once
#include <Magnum2D.h>
#include <nlohmann/json.hpp>

namespace Unit
{
	// time
	extern double Second;
	extern double Minute;
	extern double Hour;
	extern double Day;
	extern double Month;
	extern double Year;

	// mass
	extern double Kilogram;
	extern double Ton;
	extern double Megaton;

	// length
	extern double Meter;
	extern double Kilometer;
	extern double AU;

	void SetBaseSecond(double base);
	void SetBaseKilogram(double base);
	void SetBaseMeter(double base);
}

namespace Utils
{
	const double Deg2Rad = 0.0174533;
	const double Pi = 3.1415926535897931;

	Magnum2D::vec2 GetRandomPosition(float xmin, float xmax, float ymin, float ymax); // ranges are inclusive
	Magnum2D::col3 GetRandomColor();
	std::string GetRandomString(size_t chars);

	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians);
	Magnum2D::vec2d RotateVector(const Magnum2D::vec2d& vector, double radians);

	double GetAngle(const Magnum2D::vec2d& vector);

	double GetMeanDeviation(const std::vector<double>& data);

	float DistanceSqr(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2);
	double DistanceSqr(const Magnum2D::vec2d& p1, const Magnum2D::vec2d& p2);
	float LenghtSqr(const Magnum2D::vec2& p);
	double LenghtSqr(const Magnum2D::vec2d& p);

	void DrawVector(const Magnum2D::vec2& position, const Magnum2D::vec2& vector, const Magnum2D::col3& color);
	void DrawCross(const Magnum2D::vec2& position, float size, const Magnum2D::col3& color);

	std::vector<Magnum2D::vec2> ConvertToFloat(const std::vector<Magnum2D::vec2d>& arr);
	std::vector<float> ConvertToFloat(const std::vector<double>& arr);

	// check whether point c is on the left side of line a-b
	bool IsLeft(const Magnum2D::vec2d& a, const Magnum2D::vec2d& b, const Magnum2D::vec2d& c);

	Magnum2D::vec2d VelocityForCircularOrbit(const Magnum2D::vec2d& point, const Magnum2D::vec2d& center, double centerMass, bool left);

	bool SigmaCompare(double a, double b, double sigma = 0.0000000001);

	struct ClickHandler
	{
		static constexpr float MouseDeltaSqrThreshold = 0.015f * 0.015f;

		bool IsClick();
		void Update();

	private:
		// how much distance was mouse moved while pressed
		float accumulatedMouseDelta = 0.0f;
	};

	nlohmann::json ReadJsonFromResource(std::string_view group, std::string_view file);

	std::vector<Magnum2D::vec2d> GenerateEllipsePoints(double a, double b);
	std::vector<Magnum2D::vec2d> GenerateHyperbolaPoints(double a, double b);
}
