#pragma once
#include <vector>
#include <Magnum2D.h>

namespace ConicApproximation
{
	struct Ellipse
	{
		Magnum2D::vec2d radius;
		Magnum2D::vec2d position;
		double angle = 0.0;

		std::pair<Magnum2D::vec2d, Magnum2D::vec2d> GetMajorAxis() const;
		std::pair<Magnum2D::vec2d, Magnum2D::vec2d> GetMinorAxis() const;
		std::pair<Magnum2D::vec2d, Magnum2D::vec2d> GetFoci() const;

		double GetEccentricity();
	};

	struct Hyperbola
	{
		Magnum2D::vec2d radius;
		Magnum2D::vec2d position;
		double angle = 0.0;

		std::pair<Magnum2D::vec2d, Magnum2D::vec2d> GetFoci() const;

		double GetEccentricity();
	};

	struct Parabola
	{
		double radius;
		Magnum2D::vec2d position;
		double angle;
	};

	struct Conic
	{
		// AX^2 + BXY + CY^2 + DX + EY + F = 0
		double A = 0.0, B = 0.0, C = 0.0, D = 0.0, E = 0.0, F = 0.0;

		Ellipse GetEllipse() const;
		Hyperbola GetHyperbola() const;
		Parabola GetParabola() const;

		void Rotate(double rad);
		void Translate(const Magnum2D::vec2d& vec);

		enum Type
		{
			invalid,
			ellipse,
			parabola,
			hyperbola
		};
		Type GetType() const;
	};

	// all conics are parent relative

	Conic ApproximateConic(std::vector<Magnum2D::vec2d>& positions);
	Conic ApproximateConic(std::span<Magnum2D::vec2d>& positions);

	Conic ComputeConic(double mass1, double mass2, const Magnum2D::vec2d & position2, const Magnum2D::vec2d & velocity2);

}
