#include "ConicApproximation.h"
#include "conicFit.h"
#include "..\utils.h"
#include <cmath>

extern double GravitationalConstant;

using namespace Magnum2D;

namespace ConicApproximation
{
	std::pair<vec2d, vec2d> Ellipse::GetMajorAxis() const
	{
		std::pair<vec2d, vec2d> ret;
		vec2d offset = Utils::RotateVector(vec2d{ 1.0, 0.0 }, angle) * radius.x();

		ret.first = position + offset;
		ret.second = position - offset;

		return ret;
	}

	std::pair<vec2d, vec2d> Ellipse::GetMinorAxis() const
	{
		std::pair<vec2d, vec2d> ret;
		vec2d offset = Utils::RotateVector(vec2d{ 1.0, 0.0 }, angle + 90.0 * Utils::Deg2Rad) * radius.y();

		ret.first = position + offset;
		ret.second = position - offset;

		return ret;
	}

	std::pair<vec2d, vec2d> Ellipse::GetFoci() const
	{
		// TODO !!!
		//assert(radius.x() > radius.y());

		std::pair<vec2d, vec2d> ret;
		vec2d offset = Utils::RotateVector(vec2d{ 1.0, 0.0 }, angle) * std::sqrt(radius.x() * radius.x() - radius.y() * radius.y());

		ret.first = position + offset;
		ret.second = position - offset;

		return ret;
	}

	double Ellipse::GetEccentricity()
	{
		return sqrt(1.0 - pow(radius.y(), 2) / pow(radius.x(), 2));
	}

	std::pair<vec2d, vec2d> Hyperbola::GetFoci() const
	{
		std::pair<vec2d, vec2d> ret;
		vec2d offset = Utils::RotateVector(vec2d{ 1.0, 0.0 }, angle) * std::sqrt(radius.x() * radius.x() + radius.y() * radius.y());

		ret.first = position + offset;
		ret.second = position - offset;

		return ret;
	}

	double Hyperbola::GetEccentricity()
	{
		return sqrt(1.0 + pow(radius.y(), 2) / pow(radius.x(), 2));
	}

	class PointAccessorVector
	{
	public:
		PointAccessorVector(std::span<vec2d> array)
			: array(array)
		{}
		size_t GetLength() const { return array.size(); }
		double GetX(size_t index) const { return array[index].x(); }
		double GetY(size_t index) const { return array[index].y(); }
	private:
		std::span<vec2d> array;
	};

	Conic ApproximateConic(std::span<vec2d> positions)
	{
		ConicFit::EllipseAlgebraicParameters<double> pa = ConicFit::LeastSquareEllipseFitter<double>::Fit(PointAccessorVector(positions));

		Conic ret;
		ret.A = pa.a;
		ret.B = pa.b;
		ret.C = pa.c;
		ret.D = pa.d;
		ret.E = pa.e;
		ret.F = pa.f;

		return ret;
	}

	Conic ApproximateConic(std::vector<vec2d>& positions)
	{
		return ApproximateConic(std::span<vec2d>{positions.data(), positions.size()});
	}

	ConicFit::EllipseAlgebraicParameters<double> FromConic(const Conic& c)
	{
		ConicFit::EllipseAlgebraicParameters<double> pa;

		pa.a = c.A;
		pa.b = c.B;
		pa.c = c.C;
		pa.d = c.D;
		pa.e = c.E;
		pa.f = c.F;

		return pa;
	}

	Ellipse Conic::GetEllipse() const
	{
		ConicFit::EllipseParameters<double> p = ConicFit::EllipseParameters<double>::FromAlgebraicParameters(FromConic(*this));

		Ellipse ret;
		ret.position = { p.x0, p.y0 };
		ret.radius = { p.a, p.b };
		ret.angle = p.theta;

		if (ret.radius.y() > ret.radius.x())
		{
			std::swap(ret.radius.x(), ret.radius.y());
			ret.angle += 90.0 * Utils::Deg2Rad;
		}

		// TODO verify this
		//assert(ret.radius.x() > ret.radius.y());

		return ret;
	}

	Hyperbola Conic::GetHyperbola() const
	{
		Hyperbola ret;
		Conic tmp = *this;

		// B is rotation
		if (Utils::SigmaCompare(B, 0.0) && A < C)
			ret.angle = 0.0;
		else if (Utils::SigmaCompare(B, 0.0) && A > C)
			ret.angle = Utils::Pi / 2.0;
		else if (!Utils::SigmaCompare(B, 0.0) && A < C)
			ret.angle = 0.5*atan(B / (A - C));
		else if (!Utils::SigmaCompare(B, 0.0) && A > C)
			ret.angle = Utils::Pi / 2.0 + 0.5*atan(B / (A - C));
		else if (!Utils::SigmaCompare(A, C)) // TODO
			ret.angle = Utils::Pi / 4.0; // what if B is not zero ???

		// https://math.stackexchange.com/questions/280937/finding-the-angle-of-rotation-of-an-ellipse-from-its-general-equation-and-the-ot
		tmp.A = A*pow(cos(ret.angle), 2) + B*cos(ret.angle)*sin(ret.angle) + C*pow(sin(ret.angle), 2);
		tmp.B = 0.0;
		tmp.C = A*pow(sin(ret.angle), 2) - B*cos(ret.angle)*sin(ret.angle) + C*pow(cos(ret.angle), 2);
		tmp.D = D*cos(ret.angle) + E*sin(ret.angle);
		tmp.E = -D*sin(ret.angle) + E*cos(ret.angle);
		tmp.F = F;

		// http://www.mathamazement.com/Lessons/Pre-Calculus/09_Conic-Sections-and-Analytic-Geometry/rotation-of-axes.html
		//tmp.A = (A + C) / 2.0 + ((A - C) / 2)*cos(2.0*ret.angle) - (B / 2.0)*sin(2.0*ret.angle);
		//tmp.B = (A - C)*sin(2.0*ret.angle) + B*cos(2.0*ret.angle);
		//tmp.C = (A + C) / 2.0 + ((C - A) / 2.0)*cos(2.0*ret.angle) + (B / 2.0)*sin(2.0*ret.angle);
		//tmp.D = D*cos(ret.angle) - E*sin(ret.angle);
		//tmp.E = D*sin(ret.angle) + E*cos(ret.angle);
		//tmp.F = F;

		// http://www.geom.uiuc.edu/docs/reference/CRC-formulas/node28.html
		//double q = sqrt(pow((C - A) / B, 2) + 1.0) + (C - A) / B;
		//tmp.A = A*q*q - B*q + C;
		//tmp.B = 2.0*A*q + B*q*q - B - 2.0*C*q;
		//tmp.C = A + B*q + C*q*q;
		//tmp.D = D*q - E;
		//tmp.E = D + E*q;
		//tmp.F = F;

		ret.position.x() = (2.0*C*D - E*B) / (B*B - 4.0*A*C);
		ret.position.y() = (2.0*A*E - D*B) / (B*B - 4.0*A*C);

		// translation in y direction
		// y = y - 0.5(E/C)
		// Ax ^ 2 + Bxy + Cy ^ 2 + x(D - 0.5B(E / C)) - 0.5E ^ 2 / C + F + 0.25E ^ 2 / C = 0
		if (!Utils::SigmaCompare(tmp.E, 0.0))
		{
			Conic tmp2 = tmp;

			tmp2.D = tmp.D - 0.5*tmp.B*(tmp.E/tmp.C);
			tmp2.E = 0.0;
			tmp2.F = F - (tmp.E*tmp.E)/(4.0*tmp.C);

			tmp = tmp2;
		}

		// translation in x direction
		// x = x - 0.5(D/A)
		// Ax ^ 2 + Byx + Cy ^ 2 + y(E - 0.5B(D / A)) + F + 0.25D ^ 2 / A - 0.5D ^ 2 / A = 0
		if (!Utils::SigmaCompare(tmp.D, 0.0))
		{
			Conic tmp2 = tmp;

			tmp2.D = 0.0;
			tmp2.E = tmp.E - 0.5*tmp.B*(tmp.D / tmp.A);
			tmp2.F = tmp.F - (tmp.D*tmp.D)/(4.0*tmp.A);

			tmp = tmp2;
		}

		// divide by F
		if (!Utils::SigmaCompare(tmp.F, 0.0))
		{
			tmp.A = tmp.A / tmp.F;
			tmp.B = tmp.B / tmp.F;
			tmp.C = tmp.C / tmp.F;
			tmp.D = tmp.D / tmp.F;
			tmp.E = tmp.E / tmp.F;
			tmp.F = 1.0;
		}

		// A and C must have oposite signs
		ret.radius.x() = 1.0 / std::sqrt(fabs(tmp.A));
		ret.radius.y() = 1.0 / std::sqrt(fabs(tmp.C));

		return ret;
	}

	Parabola Conic::GetParabola() const
	{
		// TODO
		return Parabola();
	}

	void Conic::Rotate(double rad)
	{
		double si = sin(rad);
		double co = cos(rad);

		Conic tmp = *this;
		tmp.A = A*pow(co, 2) + B*co*si + C*pow(si, 2);
		tmp.B = -2.0*A*co*si + B*pow(co, 2) - B*pow(si, 2) + 2.0*C*si*co;
		tmp.C = A*pow(si, 2) - B*si*co + C*pow(co, 2);
		tmp.D = D*co + E*si;
		tmp.E = -D*si + E*co;
		tmp.F = F;

		*this = tmp;
	}

	void Conic::Translate(const vec2d& vec)
	{
		Conic tmp = *this;

		tmp.D = 2.0*vec.x()*A + B*vec.y() + D;
		tmp.E = B*vec.x() + 2.0*C*vec.y() + E;
		tmp.F = A*pow(vec.x(), 2) + B*vec.x()*vec.y() + C*pow(vec.y(),2) + D*vec.x() + E*vec.y() + F;

		*this = tmp;
	}

	Conic::Type Conic::GetType() const
	{
		double discriminant = pow(B, 2) - 4.0*A*C;
		if (isnan(discriminant))
			return invalid;

		if (Utils::SigmaCompare(discriminant, 0.0))
			return parabola;
		else if (discriminant > 0.0)
			return hyperbola;
		else
			return ellipse;
	}

	Conic ComputeRotatedConic(double centralMass, double oribitingMass, const vec2d& distance, const vec2d& velocity)
	{
		double G = GravitationalConstant;
		double reducedMass = (centralMass*oribitingMass)/(centralMass + oribitingMass);
		double angularMomentum = reducedMass*distance.length()*velocity.y(); // use tangential part of velocity
		double energy = 0.5*reducedMass*pow(velocity.length(), 2) - (G*centralMass*oribitingMass)/distance.length();
		double semilatusRectum = pow(angularMomentum,2)/(reducedMass*G*centralMass*oribitingMass);
		double eccentricity = sqrt(1 + (2.0*energy*pow(angularMomentum, 2))/(reducedMass*pow(G*centralMass*oribitingMass, 2)));

		// AX^2 + BXY + CY^2 + DX + EY + F = 0

		Conic ret;
		ret.A = 1.0 - pow(eccentricity, 2);
		ret.B = 0.0;
		ret.C = 1.0;
		ret.D = -2.0*eccentricity*semilatusRectum;
		ret.E = 0.0;
		ret.F = -pow(semilatusRectum, 2);

		return ret;
	}

	Conic ComputeConic(double mass1, double mass2, const vec2d& position2, const vec2d& velocity2)
	{
		vec2d offset = position2;
		vec2d relativeVelocity = velocity2;
		double angle = Utils::GetAngle(offset);

		Utils::RotateVector(offset, -angle);
		Utils::RotateVector(relativeVelocity, -angle);

		Conic ret = ComputeRotatedConic(mass1, mass2, offset, relativeVelocity);
		
		// TODO why -angle ???
		ret.Rotate(-angle);

		// additional rotation caused by radial velocity
		if (ret.GetType() == Conic::ellipse)
		{
			Ellipse ellipse = ret.GetEllipse();
			double r = offset.x(), a = ellipse.radius.x(), e = ellipse.GetEccentricity();
			double radialAngle = (pow(r, 2) - pow(2.0*a - r, 2) + pow(2.0*a*e, 2)) / (4.0*a*e*r); // TODO simplify
			radialAngle = radialAngle > 1.0 ? 1.0 : (radialAngle < -1.0 ? -1.0 : radialAngle);
			radialAngle = acos(radialAngle);

			// TODO why those quadrants ???
			if ((relativeVelocity.x() > 0.0 && relativeVelocity.y() > 0.0) || (relativeVelocity.x() < 0.0 && relativeVelocity.y() < 0.0))
				radialAngle = -radialAngle;

			ret.Rotate(radialAngle);
		}
		else if (ret.GetType() == Conic::hyperbola)
		{
			Hyperbola hyperbola = ret.GetHyperbola();
			double r = offset.x(), a = hyperbola.radius.x(), e = hyperbola.GetEccentricity();

			// there are two solutions for z: z = r - 2a, z = r + 2a
			double x = r, y = 2.0*a*e, z = r + 2 * a;
			//z = r - 2 * a; // TODO which one

			double radialAngle = (x*x + y*y - z*z)/(2.0*x*y);
			radialAngle = radialAngle > 1.0 ? 1.0 : (radialAngle < -1.0 ? -1.0 : radialAngle);
			radialAngle = acos(radialAngle);

			// TODO what is this ???
			radialAngle = Utils::Pi - radialAngle;

			// TODO why those quadrants ???
			if ((relativeVelocity.x() > 0.0 && relativeVelocity.y() > 0.0) || (relativeVelocity.x() < 0.0 && relativeVelocity.y() < 0.0))
				radialAngle = -radialAngle;

			ret.Rotate(radialAngle);
		}
		else if (ret.GetType() == Conic::parabola)
		{
			//assert(false);
		}

		return ret;
	}
}
