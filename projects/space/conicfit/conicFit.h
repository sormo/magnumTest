#pragma once
#include "Eigen/Eigen"

// *****************************************************************************
// This code is taken from github. All credit belongs to ptahmose.
// https://github.com/ptahmose/EllipseFitPlayground
// *****************************************************************************

namespace ConicFit
{
	template<typename tFloat>
	struct EllipseAlgebraicParameters
	{
	public:
		tFloat a, b, c, d, e, f;

		static EllipseAlgebraicParameters CreateFrom5Points(tFloat p1x, tFloat p1y, tFloat p2x, tFloat p2y, tFloat p3x, tFloat p3y, tFloat p4x, tFloat p4y, tFloat p5x, tFloat p5y)
		{
			tFloat pp0[3] = { p1x, p1y, 1 };
			tFloat pp1[3] = { p2x, p2y, 1 };
			tFloat pp2[3] = { p3x, p3y, 1 };
			tFloat pp3[3] = { p4x, p4y, 1 };
			tFloat pp4[3] = { p5x, p5y, 1 };
			return EllipseAlgebraicParameters::FromPoints(pp0, pp1, pp2, pp3, pp4);
		}

		static EllipseAlgebraicParameters CreateFrom5Points(const tFloat* points)
		{
			tFloat pp0[3] = { points[0], points[1], 1 };
			tFloat pp1[3] = { points[2], points[3], 1 };
			tFloat pp2[3] = { points[4], points[5], 1 };
			tFloat pp3[3] = { points[6], points[7], 1 };
			tFloat pp4[3] = { points[8], points[9], 1 };
			return EllipseAlgebraicParameters::FromPoints(pp0, pp1, pp2, pp3, pp4);
		}

		bool IsEllipse() const
		{
			return this->b*this->b - 4 * this->a*this->c < 0;
		}

		static EllipseAlgebraicParameters<tFloat> Invalid()
		{
			return  EllipseAlgebraicParameters<tFloat>
			{
				std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN(), 
					std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN()
			};
		}

	private:
		static EllipseAlgebraicParameters FromPoints(tFloat p0[3], tFloat p1[3], tFloat p2[3], tFloat p3[3], tFloat p4[3])
		{
			tFloat L0[3], L1[3], L2[3], L3[3];
			tFloat A, B, C, Q[3];
			tFloat a1, a2, b1, b2, c1, c2;
			tFloat x0, x4, y0, y4, w0, w4;
			tFloat aa, bb, cc, dd, ee, ff;
			tFloat y4w0, w4y0, w4w0, y4y0, x4w0, w4x0, x4x0, y4x0, x4y0;
			tFloat a1a2, a1b2, a1c2, b1a2, b1b2, b1c2, c1a2, c1b2, c1c2;

			L0[0] = p0[1] * p1[2] - p0[2] * p1[1]; 	L0[1] = p0[2] * p1[0] - p0[0] * p1[2]; L0[2] = p0[0] * p1[1] - p0[1] * p1[0];
			L1[0] = p1[1] * p2[2] - p1[2] * p2[1]; L1[1] = p1[2] * p2[0] - p1[0] * p2[2]; 	L1[2] = p1[0] * p2[1] - p1[1] * p2[0];
			L2[0] = p2[1] * p3[2] - p2[2] * p3[1]; 	L2[1] = p2[2] * p3[0] - p2[0] * p3[2]; 	L2[2] = p2[0] * p3[1] - p2[1] * p3[0];
			L3[0] = p3[1] * p4[2] - p3[2] * p4[1]; 	L3[1] = p3[2] * p4[0] - p3[0] * p4[2]; L3[2] = p3[0] * p4[1] - p3[1] * p4[0];
			Q[0] = L0[1] * L3[2] - L0[2] * L3[1]; Q[1] = L0[2] * L3[0] - L0[0] * L3[2]; Q[2] = L0[0] * L3[1] - L0[1] * L3[0];

			A = Q[0]; B = Q[1]; C = Q[2];
			a1 = L1[0]; b1 = L1[1]; c1 = L1[2];
			a2 = L2[0]; b2 = L2[1]; c2 = L2[2];
			x0 = p0[0]; y0 = p0[1]; w0 = p0[2];
			x4 = p4[0]; y4 = p4[1]; w4 = p4[2];

			y4w0 = y4*w0;
			w4y0 = w4*y0;
			w4w0 = w4*w0;
			y4y0 = y4*y0;
			x4w0 = x4*w0;
			w4x0 = w4*x0;
			x4x0 = x4*x0;
			y4x0 = y4*x0;
			x4y0 = x4*y0;
			a1a2 = a1*a2;
			a1b2 = a1*b2;
			a1c2 = a1*c2;
			b1a2 = b1*a2;
			b1b2 = b1*b2;
			b1c2 = b1*c2;
			c1a2 = c1*a2;
			c1b2 = c1*b2;
			c1c2 = c1*c2;

			aa = -A*a1a2*y4w0 + A*a1a2*w4y0 - B*b1a2*y4w0 - B*c1a2*w4w0 + B*a1b2*w4y0 +
				B*a1c2*w4w0 + C*b1a2*y4y0 + C*c1a2*w4y0 - C*a1b2*y4y0 - C*a1c2*y4w0;

			cc = A*c1b2*w4w0 + A*a1b2*x4w0 - A*b1c2*w4w0 - A*b1a2*w4x0 + B*b1b2*x4w0
				- B*b1b2*w4x0 + C*b1c2*x4w0 + C*b1a2*x4x0 - C*c1b2*w4x0 - C*a1b2*x4x0;

			ff = A*c1a2*y4x0 + A*c1b2*y4y0 - A*a1c2*x4y0 - A*b1c2*y4y0 - B*c1a2*x4x0
				- B*c1b2*x4y0 + B*a1c2*x4x0 + B*b1c2*y4x0 - C*c1c2*x4y0 + C*c1c2*y4x0;

			bb = A*c1a2*w4w0 + A*a1a2*x4w0 - A*a1b2*y4w0 - A*a1c2*w4w0 - A*a1a2*w4x0
				+ A*b1a2*w4y0 + B*b1a2*x4w0 - B*b1b2*y4w0 - B*c1b2*w4w0 - B*a1b2*w4x0
				+ B*b1b2*w4y0 + B*b1c2*w4w0 - C*b1c2*y4w0 - C*b1a2*x4y0 - C*b1a2*y4x0
				- C*c1a2*w4x0 + C*c1b2*w4y0 + C*a1b2*x4y0 + C*a1b2*y4x0 + C*a1c2*x4w0;

			dd = -A*c1a2*y4w0 + A*a1a2*y4x0 + A*a1b2*y4y0 + A*a1c2*w4y0 - A*a1a2*x4y0
				- A*b1a2*y4y0 + B*b1a2*y4x0 + B*c1a2*w4x0 + B*c1a2*x4w0 + B*c1b2*w4y0
				- B*a1b2*x4y0 - B*a1c2*w4x0 - B*a1c2*x4w0 - B*b1c2*y4w0 + C*b1c2*y4y0
				+ C*c1c2*w4y0 - C*c1a2*x4y0 - C*c1b2*y4y0 - C*c1c2*y4w0 + C*a1c2*y4x0;

			ee = -A*c1a2*w4x0 - A*c1b2*y4w0 - A*c1b2*w4y0 - A*a1b2*x4y0 + A*a1c2*x4w0
				+ A*b1c2*y4w0 + A*b1c2*w4y0 + A*b1a2*y4x0 - B*b1a2*x4x0 - B*b1b2*x4y0
				+ B*c1b2*x4w0 + B*a1b2*x4x0 + B*b1b2*y4x0 - B*b1c2*w4x0 - C*b1c2*x4y0
				+ C*c1c2*x4w0 + C*c1a2*x4x0 + C*c1b2*y4x0 - C*c1c2*w4x0 - C*a1c2*x4x0;

			EllipseAlgebraicParameters params = { aa,bb,cc,dd,ee,ff };
			return params;
		}
	};

	template<typename tFloat>
	struct EllipseParameters
	{
	public:
		/// <summary>	The x-coordinate of the center of the ellipse. </summary>
		tFloat x0;

		/// <summary>	The y-coordinate of the center of the ellipse. </summary>
		tFloat y0;

		/// <summary>	The length of the semi-minor axis. </summary>
		tFloat a;

		/// <summary>	The length of the semi-major axis. </summary>
		tFloat b;

		/// <summary>	The angle  between the x-axis and the major-axis. < / summary>
		tFloat theta;

		bool IsValid() const
		{
			return !std::isnan(this->x0);
		}

		static EllipseParameters<tFloat> Invalid()
		{
			return  EllipseParameters<tFloat>
			{
				std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN(), std::numeric_limits<tFloat>::quiet_NaN()
			};
		}

		static EllipseParameters<tFloat> FromAlgebraicParameters(const EllipseAlgebraicParameters<tFloat>& algebraic)
		{
			if (!algebraic.IsEllipse())
				return Invalid();

			EllipseParameters<tFloat> p;
			tFloat s = 4.0 * algebraic.a*algebraic.c - algebraic.b*algebraic.b;
			tFloat sigma = 4.0 * (algebraic.c*algebraic.d*algebraic.d + algebraic.a*algebraic.e*algebraic.e - algebraic.b*algebraic.d*algebraic.e - algebraic.f*(4.0 * algebraic.a*algebraic.c - algebraic.b*algebraic.b)) / (s*s);

			tFloat num = (4.0 * algebraic.a*algebraic.c - algebraic.b*algebraic.b);
			p.x0 = (algebraic.b*algebraic.e - 2.0 * algebraic.c*algebraic.d) / num;
			p.y0 = (algebraic.b*algebraic.d - 2.0 * algebraic.a*algebraic.e) / num;

			s = sigma*algebraic.a - sigma*algebraic.c;
			s *= s;
			tFloat sigmabsquared = sigma*algebraic.b;
			sigmabsquared *= sigmabsquared;
			tFloat v1 = sigma*algebraic.a + sigma*algebraic.c + sqrt(s + sigmabsquared);
			tFloat v2 = sigma*algebraic.a + sigma*algebraic.c - sqrt(s + sigmabsquared);

			p.a = sqrt(fabs((sigma*algebraic.a + sigma*algebraic.c + sqrt(s + sigmabsquared)) / 2.0));
			p.b = sqrt(fabs((sigma*algebraic.a + sigma*algebraic.c - sqrt(s + sigmabsquared)) / 2.0));

			p.theta = atan2(algebraic.b, algebraic.a - algebraic.c) / 2.0;

			// that's the angle between the x-axis and the ellipse's major axis
			// all we have to do is to check what the major axis is...
			if (sigma > 0.0)
			{
				p.theta += 1.570764827;
			}

			return p;
		}
	};

	template<typename tFloat>
	class LeastSquareEllipseFitter
	{
	public:
		class PointAccessorFromTwoVectors
		{
		private:
			const std::vector<tFloat>& pointsX;
			const std::vector<tFloat>& pointsY;
		public:
			PointAccessorFromTwoVectors(const std::vector<tFloat>& pointsX, const std::vector<tFloat>& pointsY)
				: pointsX(pointsX), pointsY(pointsY)
			{}

			size_t GetLength() const
			{
				return this->pointsX.size();
			}

			tFloat GetX(size_t index) const
			{
				return this->pointsX[index];
			}

			tFloat GetY(size_t index) const
			{
				return this->pointsY[index];
			}
		};

		class PointAccessorFromTwoArrays
		{
		private:
			const double* ptrX;
			const double* ptrY;
			size_t count;
		public:
			PointAccessorFromTwoArrays(const double* ptrX, const double* ptrY, size_t count)
				: ptrX(ptrX), ptrY(ptrY), count(count)
			{}


			size_t GetLength() const
			{
				return this->count;
			}

			tFloat GetX(size_t index) const
			{
				return this->ptrX[index];
			}

			tFloat GetY(size_t index) const
			{
				return this->ptrY[index];
			}
		};

		template <typename PointAccessor>
		static EllipseAlgebraicParameters<tFloat> Fit(const PointAccessor& ptAccessor)
		{
			auto numOfPoints = ptAccessor.GetLength();
			tFloat mx, my; tFloat minX, minY, maxX, maxY;
			CalcMeanMinMax([&](int index)->tFloat {return ptAccessor.GetX(index); }, ptAccessor.GetLength(), mx, minX, maxX);
			CalcMeanMinMax([&](int index)->tFloat {return ptAccessor.GetY(index); }, ptAccessor.GetLength(), my, minY, maxY);
			tFloat sx = (maxX - minX) / 2;
			tFloat sy = (maxY - minY) / 2;

			tFloat scatterM[6 * 6];
			for (int r = 0; r < 6; ++r)
			{
				for (int c = 0; c < 6; ++c)
				{
					tFloat v = 0;
					for (size_t k = 0; k < numOfPoints; ++k)
					{
						tFloat x = ptAccessor.GetX(k); tFloat y = ptAccessor.GetY(k);
						x = (x - mx) / sx;
						y = (y - my) / sy;
						tFloat v1 = CalcDesignMatrixValue(r, x, y);
						tFloat v2 = CalcDesignMatrixValue(c, x, y);
						v += v1*v2;
					}

					scatterM[r * 6 + c] = v;
				}
			}

			tFloat tmpBtimestmpE[3 * 3];
			CalcTmpBtimesTmpE(scatterM + 3, 6 * sizeof(tFloat), scatterM + (3 * 6) + 3, 6 * sizeof(tFloat), tmpBtimestmpE);

			tFloat testA[3*3];
			CalcTestA(scatterM, 6 * sizeof(tFloat), scatterM + 3, 6 * sizeof(tFloat), scatterM + (3 * 6) + 3, 6 * sizeof(tFloat), testA);

			Eigen::EigenSolver<Eigen::Matrix<tFloat, 3, 3>> eigenSolver;

			Eigen::Matrix<tFloat, 3, 3> m;
			m(0, 0) = testA[0];
			m(0, 1) = testA[1];
			m(0, 2) = testA[2];
			m(1, 0) = testA[3];
			m(1, 1) = testA[4];
			m(1, 2) = testA[5];
			m(2, 0) = testA[6];
			m(2, 1) = testA[7];
			m(2, 2) = testA[8];

			eigenSolver.compute(m, true);

			int indexPositiveEigenValue = -1;
			auto eigenval = eigenSolver.eigenvalues();
			for (int i = 0; i < 3; ++i)
			{
				if (eigenval[i].real() < 0)
				{
					indexPositiveEigenValue = i;
					break;
				}
			}

			if (indexPositiveEigenValue == -1)
				return EllipseAlgebraicParameters<tFloat>::Invalid();

			tFloat A[6];
			auto eigenVecs = eigenSolver.eigenvectors();
			A[0] = eigenVecs(0, indexPositiveEigenValue).real();
			A[1] = eigenVecs(1, indexPositiveEigenValue).real();
			A[2] = eigenVecs(2, indexPositiveEigenValue).real();

			CalcLowerHalf(scatterM + 3, 6 * sizeof(tFloat), scatterM + (3 * 6) + 3, 6 * sizeof(tFloat), A, A + 3);

			EllipseAlgebraicParameters<tFloat> params;
			params.a = A[0] * sy*sy;
			params.b = A[1] * sx*sy;
			params.c = A[2] * sx*sx;
			params.d = -2 * A[0] * sy*sy*mx - A[1] * sx*sy*my + A[3] * sx*sy*sy;
			params.e = -A[1] * sx*sy*mx - 2 * A[2] * sx*sx*my + A[4] * sx*sx*sy;
			params.f = A[0] * sy*sy*mx*mx + A[1] * sx*sy*mx*my + A[2] * sx*sx*my*my
						- A[3] * sx*sy*sy*mx - A[4] * sx*sx*sy*my
						+ A[5] * sx*sx*sy*sy;
			return params;
		}

	private:
		static tFloat CalcDesignMatrixValue(int col,tFloat x, tFloat y)
		{
			switch (col)
			{
			case 0:return x*x;
			case 1:return x*y;
			case 2:return y*y;
			case 3:return x;
			case 4:return y;
			case 5:return 1;
			}

			throw std::logic_error("Only expecting col to be in the range 0 to 5.");
		}

		static void CalcMeanMinMax(const std::function<tFloat(int)> getVal, size_t count, tFloat& mean, tFloat& min, tFloat& max)
		{
			min = (std::numeric_limits<tFloat>::max)();
			max = (std::numeric_limits<tFloat>::min)();
			mean = 0;

			for (size_t i = 0; i < count; ++i)
			{
				tFloat value = getVal(i);
				if (value < min)
				{
					min = value;
				}

				if (value > max)
				{
					max = value;
				}

				mean += value;
			}

			mean = mean / count;
		}

		static tFloat squared(tFloat f)
		{
			return f*f;
		}

		static void CalcTmpBtimesTmpE(const tFloat* pB, int strideB, const tFloat* pC, int strideC, tFloat* pDest)
		{
			/* -> Mathematica:
			{{b11, b12, b13}, {b21, b22, b23}, {b31, b32,
			b33}}.(Inverse[{{c11, c12, c13}, {c21, c22, c23}, {c31, c32,
			c33}}].Transpose[{{b11, b12, b13}, {b21, b22, b23}, {b31, b32,
			b33}}]) // Simplify // CForm
			*/

#define b(r,c) *((tFloat*)(((char*)pB)+(r-1)*strideB+(c-1)*sizeof(tFloat)))
#define c(r,c) *((tFloat*)(((char*)pC)+(r-1)*strideC+(c-1)*sizeof(tFloat)))


			pDest[0] =
				(squared(b(1, 3))*(-(c(1, 2)*c(2, 1)) + c(1, 1)*c(2, 2)) + b(1, 3)*(b(1, 1)*(-(c(1, 3)*c(2, 2)) + c(1, 2)*c(2, 3) - c(2, 2)*c(3, 1) + c(2, 1)*c(3, 2)) + b(1, 2)*(c(1, 3)*c(2, 1) + c(1, 2)*c(3, 1) - c(1, 1)*(c(2, 3) + c(3, 2)))) + squared(b(1, 2))*(-(c(1, 3)*c(3, 1)) + c(1, 1)*c(3, 3)) + b(1, 1)*b(1, 2)*(c(2, 3)*c(3, 1) + c(1, 3)*c(3, 2) - (c(1, 2) + c(2, 1))*c(3, 3)) + squared(b(1, 1))*(-(c(2, 3)*c(3, 2)) + c(2, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[1] =
				(b(1, 3) *(-b(2, 3)* c(1, 2)* c(2, 1) + b(2, 3)* c(1, 1)* c(2, 2) + b(2, 2) *c(1, 2) *c(3, 1) - b(2, 1)* c(2, 2)* c(3, 1) -
					b(2, 2)* c(1, 1) *c(3, 2) + b(2, 1)* c(2, 1) *c(3, 2)) +
					b(1, 2) *(b(2, 3)* c(1, 3) *c(2, 1) - b(2, 3)* c(1, 1)* c(2, 3) - b(2, 2)* c(1, 3)* c(3, 1) + b(2, 1)* c(2, 3)* c(3, 1) +
						b(2, 2)* c(1, 1)* c(3, 3) - b(2, 1) *c(2, 1)* c(3, 3)) +
					b(1, 1) *(-b(2, 3)* c(1, 3)* c(2, 2) + b(2, 3)* c(1, 2)* c(2, 3) + b(2, 2)* c(1, 3)* c(3, 2) - b(2, 1)* c(2, 3)* c(3, 2) -
						b(2, 2) *c(1, 2) *c(3, 3) + b(2, 1)* c(2, 2)* c(3, 3))) / (-c(1, 3) *c(2, 2)* c(3, 1) + c(1, 2) *c(2, 3) *c(3, 1) +
							c(1, 3)* c(2, 1) *c(3, 2) - c(1, 1)* c(2, 3) *c(3, 2) - c(1, 2)* c(2, 1)* c(3, 3) + c(1, 1)* c(2, 2)* c(3, 3));

			pDest[2] =
				(b(1, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 3)*c(1, 1)*c(2, 2) + b(3, 2)*c(1, 2)*c(3, 1) - b(3, 1)*c(2, 2)*c(3, 1) - b(3, 2)*c(1, 1)*c(3, 2) + b(3, 1)*c(2, 1)*c(3, 2)) + b(1, 2)*(b(3, 3)*c(1, 3)*c(2, 1) - b(3, 3)*c(1, 1)*c(2, 3) - b(3, 2)*c(1, 3)*c(3, 1) + b(3, 1)*c(2, 3)*c(3, 1) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(2, 1)*c(3, 3)) + b(1, 1)*(-(b(3, 3)*c(1, 3)*c(2, 2)) + b(3, 3)*c(1, 2)*c(2, 3) + b(3, 2)*c(1, 3)*c(3, 2) - b(3, 1)*c(2, 3)*c(3, 2) - b(3, 2)*c(1, 2)*c(3, 3) + b(3, 1)*c(2, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[3] = (b(1, 3)*(-(b(2, 3)*c(1, 2)*c(2, 1)) + b(2, 2)*c(1, 3)*c(2, 1) + b(2, 3)*c(1, 1)*c(2, 2) - b(2, 1)*c(1, 3)*c(2, 2) - b(2, 2)*c(1, 1)*c(2, 3) + b(2, 1)*c(1, 2)*c(2, 3)) + b(1, 2)*(b(2, 3)*c(1, 2)*c(3, 1) - b(2, 2)*c(1, 3)*c(3, 1) - b(2, 3)*c(1, 1)*c(3, 2) + b(2, 1)*c(1, 3)*c(3, 2) + b(2, 2)*c(1, 1)*c(3, 3) - b(2, 1)*c(1, 2)*c(3, 3)) + b(1, 1)*(-(b(2, 3)*c(2, 2)*c(3, 1)) + b(2, 2)*c(2, 3)*c(3, 1) + b(2, 3)*c(2, 1)*c(3, 2) - b(2, 1)*c(2, 3)*c(3, 2) - b(2, 2)*c(2, 1)*c(3, 3) + b(2, 1)*c(2, 2)*c(3, 3))) /
				(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[4] = (squared(b(2, 3))*(-(c(1, 2)*c(2, 1)) + c(1, 1)*c(2, 2)) + b(2, 3)*(b(2, 1)*(-(c(1, 3)*c(2, 2)) + c(1, 2)*c(2, 3) - c(2, 2)*c(3, 1) + c(2, 1)*c(3, 2)) + b(2, 2)*(c(1, 3)*c(2, 1) + c(1, 2)*c(3, 1) - c(1, 1)*(c(2, 3) + c(3, 2)))) + squared(b(2, 2))*(-(c(1, 3)*c(3, 1)) + c(1, 1)*c(3, 3)) + b(2, 1)*b(2, 2)*(c(2, 3)*c(3, 1) + c(1, 3)*c(3, 2) - (c(1, 2) + c(2, 1))*c(3, 3)) + squared(b(2, 1))*(-(c(2, 3)*c(3, 2)) + c(2, 2)*c(3, 3))) /
				(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[5] = (b(2, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 3)*c(1, 1)*c(2, 2) + b(3, 2)*c(1, 2)*c(3, 1) - b(3, 1)*c(2, 2)*c(3, 1) - b(3, 2)*c(1, 1)*c(3, 2) + b(3, 1)*c(2, 1)*c(3, 2)) + b(2, 2)*(b(3, 3)*c(1, 3)*c(2, 1) - b(3, 3)*c(1, 1)*c(2, 3) - b(3, 2)*c(1, 3)*c(3, 1) + b(3, 1)*c(2, 3)*c(3, 1) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(2, 1)*c(3, 3)) +
				b(2, 1)*(-(b(3, 3)*c(1, 3)*c(2, 2)) + b(3, 3)*c(1, 2)*c(2, 3) + b(3, 2)*c(1, 3)*c(3, 2) - b(3, 1)*c(2, 3)*c(3, 2) - b(3, 2)*c(1, 2)*c(3, 3) + b(3, 1)*c(2, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[6] = (b(1, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 2)*c(1, 3)*c(2, 1) + b(3, 3)*c(1, 1)*c(2, 2) - b(3, 1)*c(1, 3)*c(2, 2) - b(3, 2)*c(1, 1)*c(2, 3) + b(3, 1)*c(1, 2)*c(2, 3)) + b(1, 2)*(b(3, 3)*c(1, 2)*c(3, 1) - b(3, 2)*c(1, 3)*c(3, 1) - b(3, 3)*c(1, 1)*c(3, 2) + b(3, 1)*c(1, 3)*c(3, 2) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(1, 2)*c(3, 3)) + b(1, 1)*(-(b(3, 3)*c(2, 2)*c(3, 1)) + b(3, 2)*c(2, 3)*c(3, 1) + b(3, 3)*c(2, 1)*c(3, 2) - b(3, 1)*c(2, 3)*c(3, 2) - b(3, 2)*c(2, 1)*c(3, 3) + b(3, 1)*c(2, 2)*c(3, 3))) /
				(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[7] = (b(2, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 2)*c(1, 3)*c(2, 1) + b(3, 3)*c(1, 1)*c(2, 2) - b(3, 1)*c(1, 3)*c(2, 2) - b(3, 2)*c(1, 1)*c(2, 3) + b(3, 1)*c(1, 2)*c(2, 3)) + b(2, 2)*(b(3, 3)*c(1, 2)*c(3, 1) - b(3, 2)*c(1, 3)*c(3, 1) - b(3, 3)*c(1, 1)*c(3, 2) + b(3, 1)*c(1, 3)*c(3, 2) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(1, 2)*c(3, 3)) +
				b(2, 1)*(-(b(3, 3)*c(2, 2)*c(3, 1)) + b(3, 2)*c(2, 3)*c(3, 1) + b(3, 3)*c(2, 1)*c(3, 2) - b(3, 1)*c(2, 3)*c(3, 2) - b(3, 2)*c(2, 1)*c(3, 3) + b(3, 1)*c(2, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[8] = (squared(b(3, 3))*(-(c(1, 2)*c(2, 1)) + c(1, 1)*c(2, 2)) + b(3, 3)*(b(3, 1)*(-(c(1, 3)*c(2, 2)) + c(1, 2)*c(2, 3) - c(2, 2)*c(3, 1) + c(2, 1)*c(3, 2)) + b(3, 2)*(c(1, 3)*c(2, 1) + c(1, 2)*c(3, 1) - c(1, 1)*(c(2, 3) + c(3, 2)))) + squared(b(3, 2))*(-(c(1, 3)*c(3, 1)) + c(1, 1)*c(3, 3)) + b(3, 1)*b(3, 2)*(c(2, 3)*c(3, 1) + c(1, 3)*c(3, 2) - (c(1, 2) + c(2, 1))*c(3, 3)) + squared(b(3, 1))*(-(c(2, 3)*c(3, 2)) + c(2, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));
#undef b
#undef c
		}

		static void CalcTestA(const tFloat* pA, int strideA, const tFloat* pB, int strideB, const tFloat* pC, int strideC, tFloat* pDest)
		{
			/* Mathematica:

			{{0, 0, -(1/2)}, {0, 1, 0}, {-(1/2), 0,
			0}}.({{a11, a12, a13}, {a21, a22, a23}, {a31, a32,
			a33}} - {{b11, b12, b13}, {b21, b22, b23}, {b31, b32,
			b33}}.(Inverse[{{c11, c12, c13}, {c21, c22, c23}, {c31, c32,
			c33}}].Transpose[{{b11, b12, b13}, {b21, b22, b23}, {b31,
			b32, b33}}])) // Simplify // CForm
			*/
#define a(r,c) *((tFloat*)(((char*)pA)+(r-1)*strideA+(c-1)*sizeof(tFloat)))
#define b(r,c) *((tFloat*)(((char*)pB)+(r-1)*strideB+(c-1)*sizeof(tFloat)))
#define c(r,c) *((tFloat*)(((char*)pC)+(r-1)*strideC+(c-1)*sizeof(tFloat)))
			pDest[0] = (b(1, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 2)*c(1, 3)*c(2, 1) + b(3, 3)*c(1, 1)*c(2, 2) - b(3, 1)*c(1, 3)*c(2, 2) - b(3, 2)*c(1, 1)*c(2, 3) + b(3, 1)*c(1, 2)*c(2, 3)) - b(1, 1)*b(3, 3)*c(2, 2)*c(3, 1) + a(3, 1)*c(1, 3)*c(2, 2)*c(3, 1) + b(1, 1)*b(3, 2)*c(2, 3)*c(3, 1) - a(3, 1)*c(1, 2)*c(2, 3)*c(3, 1) + b(1, 1)*b(3, 3)*c(2, 1)*c(3, 2) - a(3, 1)*c(1, 3)*c(2, 1)*c(3, 2) - b(1, 1)*b(3, 1)*c(2, 3)*c(3, 2) + a(3, 1)*c(1, 1)*c(2, 3)*c(3, 2) - b(1, 1)*b(3, 2)*c(2, 1)*c(3, 3) + a(3, 1)*c(1, 2)*c(2, 1)*c(3, 3) + b(1, 1)*b(3, 1)*c(2, 2)*c(3, 3) - a(3, 1)*c(1, 1)*c(2, 2)*c(3, 3) +
				b(1, 2)*(b(3, 3)*c(1, 2)*c(3, 1) - b(3, 2)*c(1, 3)*c(3, 1) - b(3, 3)*c(1, 1)*c(3, 2) + b(3, 1)*c(1, 3)*c(3, 2) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(1, 2)*c(3, 3))) / (2.*(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3)));

			pDest[1] = (b(2, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 2)*c(1, 3)*c(2, 1) + b(3, 3)*c(1, 1)*c(2, 2) - b(3, 1)*c(1, 3)*c(2, 2) - b(3, 2)*c(1, 1)*c(2, 3) + b(3, 1)*c(1, 2)*c(2, 3)) - b(2, 1)*b(3, 3)*c(2, 2)*c(3, 1) + a(3, 2)*c(1, 3)*c(2, 2)*c(3, 1) + b(2, 1)*b(3, 2)*c(2, 3)*c(3, 1) - a(3, 2)*c(1, 2)*c(2, 3)*c(3, 1) + b(2, 1)*b(3, 3)*c(2, 1)*c(3, 2) - a(3, 2)*c(1, 3)*c(2, 1)*c(3, 2) - b(2, 1)*b(3, 1)*c(2, 3)*c(3, 2) + a(3, 2)*c(1, 1)*c(2, 3)*c(3, 2) - b(2, 1)*b(3, 2)*c(2, 1)*c(3, 3) + a(3, 2)*c(1, 2)*c(2, 1)*c(3, 3) + b(2, 1)*b(3, 1)*c(2, 2)*c(3, 3) - a(3, 2)*c(1, 1)*c(2, 2)*c(3, 3) +
				b(2, 2)*(b(3, 3)*c(1, 2)*c(3, 1) - b(3, 2)*c(1, 3)*c(3, 1) - b(3, 3)*c(1, 1)*c(3, 2) + b(3, 1)*c(1, 3)*c(3, 2) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(1, 2)*c(3, 3))) / (2.*(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3)));

			pDest[2] = (squared(b(3, 3))*(-(c(1, 2)*c(2, 1)) + c(1, 1)*c(2, 2)) + a(3, 3)*c(1, 3)*c(2, 2)*c(3, 1) - a(3, 3)*c(1, 2)*c(2, 3)*c(3, 1) - a(3, 3)*c(1, 3)*c(2, 1)*c(3, 2) - squared(b(3, 1))*c(2, 3)*c(3, 2) + a(3, 3)*c(1, 1)*c(2, 3)*c(3, 2) + b(3, 3)*(b(3, 1)*(-(c(1, 3)*c(2, 2)) + c(1, 2)*c(2, 3) - c(2, 2)*c(3, 1) + c(2, 1)*c(3, 2)) + b(3, 2)*(c(1, 3)*c(2, 1) + c(1, 2)*c(3, 1) - c(1, 1)*(c(2, 3) + c(3, 2)))) + a(3, 3)*c(1, 2)*c(2, 1)*c(3, 3) + squared(b(3, 1))*c(2, 2)*c(3, 3) - a(3, 3)*c(1, 1)*c(2, 2)*c(3, 3) + squared(b(3, 2))*(-(c(1, 3)*c(3, 1)) + c(1, 1)*c(3, 3)) +
				b(3, 1)*b(3, 2)*(c(2, 3)*c(3, 1) + c(1, 3)*c(3, 2) - (c(1, 2) + c(2, 1))*c(3, 3))) / (2.*(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3)));

			pDest[3] = (b(1, 3)*(b(2, 3)*c(1, 2)*c(2, 1) - b(2, 2)*c(1, 3)*c(2, 1) - b(2, 3)*c(1, 1)*c(2, 2) + b(2, 1)*c(1, 3)*c(2, 2) + b(2, 2)*c(1, 1)*c(2, 3) - b(2, 1)*c(1, 2)*c(2, 3)) + b(1, 1)*b(2, 3)*c(2, 2)*c(3, 1) - a(2, 1)*c(1, 3)*c(2, 2)*c(3, 1) - b(1, 1)*b(2, 2)*c(2, 3)*c(3, 1) + a(2, 1)*c(1, 2)*c(2, 3)*c(3, 1) - b(1, 1)*b(2, 3)*c(2, 1)*c(3, 2) + a(2, 1)*c(1, 3)*c(2, 1)*c(3, 2) + b(1, 1)*b(2, 1)*c(2, 3)*c(3, 2) - a(2, 1)*c(1, 1)*c(2, 3)*c(3, 2) + b(1, 1)*b(2, 2)*c(2, 1)*c(3, 3) - a(2, 1)*c(1, 2)*c(2, 1)*c(3, 3) - b(1, 1)*b(2, 1)*c(2, 2)*c(3, 3) + a(2, 1)*c(1, 1)*c(2, 2)*c(3, 3) +
				b(1, 2)*(-(b(2, 3)*c(1, 2)*c(3, 1)) + b(2, 2)*c(1, 3)*c(3, 1) + b(2, 3)*c(1, 1)*c(3, 2) - b(2, 1)*c(1, 3)*c(3, 2) - b(2, 2)*c(1, 1)*c(3, 3) + b(2, 1)*c(1, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[4] = (squared(b(2, 3))*(-(c(1, 2)*c(2, 1)) + c(1, 1)*c(2, 2)) + a(2, 2)*c(1, 3)*c(2, 2)*c(3, 1) - a(2, 2)*c(1, 2)*c(2, 3)*c(3, 1) - a(2, 2)*c(1, 3)*c(2, 1)*c(3, 2) - squared(b(2, 1))*c(2, 3)*c(3, 2) + a(2, 2)*c(1, 1)*c(2, 3)*c(3, 2) + b(2, 3)*(b(2, 1)*(-(c(1, 3)*c(2, 2)) + c(1, 2)*c(2, 3) - c(2, 2)*c(3, 1) + c(2, 1)*c(3, 2)) + b(2, 2)*(c(1, 3)*c(2, 1) + c(1, 2)*c(3, 1) - c(1, 1)*(c(2, 3) + c(3, 2)))) + a(2, 2)*c(1, 2)*c(2, 1)*c(3, 3) + squared(b(2, 1))*c(2, 2)*c(3, 3) - a(2, 2)*c(1, 1)*c(2, 2)*c(3, 3) + squared(b(2, 2))*(-(c(1, 3)*c(3, 1)) + c(1, 1)*c(3, 3)) +
				b(2, 1)*b(2, 2)*(c(2, 3)*c(3, 1) + c(1, 3)*c(3, 2) - (c(1, 2) + c(2, 1))*c(3, 3))) / (c(1, 3)*c(2, 2)*c(3, 1) - c(1, 2)*c(2, 3)*c(3, 1) - c(1, 3)*c(2, 1)*c(3, 2) + c(1, 1)*c(2, 3)*c(3, 2) + c(1, 2)*c(2, 1)*c(3, 3) - c(1, 1)*c(2, 2)*c(3, 3));

			pDest[5] = (b(2, 1)*b(3, 3)*c(1, 3)*c(2, 2) - b(2, 1)*b(3, 3)*c(1, 2)*c(2, 3) - a(2, 3)*c(1, 3)*c(2, 2)*c(3, 1) + a(2, 3)*c(1, 2)*c(2, 3)*c(3, 1) - b(2, 1)*b(3, 2)*c(1, 3)*c(3, 2) + a(2, 3)*c(1, 3)*c(2, 1)*c(3, 2) + b(2, 1)*b(3, 1)*c(2, 3)*c(3, 2) - a(2, 3)*c(1, 1)*c(2, 3)*c(3, 2) +
				b(2, 3)*(b(3, 3)*c(1, 2)*c(2, 1) - b(3, 3)*c(1, 1)*c(2, 2) - b(3, 2)*c(1, 2)*c(3, 1) + b(3, 1)*c(2, 2)*c(3, 1) + b(3, 2)*c(1, 1)*c(3, 2) - b(3, 1)*c(2, 1)*c(3, 2)) + b(2, 1)*b(3, 2)*c(1, 2)*c(3, 3) - a(2, 3)*c(1, 2)*c(2, 1)*c(3, 3) - b(2, 1)*b(3, 1)*c(2, 2)*c(3, 3) + a(2, 3)*c(1, 1)*c(2, 2)*c(3, 3) + b(2, 2)*(-(b(3, 3)*c(1, 3)*c(2, 1)) + b(3, 3)*c(1, 1)*c(2, 3) + b(3, 2)*c(1, 3)*c(3, 1) - b(3, 1)*c(2, 3)*c(3, 1) - b(3, 2)*c(1, 1)*c(3, 3) + b(3, 1)*c(2, 1)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));

			pDest[6] = (squared(b(1, 3))*(-(c(1, 2)*c(2, 1)) + c(1, 1)*c(2, 2)) + a(1, 1)*c(1, 3)*c(2, 2)*c(3, 1) - a(1, 1)*c(1, 2)*c(2, 3)*c(3, 1) - a(1, 1)*c(1, 3)*c(2, 1)*c(3, 2) - squared(b(1, 1))*c(2, 3)*c(3, 2) + a(1, 1)*c(1, 1)*c(2, 3)*c(3, 2) + b(1, 3)*(b(1, 1)*(-(c(1, 3)*c(2, 2)) + c(1, 2)*c(2, 3) - c(2, 2)*c(3, 1) + c(2, 1)*c(3, 2)) + b(1, 2)*(c(1, 3)*c(2, 1) + c(1, 2)*c(3, 1) - c(1, 1)*(c(2, 3) + c(3, 2)))) + a(1, 1)*c(1, 2)*c(2, 1)*c(3, 3) + squared(b(1, 1))*c(2, 2)*c(3, 3) - a(1, 1)*c(1, 1)*c(2, 2)*c(3, 3) + squared(b(1, 2))*(-(c(1, 3)*c(3, 1)) + c(1, 1)*c(3, 3)) +
				b(1, 1)*b(1, 2)*(c(2, 3)*c(3, 1) + c(1, 3)*c(3, 2) - (c(1, 2) + c(2, 1))*c(3, 3))) / (2.*(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3)));

			pDest[7] = (-(b(1, 1)*b(2, 3)*c(1, 3)*c(2, 2)) + b(1, 1)*b(2, 3)*c(1, 2)*c(2, 3) + a(1, 2)*c(1, 3)*c(2, 2)*c(3, 1) - a(1, 2)*c(1, 2)*c(2, 3)*c(3, 1) + b(1, 1)*b(2, 2)*c(1, 3)*c(3, 2) - a(1, 2)*c(1, 3)*c(2, 1)*c(3, 2) - b(1, 1)*b(2, 1)*c(2, 3)*c(3, 2) + a(1, 2)*c(1, 1)*c(2, 3)*c(3, 2) +
				b(1, 3)*(-(b(2, 3)*c(1, 2)*c(2, 1)) + b(2, 3)*c(1, 1)*c(2, 2) + b(2, 2)*c(1, 2)*c(3, 1) - b(2, 1)*c(2, 2)*c(3, 1) - b(2, 2)*c(1, 1)*c(3, 2) + b(2, 1)*c(2, 1)*c(3, 2)) - b(1, 1)*b(2, 2)*c(1, 2)*c(3, 3) + a(1, 2)*c(1, 2)*c(2, 1)*c(3, 3) + b(1, 1)*b(2, 1)*c(2, 2)*c(3, 3) - a(1, 2)*c(1, 1)*c(2, 2)*c(3, 3) + b(1, 2)*(b(2, 3)*c(1, 3)*c(2, 1) - b(2, 3)*c(1, 1)*c(2, 3) - b(2, 2)*c(1, 3)*c(3, 1) + b(2, 1)*c(2, 3)*c(3, 1) + b(2, 2)*c(1, 1)*c(3, 3) - b(2, 1)*c(2, 1)*c(3, 3))) / (2.*(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3)));

			pDest[8] = (-(b(1, 1)*b(3, 3)*c(1, 3)*c(2, 2)) + b(1, 1)*b(3, 3)*c(1, 2)*c(2, 3) + a(1, 3)*c(1, 3)*c(2, 2)*c(3, 1) - a(1, 3)*c(1, 2)*c(2, 3)*c(3, 1) + b(1, 1)*b(3, 2)*c(1, 3)*c(3, 2) - a(1, 3)*c(1, 3)*c(2, 1)*c(3, 2) - b(1, 1)*b(3, 1)*c(2, 3)*c(3, 2) + a(1, 3)*c(1, 1)*c(2, 3)*c(3, 2) + b(1, 3)*(-(b(3, 3)*c(1, 2)*c(2, 1)) + b(3, 3)*c(1, 1)*c(2, 2) + b(3, 2)*c(1, 2)*c(3, 1) - b(3, 1)*c(2, 2)*c(3, 1) - b(3, 2)*c(1, 1)*c(3, 2) + b(3, 1)*c(2, 1)*c(3, 2)) - b(1, 1)*b(3, 2)*c(1, 2)*c(3, 3) + a(1, 3)*c(1, 2)*c(2, 1)*c(3, 3) + b(1, 1)*b(3, 1)*c(2, 2)*c(3, 3) - a(1, 3)*c(1, 1)*c(2, 2)*c(3, 3) +
				b(1, 2)*(b(3, 3)*c(1, 3)*c(2, 1) - b(3, 3)*c(1, 1)*c(2, 3) - b(3, 2)*c(1, 3)*c(3, 1) + b(3, 1)*c(2, 3)*c(3, 1) + b(3, 2)*c(1, 1)*c(3, 3) - b(3, 1)*c(2, 1)*c(3, 3))) / (2.*(-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3)));

#undef c
#undef b
#undef a
		}

		static void CalcLowerHalf(const tFloat* pB, int strideB, const tFloat* pC, int strideC, const tFloat* ptrAUpperHalf, tFloat* ptrDest)
		{
#define b(r,c) *((tFloat*)(((char*)pB)+(r-1)*strideB+(c-1)*sizeof(tFloat)))
#define c(r,c) *((tFloat*)(((char*)pC)+(r-1)*strideC+(c-1)*sizeof(tFloat)))

			tFloat a1 = ptrAUpperHalf[0]; tFloat a2 = ptrAUpperHalf[1]; tFloat a3 = ptrAUpperHalf[2];
			ptrDest[0] = (a1*(b(1, 3)*c(1, 3)*c(2, 2) - b(1, 3)*c(1, 2)*c(2, 3) - b(1, 2)*c(1, 3)*c(3, 2) + b(1, 1)*c(2, 3)*c(3, 2) + b(1, 2)*c(1, 2)*c(3, 3) - b(1, 1)*c(2, 2)*c(3, 3)) + a2*(b(2, 3)*c(1, 3)*c(2, 2) - b(2, 3)*c(1, 2)*c(2, 3) - b(2, 2)*c(1, 3)*c(3, 2) + b(2, 1)*c(2, 3)*c(3, 2) + b(2, 2)*c(1, 2)*c(3, 3) - b(2, 1)*c(2, 2)*c(3, 3)) + a3*(b(3, 3)*c(1, 3)*c(2, 2) - b(3, 3)*c(1, 2)*c(2, 3) - b(3, 2)*c(1, 3)*c(3, 2) + b(3, 1)*c(2, 3)*c(3, 2) + b(3, 2)*c(1, 2)*c(3, 3) - b(3, 1)*c(2, 2)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));
			ptrDest[1] = (a1*(-(b(1, 3)*c(1, 3)*c(2, 1)) + b(1, 3)*c(1, 1)*c(2, 3) + b(1, 2)*c(1, 3)*c(3, 1) - b(1, 1)*c(2, 3)*c(3, 1) - b(1, 2)*c(1, 1)*c(3, 3) + b(1, 1)*c(2, 1)*c(3, 3)) + a2*(-(b(2, 3)*c(1, 3)*c(2, 1)) + b(2, 3)*c(1, 1)*c(2, 3) + b(2, 2)*c(1, 3)*c(3, 1) - b(2, 1)*c(2, 3)*c(3, 1) - b(2, 2)*c(1, 1)*c(3, 3) + b(2, 1)*c(2, 1)*c(3, 3)) + a3*(-(b(3, 3)*c(1, 3)*c(2, 1)) + b(3, 3)*c(1, 1)*c(2, 3) + b(3, 2)*c(1, 3)*c(3, 1) - b(3, 1)*c(2, 3)*c(3, 1) - b(3, 2)*c(1, 1)*c(3, 3) + b(3, 1)*c(2, 1)*c(3, 3))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));
			ptrDest[2] = (a1*(b(1, 3)*c(1, 2)*c(2, 1) - b(1, 3)*c(1, 1)*c(2, 2) - b(1, 2)*c(1, 2)*c(3, 1) + b(1, 1)*c(2, 2)*c(3, 1) + b(1, 2)*c(1, 1)*c(3, 2) - b(1, 1)*c(2, 1)*c(3, 2)) + a2*(b(2, 3)*c(1, 2)*c(2, 1) - b(2, 3)*c(1, 1)*c(2, 2) - b(2, 2)*c(1, 2)*c(3, 1) + b(2, 1)*c(2, 2)*c(3, 1) + b(2, 2)*c(1, 1)*c(3, 2) - b(2, 1)*c(2, 1)*c(3, 2)) + a3*(b(3, 3)*c(1, 2)*c(2, 1) - b(3, 3)*c(1, 1)*c(2, 2) - b(3, 2)*c(1, 2)*c(3, 1) + b(3, 1)*c(2, 2)*c(3, 1) + b(3, 2)*c(1, 1)*c(3, 2) - b(3, 1)*c(2, 1)*c(3, 2))) / (-(c(1, 3)*c(2, 2)*c(3, 1)) + c(1, 2)*c(2, 3)*c(3, 1) + c(1, 3)*c(2, 1)*c(3, 2) - c(1, 1)*c(2, 3)*c(3, 2) - c(1, 2)*c(2, 1)*c(3, 3) + c(1, 1)*c(2, 2)*c(3, 3));
#undef b
#undef c
		}
	};
}