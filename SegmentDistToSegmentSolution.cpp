#pragma once
#ifndef STOSSOLVER_CPP
#define STOSSOLVER_CPP

#include "libs/glm/glm.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "libs/glm/gtx/compatibility.hpp"
#include "libs/glm/gtx/intersect.hpp"
//#include <glm/gtx/intersect.inl>

#define KINDA_SMALL_NUMBER (1.e-4f)
#define SMALL_NUMBER (1.e-8f)

namespace csd
{

	glm::vec3 GetSafeNormal(glm::vec3 v, float Tolerance = SMALL_NUMBER) //const
	{

		const float SquareSum = v.x * v.x + v.y * v.y + v.z * v.z;

		// Not sure if it's safe to add tolerance in there. Might introduce too many errors
		if (SquareSum == 1.f)
		{
			return v;
		}
		else if (SquareSum < Tolerance)
		{
			return glm::vec3(0.0f, 0.0f, 0.0f);
		}
		const float Scale = glm::inversesqrt(SquareSum);
		return glm::vec3(v.x * Scale, v.y * Scale, v.z * Scale);
	}

	bool IsZero(glm::vec3 v) //const
	{
		return v.x == 0.f && v.y == 0.f && v.z == 0.f;
	}

	/////

	glm::vec3 ClosestPointOnSegment(const glm::vec3 &Point, const glm::vec3 &StartPoint, const glm::vec3 &EndPoint)
	{
		const glm::vec3 Segment = EndPoint - StartPoint;
		const glm::vec3 VectToPoint = Point - StartPoint;

		// See if closest point is before StartPoint
		const float Dot1 = glm::dot(VectToPoint, Segment);
		if (Dot1 <= 0)
		{
			return StartPoint;
		}

		// See if closest point is beyond EndPoint
		const float Dot2 = glm::dot(Segment, Segment);
		if (Dot2 <= Dot1)
		{
			return EndPoint;
		}

		// Closest Point is within segment
		return StartPoint + Segment * (Dot1 / Dot2);
	}

	/////

	struct SegmentDistToSegment_Solver
	{
		SegmentDistToSegment_Solver(const glm::vec3 &InA1, const glm::vec3 &InB1, const glm::vec3 &InA2, const glm::vec3 &InB2) : bLinesAreNearlyParallel(false),
																																  A1(InA1),
																																  A2(InA2),
																																  S1(InB1 - InA1),
																																  S2(InB2 - InA2),
																																  S3(InA1 - InA2)
		{
		}

		bool bLinesAreNearlyParallel;

		const glm::vec3 &A1;
		const glm::vec3 &A2;

		const glm::vec3 S1;
		const glm::vec3 S2;
		const glm::vec3 S3;

		void Solve(glm::vec3 &OutP1, glm::vec3 &OutP2)
		{

			const float Dot11 = glm::dot(S1, S1);
			const float Dot12 = glm::dot(S1, S2);
			const float Dot13 = glm::dot(S1, S3);
			const float Dot22 = glm::dot(S2, S2);
			const float Dot23 = glm::dot(S2, S3);

			const float D = Dot11 * Dot22 - Dot12 * Dot12;

			float D1 = D;
			float D2 = D;

			float N1;
			float N2;

			if (bLinesAreNearlyParallel || D < KINDA_SMALL_NUMBER)
			{
				// the lines are almost parallel
				N1 = 0.f; // force using point A on segment S1
				D1 = 1.f; // to prevent possible division by 0 later
				N2 = Dot23;
				D2 = Dot22;
			}
			else
			{
				// get the closest points on the infinite lines
				N1 = (Dot12 * Dot23 - Dot22 * Dot13);
				N2 = (Dot11 * Dot23 - Dot12 * Dot13);

				if (N1 < 0.f)
				{
					// t1 < 0.f => the s==0 edge is visible
					N1 = 0.f;
					N2 = Dot23;
					D2 = Dot22;
				}
				else if (N1 > D1)
				{
					// t1 > 1 => the t1==1 edge is visible
					N1 = D1;
					N2 = Dot23 + Dot12;
					D2 = Dot22;
				}
			}

			if (N2 < 0.f)
			{
				// t2 < 0 => the t2==0 edge is visible
				N2 = 0.f;

				// recompute t1 for this edge
				if (-Dot13 < 0.f)
				{
					N1 = 0.f;
				}
				else if (-Dot13 > Dot11)
				{
					N1 = D1;
				}
				else
				{
					N1 = -Dot13;
					D1 = Dot11;
				}
			}
			else if (N2 > D2)
			{
				// t2 > 1 => the t2=1 edge is visible
				N2 = D2;

				// recompute t1 for this edge
				if ((-Dot13 + Dot12) < 0.f)
				{
					N1 = 0.f;
				}
				else if ((-Dot13 + Dot12) > Dot11)
				{
					N1 = D1;
				}
				else
				{
					N1 = (-Dot13 + Dot12);
					D1 = Dot11;
				}
			}

			// finally do the division to get the points' location
			const float T1 = (glm::abs(N1) < KINDA_SMALL_NUMBER ? 0.f : N1 / D1); ///can use std abs
			const float T2 = (glm::abs(N2) < KINDA_SMALL_NUMBER ? 0.f : N2 / D2);

			// return the closest points
			OutP1 = A1 + T1 * S1;
			OutP2 = A2 + T2 * S2;
		}
	};

	void SegmentDistToSegmentSafe(glm::vec3 A1, glm::vec3 B1, glm::vec3 A2, glm::vec3 B2, glm::vec3 &OutP1, glm::vec3 &OutP2)
	{
		SegmentDistToSegment_Solver Solver(A1, B1, A2, B2);

		const glm::vec3 S1_norm = GetSafeNormal(Solver.S1);
		const glm::vec3 S2_norm = GetSafeNormal(Solver.S2);

		const bool bS1IsPoint = IsZero(S1_norm);
		const bool bS2IsPoint = IsZero(S2_norm);

		if (bS1IsPoint && bS2IsPoint)
		{
			OutP1 = A1;
			OutP2 = A2;
		}
		else if (bS2IsPoint)
		{
			OutP1 = ClosestPointOnSegment(A2, A1, B1);
			OutP2 = A2;
		}
		else if (bS1IsPoint)
		{
			OutP1 = A1;
			OutP2 = ClosestPointOnSegment(A1, A2, B2);
		}
		else
		{
			const float Dot11_norm = glm::dot(S1_norm, S1_norm); // always >= 0
			const float Dot22_norm = glm::dot(S2_norm, S2_norm); // always >= 0
			const float Dot12_norm = glm::dot(S1_norm, S2_norm);
			const float D_norm = Dot11_norm * Dot22_norm - Dot12_norm * Dot12_norm; // always >= 0

			Solver.bLinesAreNearlyParallel = D_norm < KINDA_SMALL_NUMBER;
			Solver.Solve(OutP1, OutP2);
		}
	}

	void SegmentDistToSegment(glm::vec3 A1, glm::vec3 B1, glm::vec3 A2, glm::vec3 B2, glm::vec3 &OutP1, glm::vec3 &OutP2)
	{
		SegmentDistToSegment_Solver(A1, B1, A2, B2).Solve(OutP1, OutP2);
	}

}

#endif ///STOSSOLVER_CPP